# -*- coding: utf-8 -*-
"""管件硬模成型 —— 极简 Web 服务 (web_box_generator/app.py)

只做三件事（**没有三维查看**，不加载 three.js）：
    1. 上传产品数模（STEP / IGES / STL）
    2. 生成管件模具 -> 上模 / 下模 / 芯棒 / 四件套装配体，逐个下载
    3. 镜像 / 旋转模型 -> 下载；另可"只算推荐分模面高度"（不跑布尔）

启动：
    venv\\Scripts\\python app.py --port 5002
或双击项目根目录的 启动Web服务.bat，浏览器打开 http://127.0.0.1:5002

接口一览：
    GET  /                       页面
    POST /api/upload             上传数模 -> {file, original_name, part_number, size_mb}
    POST /api/mold_generate      开工（异步）-> {task_id}
    POST /api/pipe_plane_suggest 只算推荐分模面高度（异步）-> {task_id}
    GET  /api/task_status?id=    轮询进度 / 结果 / 日志
    GET  /api/download?task=&file=  下载生成件（只允许任务自己的输出目录）
    POST /api/generate_mirror    镜像/旋转（同步）-> {name, url, ...}
    GET  /api/download_mirror?name= 下载镜像/旋转结果
"""

from __future__ import annotations

import argparse
import contextlib
import io as _io
import json
import os
import re
import sys
import threading
import time
import traceback
import uuid

from flask import Flask, jsonify, render_template, request, send_file

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from io_utils import ensure_utf8_stdout                 # noqa: E402
from naming_utils import part_number_from_filename      # noqa: E402

ensure_utf8_stdout()

app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = 300 * 1024 * 1024      # 单文件上限 300 MB

UPLOAD_DIR = os.path.join(_HERE, "uploads")
OUT_ROOT = os.path.join(UPLOAD_DIR, "output")
MIRROR_DIR = os.path.join(UPLOAD_DIR, "mirror_output")
for _d in (UPLOAD_DIR, OUT_ROOT, MIRROR_DIR):
    os.makedirs(_d, exist_ok=True)

ALLOWED_EXTS = (".stp", ".step", ".iges", ".igs", ".stl")
_ERROR_LOG = os.path.join(_HERE, "error.log")


# --------------------------------------------------------------------------- 工具
def _log_error(text: str) -> None:
    """把异常写进 error.log（同时打一份到控制台），方便事后排查。"""
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    try:
        with open(_ERROR_LOG, "a", encoding="utf-8") as fh:
            fh.write(f"\n===== {stamp} =====\n{text}\n")
    except OSError:
        pass
    try:
        print(f"[错误] {text.splitlines()[0] if text else ''}", flush=True)
    except Exception:  # noqa: BLE001
        pass


def _json_safe(obj):
    """把 numpy / OCC 之类"不是标准 JSON 类型"的东西转成普通类型。"""
    if obj is None or isinstance(obj, (bool, int, str)):
        return obj
    if isinstance(obj, float):
        return obj if obj == obj and abs(obj) != float("inf") else None
    if isinstance(obj, dict):
        return {str(k): _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set)):
        return [_json_safe(v) for v in obj]
    try:                                     # numpy 标量/数组等
        import numpy as np

        if isinstance(obj, np.ndarray):
            return _json_safe(obj.tolist())
        if isinstance(obj, np.generic):
            return _json_safe(obj.item())
    except Exception:  # noqa: BLE001
        pass
    return str(obj)


def _safe_name(name: str, fallback: str = "part") -> str:
    """文件名安全化（去掉路径分隔符与 Windows 非法字符）。"""
    name = os.path.basename(str(name or "")).strip()
    name = re.sub(r'[\\/:*?"<>|\r\n\t]+', "_", name)
    return name or fallback


def _uploaded_path(name: str) -> str:
    """把"上传文件名"解析成 uploads 目录下的绝对路径（越界返回空串）。"""
    safe = os.path.basename(str(name or ""))
    if not safe:
        return ""
    path = os.path.join(UPLOAD_DIR, safe)
    if os.path.isfile(path):
        return path
    return ""


def _human_size(path: str) -> float:
    try:
        return round(os.path.getsize(path) / 1048576.0, 2)
    except OSError:
        return 0.0


# --------------------------------------------------------------------------- 异步任务
_TASKS: dict = {}
_TASK_KEEP = 12
_NOISE = ("******", " Step File Name", "** WorkSession", "** Transferring")


def _task_new(label: str) -> str:
    tid = uuid.uuid4().hex[:12]
    _TASKS[tid] = {"id": tid, "label": label, "state": "running",
                   "t0": time.time(), "t1": None, "log": _io.StringIO(),
                   "result": None, "error": None}
    # 只保留最近若干个已完成任务
    while len(_TASKS) > _TASK_KEEP:
        done = [t for t in _TASKS.values() if t["state"] != "running"]
        if not done:
            break
        _TASKS.pop(min(done, key=lambda t: t["t0"])["id"], None)
    return tid


def _task_start(tid: str, fn) -> None:
    def _run():
        t = _TASKS[tid]
        try:
            with contextlib.redirect_stdout(t["log"]):
                result = fn()
            t["result"] = _json_safe(result)
            t["state"] = "done"
        except Exception as exc:  # noqa: BLE001 —— 任何异常都要变成任务错误，不能只留在线程里
            tb = traceback.format_exc()
            t["error"] = f"{type(exc).__name__}: {exc}"
            t["state"] = "error"
            _log_error(f"[{t['label']}] {tb}")
        finally:
            t["t1"] = time.time()

    threading.Thread(target=_run, daemon=True).start()


def _task_log(t: dict, limit: int = 80) -> list:
    """取任务日志尾部；过滤掉 OCC 写 STEP 时刷屏的统计信息。"""
    lines = t["log"].getvalue().splitlines()
    keep = [ln.rstrip() for ln in lines
            if ln.strip() and not ln.lstrip().startswith(_NOISE)]
    return keep[-limit:]


@app.route("/api/task_status")
def api_task_status():
    t = _TASKS.get(request.args.get("id", ""))
    if t is None:
        return jsonify({"error": "任务不存在（可能服务重启过）"}), 404
    return jsonify({
        "id": t["id"], "label": t["label"], "state": t["state"],
        "elapsed_s": round((t["t1"] or time.time()) - t["t0"], 1),
        "log": _task_log(t), "result": t["result"], "error": t["error"],
    })


# --------------------------------------------------------------------------- 页面
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/upload", methods=["POST"])
def api_upload():
    """上传产品数模。返回管件编号（默认输出前缀）与存储文件名。"""
    f = request.files.get("file")
    if f is None or not f.filename:
        return jsonify({"error": "没有收到文件"}), 400
    name = _safe_name(f.filename)
    ext = os.path.splitext(name)[1].lower()
    if ext not in ALLOWED_EXTS:
        return jsonify({"error": f"不支持的格式 {ext or '(无扩展名)'}，"
                                 f"请上传 {' / '.join(ALLOWED_EXTS)}"}), 400
    # 存成 "<12位hex>_原名"：避免重名覆盖，同时 naming_utils 会剥掉这个前缀
    save_name = f"{uuid.uuid4().hex[:12]}_{name}"
    save_path = os.path.join(UPLOAD_DIR, save_name)
    f.save(save_path)
    if os.path.getsize(save_path) == 0:
        os.remove(save_path)
        return jsonify({"error": "上传的文件是空的"}), 400
    return jsonify({
        "file": save_name,
        "original_name": name,
        "part_number": part_number_from_filename(name),
        "size_mb": _human_size(save_path),
        "ext": ext,
        "stl_note": ("STL 是多面体近似：型腔会是平面小面。要光滑曲面请用 STEP。"
                     if ext == ".stl" else ""),
    })


# --------------------------------------------------------------------------- 管件模具
def _mold_files(tid: str, paths: dict) -> list:
    """把导出路径整理成前端要的"下载卡片"列表。"""
    spec = (("upper", "上模", "型腔内壁 = 管件外壁"),
            ("lower", "下模", "与上模在同一水平分模面处切开"),
            ("core_pin", "芯棒", "内孔全长、端面与管件端口齐平"),
            ("mold4_assembly", "四件套装配体", "上模 + 产品 + 芯棒 + 下模（合模状态）"),
            ("metadata", "元数据 JSON", "分模高度 / 体积 / 耗时 / 警告"))
    out = []
    for key, label, note in spec:
        p = paths.get(key)
        if p and os.path.isfile(p):
            fname = os.path.basename(p)
            out.append({"key": key, "label": label, "note": note, "name": fname,
                        "size_mb": _human_size(p),
                        "url": f"/api/download?task={tid}&file={fname}"})
    return out


@app.route("/api/mold_generate", methods=["POST"])
def api_mold_generate():
    """生成管件模具（异步任务）。

    请求 JSON:
        file            上传后的存储文件名
        prefix          输出前缀（默认取管件编号）
        parting_surface "plane"（水平面，默认）/ "silhouette"（侧影随形）
        parting_z       水平分模面高度 mm（不给 = 自动推荐）
        margin          [mx,my,mz] 模芯余量
        verify          true = 做开模/顶出干涉校验（慢几分钟；默认 false）
        with_core       true(默认) = 出芯棒与四件套
    """
    d = request.get_json(silent=True) or {}
    abs_src = _uploaded_path(d.get("file"))
    if not abs_src:
        return jsonify({"error": "请先上传产品数模"}), 400

    prefix = _safe_name(d.get("prefix") or "", "")
    if not prefix:
        prefix = part_number_from_filename(abs_src)

    parting = str(d.get("parting_surface") or "plane").lower()
    if parting not in ("plane", "silhouette"):
        parting = "plane"

    pz = d.get("parting_z")
    try:
        pz = None if pz in (None, "", "auto") else float(pz)
    except (TypeError, ValueError):
        return jsonify({"error": "分模面高度必须是数字（或留空用自动推荐值）"}), 400

    margin = d.get("margin") or (20.0, 20.0, 20.0)
    try:
        margin = tuple(float(v) for v in list(margin)[:3])
        if len(margin) != 3:
            raise ValueError
    except (TypeError, ValueError):
        return jsonify({"error": "模芯余量要 3 个数字"}), 400

    verify = bool(d.get("verify", False))
    with_core = bool(d.get("with_core", True))
    out_dir = os.path.join(OUT_ROOT, time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6])

    tid = _task_new(f"生成管件模具：{prefix}")
    os.makedirs(out_dir, exist_ok=True)

    def _work():
        from main import generate_mold

        t0 = time.time()
        res = generate_mold(
            part_path=abs_src, margin=margin, output_dir=out_dir, prefix=prefix,
            parting_surface=parting, parting_z=pz, verify=verify, with_core=with_core,
        )
        pipe = res["pipe"]
        product = res["product"]
        meta = {}
        mpath = res["paths"].get("metadata")
        if mpath and os.path.isfile(mpath):
            try:
                with open(mpath, encoding="utf-8") as fh:
                    meta = json.load(fh)
            except (OSError, ValueError):
                meta = {}
        return {
            "prefix": prefix,
            "out_dir": out_dir,          # /api/download 靠它定位文件（只允许本任务的目录）
            "elapsed_s": round(time.time() - t0, 1),
            "files": _mold_files(tid, res["paths"]),
            "summary": {
                "product_size_mm": [round(v, 2) for v in
                                    (product.bbox[3] - product.bbox[0],
                                     product.bbox[4] - product.bbox[1],
                                     product.bbox[5] - product.bbox[2])],
                "product_volume_mm3": round(float(product.volume), 1),
                "core_volume_mm3": round(float(pipe.core_volume), 1),
                "upper_volume_mm3": round(float(pipe.upper_volume), 1),
                "lower_volume_mm3": round(float(pipe.lower_volume), 1),
                "parting_kind": pipe.parting_kind,
                "parting_z_mm": round(float(pipe.parting_z_mean), 2),
                "trim_via": (meta.get("pipe_stats", {}) or {}).get("envelope", {}).get("via"),
            },
            "warnings": list(pipe.warnings or []),
            "metadata": meta,
        }

    _task_start(tid, _work)
    return jsonify({"task_id": tid, "async": True, "state": "running", "out_dir": out_dir})


@app.route("/api/pipe_plane_suggest", methods=["POST"])
def api_pipe_plane_suggest():
    """只算"推荐水平分模面高度 + 安全区间"（不跑布尔，异步任务，几十秒）。"""
    d = request.get_json(silent=True) or {}
    abs_src = _uploaded_path(d.get("file"))
    if not abs_src:
        return jsonify({"error": "请先上传产品数模"}), 400
    margin = d.get("margin") or (20.0, 20.0, 20.0)
    try:
        margin = tuple(float(v) for v in list(margin)[:3])
    except (TypeError, ValueError):
        margin = (20.0, 20.0, 20.0)

    tid = _task_new("推荐分模面高度")

    def _work():
        from main import suggest_pipe_plane

        t0 = time.time()
        info = suggest_pipe_plane(part_path=abs_src, margin=margin)
        return {"suggest": info, "elapsed_s": round(time.time() - t0, 1)}

    _task_start(tid, _work)
    return jsonify({"task_id": tid, "async": True, "state": "running"})


@app.route("/api/download")
def api_download():
    """下载某个任务的输出文件（只允许该任务自己的输出目录）。"""
    t = _TASKS.get(request.args.get("task", ""))
    if t is None or not t.get("result"):
        return "任务不存在或还没完成", 404
    out_dir = str(t["result"].get("out_dir") or "")
    name = os.path.basename(request.args.get("file", ""))
    if not out_dir or not name:
        return "参数不对", 400
    path = os.path.join(out_dir, name)
    if not os.path.isfile(path):
        return "文件不存在", 404
    return send_file(path, as_attachment=True, download_name=name)


# --------------------------------------------------------------------------- 镜像 / 旋转
@app.route("/api/generate_mirror", methods=["POST"])
def api_generate_mirror():
    """镜像 / 绕轴旋转（可叠加：先镜像后旋转），导出一个 STEP 供下载。"""
    d = request.get_json(silent=True) or {}
    abs_src = _uploaded_path(d.get("file"))
    if not abs_src:
        return jsonify({"error": "请先上传产品数模"}), 400

    do_mirror = bool(d.get("mirror", True))
    do_rotate = bool(d.get("rotate", False))
    if not (do_mirror or do_rotate):
        return jsonify({"error": "镜像和旋转至少要选一个"}), 400

    plane = str(d.get("plane") or "xOy")
    if plane not in ("xOy", "xOz", "yOz"):
        plane = "xOy"
    axis = str(d.get("axis") or "z").lower()
    if axis not in ("x", "y", "z"):
        axis = "z"
    direction = "cw" if str(d.get("direction") or "cw").lower().startswith(("cw", "顺")) else "ccw"
    try:
        degrees = float(d.get("degrees", 90.0))
    except (TypeError, ValueError):
        return jsonify({"error": "旋转角度必须是数字"}), 400

    try:
        from mirror_generator import MirrorGenerator

        t0 = time.time()
        res = MirrorGenerator().generate(
            abs_src, plane=plane, output_dir=MIRROR_DIR,
            center_on_centroid=bool(d.get("center_on_centroid", True)),
            rotate=do_rotate, axis=axis, degrees=degrees, direction=direction,
            mirror=do_mirror,
        )
    except Exception as exc:  # noqa: BLE001
        _log_error(traceback.format_exc())
        return jsonify({"error": f"{type(exc).__name__}: {exc}"}), 500

    name = os.path.basename(res["output_path"])
    return jsonify({
        "ok": True, "name": name, "size_mb": _human_size(res["output_path"]),
        "url": f"/api/download_mirror?name={name}",
        "tag": res.get("tag"), "mirrored": res.get("mirrored"), "rotated": res.get("rotated"),
        "rotate_axis": res.get("rotate_axis"), "rotate_deg": res.get("rotate_deg"),
        "rotate_dir": res.get("rotate_dir"), "plane_label": res.get("plane_label"),
        "bbox_size": res.get("bbox", {}).get("size"),
        "elapsed_s": round(time.time() - t0, 1),
    })


@app.route("/api/download_mirror")
def api_download_mirror():
    name = os.path.basename(request.args.get("name", ""))
    path = os.path.join(MIRROR_DIR, name)
    if not name or not os.path.isfile(path):
        return "文件不存在", 404
    return send_file(path, as_attachment=True, download_name=name)


# --------------------------------------------------------------------------- 兜底
@app.errorhandler(Exception)
def _handle_uncaught(exc):
    from werkzeug.exceptions import HTTPException

    if isinstance(exc, HTTPException):
        return exc
    _log_error(traceback.format_exc())
    if request.path.startswith("/api/"):
        return jsonify({"error": f"{type(exc).__name__}: {exc}"}), 500
    return "服务器内部错误，详见 web_box_generator/error.log", 500


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="管件硬模成型 —— 极简 Web 服务")
    parser.add_argument("--port", type=int, default=5002, help="端口（默认 5002）")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="监听地址")
    args = parser.parse_args(argv)
    print("=" * 62)
    print("  管件硬模成型 —— 上传 / 生成（上模·下模·芯棒·四件套）/ 镜像旋转")
    print(f"  浏览器打开: http://{args.host}:{args.port}")
    print("=" * 62, flush=True)
    app.run(host=args.host, port=args.port, debug=False, threaded=True, use_reloader=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
