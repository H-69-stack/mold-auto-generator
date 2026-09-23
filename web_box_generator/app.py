"""
三维模型查看与测量系统 - Flask 后端
升级版：使用 OpenCASCADE 引擎解析 STEP，支持切割面分析、特征检测
"""

import os
import sys
import io
import uuid
import json
import tempfile
import math
import threading
import contextlib
import multiprocessing

# Ensure project root is in path
_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

# Windows 控制台默认 GBK 编码，而 mold_generator/mold_fix 等模块会打印
# ✓/→/⚠ 等 Unicode 符号 —— print 时抛 UnicodeEncodeError 会让 /api/generate_mold
# 等接口返回 500。强制 stdout/stderr 使用 UTF-8 + errors='replace'，
# 任何模块打印任何字符都不会再崩溃。
if sys.stdout and hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
if sys.stderr and hasattr(sys.stderr, 'reconfigure'):
    try:
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

from flask import Flask, render_template, request, send_file, jsonify, after_this_request

# Import BoxGenerator from project root
from BoxGenerator import StepBoxWriter, StlBoxWriter

# 项目根目录也加进 sys.path：naming_utils / model_reader 等公共模块都在根目录
_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)
from naming_utils import part_number_from_filename, strip_uid_prefix  # noqa: E402

app = Flask(__name__)
app.config['MAX_CONTENT_LENGTH'] = 200 * 1024 * 1024

UPLOAD_DIR = os.path.join(os.path.dirname(__file__), 'uploads')
os.makedirs(UPLOAD_DIR, exist_ok=True)

# 未捕获异常统一落盘（排查"前端只看到 Unexpected token '<' ... not valid JSON"这类问题时，
# 只要看这个文件就能拿到真正的 Python 堆栈，不用去翻启动服务的终端）
_ERROR_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'error.log')


def _log_exception(exc, where: str = '') -> str:
    import datetime
    import traceback
    tb = traceback.format_exc()
    text = (f"\n[{datetime.datetime.now():%Y-%m-%d %H:%M:%S}] {where} "
            f"{type(exc).__name__}: {exc}\n{tb}")
    try:
        with open(_ERROR_LOG, 'a', encoding='utf-8') as fh:
            fh.write(text)
    except Exception:
        pass
    try:
        sys.stderr.write(text)
    except Exception:
        pass
    return tb


def _json_safe(obj):
    """递归把"不能 JSON 序列化的东西"转成字符串（TopoDS_Shape / set / 自定义对象…）。

    踩过的坑：把某个 OCC 实体顺手塞进统计信息里，`jsonify` 直接抛
    `TypeError: Object of type TopoDS_Shape is not JSON serializable`，
    整个生成结果都拿不到（前端只看到"失败"）。所有接口返回前统一过一遍这里。
    """
    if isinstance(obj, dict):
        return {str(k): _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, (bool, int, float, str)) or obj is None:
        return obj
    return str(obj)


@app.errorhandler(Exception)
def _handle_uncaught(exc):
    """任何未捕获异常：写 error.log，并且**对接口返回 JSON**（而不是 HTML 错误页）。

    以前未捕获异常会让 Flask 返回 HTML 500，前端 `resp.json()` 直接报
    "Unexpected token '<' ... is not valid JSON"，把真正的错误信息掩盖掉了。
    """
    from werkzeug.exceptions import HTTPException
    if isinstance(exc, HTTPException) and (exc.code or 500) < 500:
        return exc
    tb = _log_exception(exc, request.path)
    code = getattr(exc, 'code', 500) or 500
    if request.path.startswith('/api') or request.path in ('/upload', '/generate'):
        return jsonify({'error': f'{type(exc).__name__}: {exc}',
                        'path': request.path,
                        'hint': '详细堆栈见 web_box_generator/error.log'}), code
    import html as _html
    return (f'<h1>500 Internal Server Error</h1><p>{type(exc).__name__}: '
            f'{_html.escape(str(exc))}</p><pre>{_html.escape(tb)}</pre>'), 500

# 缓存
_raw_stp_cache = {}

# ============ 镜像模型 API ============

_MIRROR_RESULTS = {}  # task_id -> result

@app.route('/api/generate_mirror', methods=['POST'])
def api_generate_mirror():
    """生成"镜像 / 绕轴旋转 / 镜像+旋转"后的模型（2026-09：镜像功能扩展为镜像+旋转）

    请求体:
        filepath:            上传文件的绝对路径
        mirror:              是否镜像（默认 true，兼容旧前端）
        plane:               镜像面 'xOy' / 'xOz' / 'yOz'
        center_on_centroid:  镜像面是否过质心
        rotate:              是否旋转（默认 false）
        axis:                旋转轴 'x' / 'y' / 'z'
        direction:           'ccw' 逆时针（默认）/ 'cw' 顺时针
        degrees:             旋转角度（度，正数，自填）
        rotate_about_centroid: 旋转轴是否过质心（默认 true）
    """
    data = request.get_json(force=True)
    filepath = data.get('filepath', '')
    plane = data.get('plane', 'xOy')
    center_on_centroid = data.get('center_on_centroid', True)
    mirror = bool(data.get('mirror', True))
    rotate = bool(data.get('rotate', False))
    axis = str(data.get('axis', 'z')).lower()
    direction = str(data.get('direction', 'ccw')).lower()
    rotate_about_centroid = bool(data.get('rotate_about_centroid', True))
    try:
        degrees = float(data.get('degrees', 90.0))
    except (TypeError, ValueError):
        return jsonify({'error': '旋转角度必须是数字'}), 400

    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    allowed_dirs = [os.path.abspath(UPLOAD_DIR)]
    if not any(abs_path.startswith(d) for d in allowed_dirs):
        return jsonify({'error': 'Access denied'}), 403

    if mirror and plane not in ('xOy', 'xOz', 'yOz'):
        return jsonify({'error': 'Invalid plane, must be xOy, xOz or yOz'}), 400
    if not mirror and not rotate:
        return jsonify({'error': '镜像和旋转至少要选一个'}), 400
    if rotate:
        if axis not in ('x', 'y', 'z'):
            return jsonify({'error': '旋转轴必须是 x / y / z'}), 400
        if not (0.0 < abs(degrees) <= 360.0):
            return jsonify({'error': '旋转角度需在 0~360 度之间'}), 400
        if direction not in ('cw', 'ccw'):
            return jsonify({'error': '旋转方向必须是 cw（顺时针）或 ccw（逆时针）'}), 400

    try:
        from mirror_generator import MirrorGenerator
        gen = MirrorGenerator()
        output_dir = os.path.join(UPLOAD_DIR, 'mirror_output')
        os.makedirs(output_dir, exist_ok=True)

        # 输出名用**管件编号**（不是整串文件名），避免名字越滚越长
        clean_name = part_number_from_filename(_strip_uid_prefix(abs_path))

        result = gen.generate(
            filepath=abs_path,
            plane=plane,
            center_on_centroid=bool(center_on_centroid),
            output_dir=output_dir,
            product_name=clean_name,
            mirror=mirror,
            rotate=rotate,
            axis=axis,
            degrees=degrees,
            direction=direction,
            rotate_about_centroid=rotate_about_centroid,
        )

        task_id = uuid.uuid4().hex[:12]
        _MIRROR_RESULTS[task_id] = result

        return jsonify({
            'task_id': task_id,
            'output_path': result['output_path'],
            'output_name': os.path.basename(result['output_path']),
            'plane': result['plane'],
            'plane_label': result['plane_label'],
            'centroid': result['centroid'],
            'bbox': result['bbox'],
            'mirrored': result['mirrored'],
            'rotated': result['rotated'],
            'rotate_axis': result['rotate_axis'],
            'rotate_deg': result['rotate_deg'],
            'rotate_dir': result['rotate_dir'],
        })

    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/download_mirror')
def api_download_mirror():
    """下载生成的镜像模型文件"""
    filepath = _normalize_path(request.args.get('file', ''))
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    # 验证文件来自镜像输出目录
    allowed_dirs = [os.path.abspath(os.path.join(UPLOAD_DIR, 'mirror_output'))]
    if not any(abs_path.startswith(d) for d in allowed_dirs):
        return jsonify({'error': 'Access denied'}), 403

    return send_file(
        abs_path,
        mimetype='application/step',
        as_attachment=True,
        download_name=_strip_uid_prefix(abs_path),
    )


# ============ 模具修复 API (补齐/挖空) ============

_FIX_RESULTS = {}  # task_id -> result

@app.route('/api/fix_mold', methods=['POST'])
def api_fix_mold():
    """模具修复：补齐(填实空腔) 或 挖空(产品贯穿孔)"""
    data = request.get_json(force=True)
    filepath = data.get('filepath', '')
    operation = data.get('operation', 'fill')  # 'fill' | 'hollow'
    product_filepath = data.get('product_filepath', None)

    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    allowed_dirs = [os.path.abspath(UPLOAD_DIR)]
    if not any(abs_path.startswith(d) for d in allowed_dirs):
        return jsonify({'error': 'Access denied'}), 403

    if operation not in ('fill', 'hollow'):
        return jsonify({'error': 'Invalid operation, must be fill or hollow'}), 400

    try:
        from mold_fix import MoldFixer
        fixer = MoldFixer()
        output_dir = os.path.join(UPLOAD_DIR, 'fix_output')
        os.makedirs(output_dir, exist_ok=True)

        # 使用剥离 uid 前缀后的干净文件名作为输出名，避免 hex 前缀
        clean_name = os.path.splitext(_strip_uid_prefix(abs_path))[0]

        if operation == 'fill':
            result = fixer.fill(
                filepath=abs_path,
                product_filepath=product_filepath,
                output_dir=output_dir,
                output_name=clean_name,
            )
        else:
            result = fixer.hollow(
                filepath=abs_path,
                product_filepath=product_filepath,
                output_dir=output_dir,
                output_name=clean_name,
            )

        task_id = uuid.uuid4().hex[:12]
        _FIX_RESULTS[task_id] = result

        return jsonify({
            'task_id': task_id,
            'output_path': result['output_path'],
            'operation': result['operation'],
            'method': result.get('method'),
            'bbox': result['bbox'],
        })

    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/download_fix')
def api_download_fix():
    """下载修复后的模具文件"""
    filepath = _normalize_path(request.args.get('file', ''))
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    # 验证文件来自修复输出目录
    allowed_dirs = [os.path.abspath(os.path.join(UPLOAD_DIR, 'fix_output'))]
    if not any(abs_path.startswith(d) for d in allowed_dirs):
        return jsonify({'error': 'Access denied'}), 403

    return send_file(
        abs_path,
        mimetype='application/step',
        as_attachment=True,
        download_name=_strip_uid_prefix(abs_path),
    )


# ============ 页面路由 ============

@app.route('/')
def index():
    return render_template('index.html')

# ============ 生成立方体 ============

def _params(data):
    L = float(data.get('length', 100))
    W = float(data.get('width', 50))
    H = float(data.get('height', 30))
    fmt = data.get('format', 'stp')
    fname = data.get('filename', f'Box_{L:.0f}x{W:.0f}x{H:.0f}')
    return L, W, H, fmt, fname


@app.route('/generate', methods=['POST'])
def generate():
    try:
        data = request.get_json(force=True)
        L, W, H, fmt, fname = _params(data)
        if fmt == 'stl':
            writer = StlBoxWriter(L, W, H)
            with tempfile.NamedTemporaryFile(suffix='.stl', delete=False) as tmp:
                tmp_path = tmp.name
            writer.write(tmp_path)
            buf = io.BytesIO()
            with open(tmp_path, 'rb') as f:
                buf.write(f.read())
            os.unlink(tmp_path)
            buf.seek(0)
            return send_file(buf, mimetype='application/sla', as_attachment=True, download_name=f'{fname}.stl')
        else:
            writer = StepBoxWriter(L, W, H)
            with tempfile.NamedTemporaryFile(suffix='.stp', delete=False) as tmp:
                tmp_path = tmp.name
            writer.write(tmp_path)
            buf = io.BytesIO()
            with open(tmp_path, 'rb') as f:
                buf.write(f.read())
            os.unlink(tmp_path)
            buf.seek(0)
            return send_file(buf, mimetype='application/step', as_attachment=True, download_name=f'{fname}.stp')
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# ============ 上传 ============

def _normalize_path(path):
    path = path.replace('\\', '/')
    return path


def _strip_uid_prefix(filename):
    """从文件名中移除用于唯一存储的 {12位hex}_ 前缀，返回干净的文件名。

    上传时文件以 f'{uid}_{原文件名}' 存储，uid 是 uuid4().hex[:12]，
    这里循环剥离所有连续的前缀（防止多重上传累积多个 hex 前缀），
    避免 hex 前缀传播到镜像/模具等生成文件名中。

    Args:
        filename: 文件路径或文件名

    Returns:
        去掉所有 {12位hex}_ 前缀后的基本文件名；若无前缀则原样返回
    """
    base = os.path.basename(filename)
    while True:
        parts = base.split('_', 1)
        if len(parts) == 2 and len(parts[0]) == 12 and all(c in '0123456789abcdef' for c in parts[0].lower()):
            # UUID hex 前缀几乎总包含 a-f 字母；全数字的 12 位前缀可能是
            # 正常产品编号，不应剥离
            if any(c in 'abcdef' for c in parts[0].lower()):
                base = parts[1]
                continue
        break
    return base


@app.route('/upload', methods=['POST'])
def upload():
    if 'file' not in request.files:
        return jsonify({'error': 'No file uploaded'}), 400
    f = request.files['file']
    if f.filename == '':
        return jsonify({'error': 'No file selected'}), 400
    ext = os.path.splitext(f.filename)[1].lower()
    if ext not in ('.stp', '.step', '.stl'):
        return jsonify({'error': 'Unsupported format'}), 400
    uid = uuid.uuid4().hex[:12]
    save_name = f'{uid}_{f.filename}'
    save_path = os.path.join(UPLOAD_DIR, save_name)
    f.save(save_path)

    abs_path = os.path.abspath(save_path)

    if ext in ('.stp', '.step'):
        data = _parse_step(abs_path)
    else:
        data = _parse_stl(abs_path)

    # 剥离存储用的 uid 前缀，避免 hex 前缀传播到界面和生成文件名
    clean_name = _strip_uid_prefix(f.filename)
    data['filename'] = clean_name
    data['file_name'] = clean_name
    data['filepath'] = abs_path
    # 管件编号（输出前缀默认用它）：24TK_1324-31_ROTATE_x_CW_90deg → 24TK_1324-31
    data['part_number'] = part_number_from_filename(clean_name)
    return jsonify(data)


def _clean_mesh(vertices, triangles, merge_threshold=1e-6):
    """
    清理网格数据：去除重复顶点、移除退化三角形
    
    Args:
        vertices: 顶点列表 [v0x, v0y, v0z, v1x, v1y, v1z, ...]
        triangles: 三角形索引列表 [i0, i1, i2, i3, i4, i5, ...]
        merge_threshold: 顶点合并距离阈值
    
    Returns:
        (new_vertices, new_triangles)
    """
    if len(vertices) < 9:
        return vertices, triangles
    
    # 1. 合并重复顶点
    vert_list = [(vertices[i], vertices[i+1], vertices[i+2]) for i in range(0, len(vertices), 3)]
    
    # 用哈希表快速去重
    vert_map = {}
    new_verts = []
    index_map = []
    
    for v in vert_list:
        # 超高精度容差 0.0000001mm（1e-7）：只合并坐标完全一致的顶点
        # OCCT 相邻面共享边上的节点坐标完全一致，合并后曲面无缝；
        # 不同面的边界顶点哪怕差 0.00001mm 也不合并，避免缝合处法线平均鼓包
        key = (round(v[0] * 1e7) / 1e7,
               round(v[1] * 1e7) / 1e7,
               round(v[2] * 1e7) / 1e7)
        existing = vert_map.get(key)
        if existing is not None:
            index_map.append(existing)
        else:
            idx = len(new_verts)
            vert_map[key] = idx
            new_verts.append(v)
            index_map.append(idx)
    
    # 2. 重建三角形并移除退化三角形
    new_tris = []
    for i in range(0, len(triangles), 3):
        i0 = index_map[triangles[i]]
        i1 = index_map[triangles[i+1]]
        i2 = index_map[triangles[i+2]]
        
        # 跳过退化三角形（两个或三个顶点相同）
        if i0 == i1 or i1 == i2 or i0 == i2:
            continue
        
        new_tris.extend([i0, i1, i2])
    
    # 展平顶点
    flat_verts = []
    for v in new_verts:
        flat_verts.extend(v)
    
    return flat_verts, new_tris


_PARSE_TIMEOUT = 60  # 秒；超过则强制终止解析子进程，防止页面无限加载


def _parse_error_data(filepath, message):
    """生成解析失败/超时的标准返回结构（前端识别 error 字段后停止转圈）"""
    return {
        'filename': os.path.basename(filepath),
        'filepath': os.path.abspath(filepath),
        'error': message,
        'solids': [],
        'bbox': {'min': [0, 0, 0], 'max': [0, 0, 0], 'center': [0, 0, 0], 'size': [0, 0, 0]},
        'centroid': [0, 0, 0],
    }


_pool = None


def _get_parse_pool():
    """懒加载单工作进程池。

    关键原因：OpenCASCADE 的网格化（BRepMesh）是原生 C++ 代码，阻塞期间
    Python 解释器拿不到控制权 —— 这就是"解析卡住时页面一直转圈、终端
    Ctrl+C 也无效"的根本原因。把解析放进独立常驻子进程后：
      - 主进程永不阻塞，Ctrl+C 可立即响应；
      - 解析超时可被主进程强制终止，页面不再无限加载。
    """
    global _pool
    if _pool is None:
        _pool = multiprocessing.get_context('spawn').Pool(1)
    return _pool


def _reset_parse_pool():
    """终止异常/超时的解析子进程池（下次调用自动重建）"""
    global _pool
    if _pool is not None:
        try:
            _pool.terminate()
            _pool.join()
        except Exception:
            pass
        _pool = None


def _parse_step_worker(filepath):
    """在子进程中执行自适应精度的 STEP 解析，返回 raw 数据字典。

    精度策略（相对原来固定 0.005mm 可快数百倍）：
      1. 先用粗精度(2.0mm)读取，估算模型最大尺寸 L；
      2. 自适应线性偏差 = L/2500，即最长边细分约 2500 段；
      3. 小零件(≈30mm)→0.01mm，大模具(≈300mm)→0.06mm，
         既保证切割面/轮廓精度，又不会生成上亿三角形拖死浏览器和服务器。
    """
    try:
        from step_reader import StepReader

        # 第一遍：粗精度读取，获取模型包围盒/尺寸
        try:
            coarse = StepReader(linear_deflection=2.0, angular_deflection=0.5).read(filepath)
        except Exception as e:
            return {'error': f'无法解析 STEP 文件: {e}'}

        bbox = coarse.get('bbox', {})
        size = bbox.get('size', [0, 0, 0])
        size_max = max(size) if size else 0
        if size_max <= 0:
            return coarse

        # 自适应精度：小零件细、大零件粗（大模具曲面交线区会自然加密，不能太细）
        # 网格密度与前端"共享法线平滑着色"配合即可实现视觉光滑——精度不必过高，
        # 更重要的是保证浏览器能流畅解析/渲染，避免卡死。
        if size_max <= 120:
            segments = 1200          # 小零件：细分密一点，细节清晰
        elif size_max <= 300:
            segments = 900           # 中型：兼顾细节与速度
        else:
            segments = 700           # 大模具：防三角形爆炸
        linear = max(0.08, min(size_max / float(segments), 0.6))
        angular = max(0.08, min(linear * 1.2, 0.2))

        # 第二遍：按选定精度完整解析；若失败则退回粗精度结果
        if linear >= 2.0:
            raw = coarse
        else:
            try:
                reader = StepReader(linear_deflection=linear, angular_deflection=angular)
                raw = reader.read(filepath)
            except Exception:
                raw = coarse

        # 三角形数量上限保护：20万面以内保证浏览器流畅处理；
        # 超限时最多 3 轮逐步降低精度重解析，直到达标或无法再降
        MAX_TRIANGLES = 200000
        tris = sum(s['triangle_count'] for s in raw['solids'])
        guard = 0
        while tris > MAX_TRIANGLES and guard < 3:
            guard += 1
            coarser = max(linear * 1.7, size_max / 700.0)
            coarser = min(coarser, 1.2)
            coarser_ang = min(coarser * 1.3, 0.25)
            try:
                reader2 = StepReader(linear_deflection=coarser, angular_deflection=coarser_ang)
                raw2 = reader2.read(filepath)
                tris2 = sum(s['triangle_count'] for s in raw2['solids'])
                if tris2 >= tris:
                    break  # 降精度无收益，停止
                linear, angular = coarser, coarser_ang
                raw, tris = raw2, tris2
            except Exception:
                break

        raw['_parsed_linear'] = linear
        raw['_parsed_angular'] = angular
        return raw

    except Exception as e:
        return {'error': f'解析进程异常: {e}'}


def _parse_step(filepath):
    """使用 OpenCASCADE 引擎解析 STEP 文件。

    在独立子进程中执行：
      1. 自适应网格精度，避免固定 0.005mm 超高精度生成上亿三角形；
      2. 超过 _PARSE_TIMEOUT 秒自动强制终止，页面不再无限加载；
      3. 解析卡在原生代码时，主进程仍能响应 Ctrl+C。

    ⚠ 子进程池**建不起来**时（例如服务是在受限/沙箱终端里启动的，系统不允许创建
      multiprocessing 需要的管道/进程），退化为**在主进程内解析**：
      虽然会阻塞，但至少上传不会直接 500（实测这种环境里 /upload 返回 500 HTML、
      前端报 "Unexpected token '<' ... is not valid JSON"）。
    """
    abs_path = os.path.abspath(filepath)
    try:
        pool = _get_parse_pool()
    except Exception as exc:  # noqa: BLE001 —— 受限环境：退化为主进程内解析
        print(f"[parse] 子进程池不可用（{type(exc).__name__}: {exc}），"
              f"改为在主进程内解析（建议改用普通命令行窗口启动服务）")
        pool = None

    if pool is None:
        payload = _parse_step_worker(abs_path)
        if isinstance(payload, dict) and payload.get('error'):
            print(f"[parse] 主进程内解析失败: {payload['error']}")
            return _parse_error_data(abs_path, payload['error'])
    else:
        try:
            result = pool.apply_async(_parse_step_worker, (abs_path,))
            payload = result.get(timeout=_PARSE_TIMEOUT)
        except Exception as e:
            name = type(e).__name__
            print(f"[parse] {name}: {e}")
            _reset_parse_pool()
            if 'Timeout' in name:
                return _parse_error_data(
                    abs_path,
                    f'解析超时（超过 {_PARSE_TIMEOUT} 秒），已自动终止。'
                    '该 STEP 模型过于复杂，建议：1) 在 CAD 中精简模型后重新上传；'
                    '2) 导出为 STL 格式上传。'
                )
            return _parse_error_data(abs_path, f'解析进程异常: {e}')

        if isinstance(payload, dict) and payload.get('error'):
            _reset_parse_pool()
            return _parse_error_data(abs_path, payload['error'])

    raw = payload
    _raw_stp_cache[abs_path] = raw
    
    # 收集所有顶点
    all_verts = []
    for solid in raw['solids']:
        verts = solid.get('vertices', [])
        all_verts.extend([verts[i:i+3] for i in range(0, len(verts), 3)])

    # 计算质心
    cx = sum(v[0] for v in all_verts) / len(all_verts) if all_verts else 0
    cy = sum(v[1] for v in all_verts) / len(all_verts) if all_verts else 0
    cz = sum(v[2] for v in all_verts) / len(all_verts) if all_verts else 0
    centroid = [cx, cy, cz]

    bbox = raw.get('bbox', {})

    data = {
        'filename': raw.get('filename', os.path.basename(filepath)),
        'file_name': raw.get('filename', os.path.basename(filepath)),
        'filepath': os.path.abspath(filepath),
        'centroid': centroid,
        'bbox': bbox,
        'stats': raw.get('stats', {}),
        'solids': [],
    }
    for solid in raw['solids']:
        verts = solid['vertices']
        tris = solid['triangles']
        
        # 清理网格数据：去重、去除退化三角形
        # 阈值使用 1e-6：仅合并真正重复的顶点，避免 1e-4 强制网格取整
        # 导致 CAD 布尔窄边被塌陷成畸形三角形（突起/锯齿）
        clean_verts, clean_tris = _clean_mesh(verts, tris, merge_threshold=1e-6)
        
        # 如果去重后顶点太少，说明原网格有问题，保留原始数据
        if len(clean_verts) < 9 and len(verts) >= 9:
            clean_verts, clean_tris = verts, tris
        
        data['solids'].append({
            'name': solid.get('name', 'Shape'),
            'vertices': clean_verts,
            'triangles': clean_tris,
            'edges': [{'vertices': e['vertices'], 'curve_type': e.get('curve_type', 'Line')}
                      for e in solid.get('edges', [])],
        })
    
    total_verts = sum(len(s['vertices']) // 3 for s in data['solids'])
    total_tris = sum(len(s['triangles']) // 3 for s in data['solids'])
    data['vertex_count'] = total_verts
    data['face_count'] = total_tris
    
    return data


def _compute_raw_centroid(raw):
    """Compute centroid from raw (uncentered) vertex data"""
    all_verts = []
    for solid in raw.get('solids', []):
        verts = solid.get('vertices', [])
        for i in range(0, len(verts), 3):
            all_verts.append([verts[i], verts[i+1], verts[i+2]])
    if not all_verts:
        return [0, 0, 0]
    cx = sum(v[0] for v in all_verts) / len(all_verts)
    cy = sum(v[1] for v in all_verts) / len(all_verts)
    cz = sum(v[2] for v in all_verts) / len(all_verts)
    return [cx, cy, cz]


def _parse_stl(filepath):
    """Parse binary or ASCII STL file"""
    # Try to detect and parse
    with open(filepath, 'rb') as f:
        header = f.read(80)
        f.seek(0)
        content = f.read()

    # Check if it's binary STL (look for "solid" at start in ASCII)
    is_ascii = content[:5].lower() == b'solid' and b'\n' in content[:80]

    if is_ascii:
        return _parse_ascii_stl(filepath)
    else:
        return _parse_binary_stl(filepath)


def _parse_binary_stl(filepath):
    import struct
    with open(filepath, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]

    vert_map = {}
    unique_verts = []
    faces = []

    with open(filepath, 'rb') as f:
        f.read(80)  # skip header
        num_triangles = struct.unpack('<I', f.read(4))[0]
        for _ in range(num_triangles):
            f.read(12)  # skip normal
            face_verts = []
            for _ in range(3):
                x, y, z = struct.unpack('<fff', f.read(12))
                key = (round(x, 5), round(y, 5), round(z, 5))
                if key not in vert_map:
                    vert_map[key] = len(unique_verts)
                    unique_verts.append([x, y, z])
                face_verts.append(vert_map[key])
            faces.append(face_verts)
            f.read(2)  # attribute byte count

    edges = _build_edges(faces)
    unique_verts = _center_verts(unique_verts)
    bbox = _calc_bbox(unique_verts)
    return {
        'solids': [{'vertices': [v for pt in unique_verts for v in pt],
                     'triangles': [idx for f in faces for idx in f],
                     'edges': []}],
        'bbox': bbox,
        'stats': {'solids': 1, 'faces': len(faces), 'edges': len(edges), 'triangles': len(faces),
                   'volume': 0, 'surface_area': 0},
        'vertex_count': len(unique_verts),
        'face_count': len(faces),
        'filename': os.path.basename(filepath),
    }


def _parse_ascii_stl(filepath):
    import re
    with open(filepath, 'r') as f:
        content = f.read()
    vert_map = {}
    unique_verts = []
    faces = []
    pattern = r'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)'
    matches = list(re.finditer(pattern, content, re.IGNORECASE))
    for i in range(0, len(matches), 3):
        if i + 2 >= len(matches):
            break
        face_verts = []
        for j in range(3):
            x, y, z = float(matches[i+j].group(1)), float(matches[i+j].group(2)), float(matches[i+j].group(3))
            key = (round(x, 5), round(y, 5), round(z, 5))
            if key not in vert_map:
                vert_map[key] = len(unique_verts)
                unique_verts.append([x, y, z])
            face_verts.append(vert_map[key])
        faces.append(face_verts)
    edges = _build_edges(faces)
    unique_verts = _center_verts(unique_verts)
    bbox = _calc_bbox(unique_verts)
    return {
        'solids': [{'vertices': [v for pt in unique_verts for v in pt],
                     'triangles': [idx for f in faces for idx in f],
                     'edges': []}],
        'bbox': bbox,
        'stats': {'solids': 1, 'faces': len(faces), 'edges': len(edges), 'triangles': len(faces),
                   'volume': 0, 'surface_area': 0},
        'vertex_count': len(unique_verts),
        'face_count': len(faces),
        'filename': os.path.basename(filepath),
    }


def _build_edges(faces):
    edge_set = set()
    edges_out = []
    for f in faces:
        for i in range(3):
            a, b = f[i], f[(i + 1) % 3]
            key = (a, b) if a < b else (b, a)
            if key not in edge_set:
                edge_set.add(key)
                edges_out.append([key[0], key[1]])
    return edges_out


def _center_verts(verts):
    if not verts:
        return verts
    cx = sum(v[0] for v in verts) / len(verts)
    cy = sum(v[1] for v in verts) / len(verts)
    cz = sum(v[2] for v in verts) / len(verts)
    for v in verts:
        v[0] -= cx
        v[1] -= cy
        v[2] -= cz
    return verts


def _calc_bbox(verts):
    if not verts:
        return {'min': [0, 0, 0], 'max': [0, 0, 0], 'center': [0, 0, 0], 'size': [0, 0, 0]}
    arr = list(zip(*verts))
    bmin = [min(arr[0]), min(arr[1]), min(arr[2])]
    bmax = [max(arr[0]), max(arr[1]), max(arr[2])]
    return {
        'min': [round(v, 4) for v in bmin],
        'max': [round(v, 4) for v in bmax],
        'center': [round((bmin[i] + bmax[i]) / 2, 4) for i in range(3)],
        'size': [round(bmax[i] - bmin[i], 4) for i in range(3)],
    }


# ============ 切割面分析 API ============


@app.route('/api/slice_plane')
def api_slice_plane():
    """计算平面与模型相交轮廓"""
    filepath = _normalize_path(request.args.get('file', ''))
    origin_str = request.args.get('origin', '')
    normal_str = request.args.get('normal', '')

    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    allowed_dirs = [
        os.path.abspath(UPLOAD_DIR),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '三维数模')),
    ]
    if not any(abs_path.startswith(d) for d in allowed_dirs):
        return jsonify({'error': 'Access denied'}), 403

    try:
        origin = tuple(float(v.strip()) for v in origin_str.split(','))
        normal = tuple(float(v.strip()) for v in normal_str.split(','))
    except (ValueError, TypeError):
        return jsonify({'error': 'Invalid origin or normal format'}), 400

    if len(origin) != 3 or len(normal) != 3:
        return jsonify({'error': 'origin and normal must have exactly 3 components'}), 400

    # 缓存加载
    if abs_path in _raw_stp_cache:
        raw = _raw_stp_cache[abs_path]
    else:
        from step_reader import StepReader
        reader = StepReader(linear_deflection=0.5, angular_deflection=0.3)
        raw = reader.read(abs_path)
        _raw_stp_cache[abs_path] = raw

    # 计算原始坐标系的质心
    raw_centroid = _compute_raw_centroid(raw)

    # 前端传过来的 origin 是在居中的坐标系（已减去质心）中的坐标，
    # 需要转换回原始坐标系才能与 raw 数据匹配
    raw_origin = (
        origin[0] + raw_centroid[0],
        origin[1] + raw_centroid[1],
        origin[2] + raw_centroid[2],
    )

    from slice_analyzer import MeshSlicer
    slicer = MeshSlicer(direction=normal, origin=raw_origin)
    slicer.add_solids_from_raw(raw)
    result = slicer.intersect_plane(plane_origin=raw_origin, plane_normal=normal)

    # 补充包围盒信息（保持在原始坐标系中）
    bbox = raw.get('bbox', {})
    result['bbox'] = {
        'min': bbox.get('min', [0, 0, 0]),
        'max': bbox.get('max', [0, 0, 0]),
        'size': bbox.get('size', [0, 0, 0]),
        'center': [
            (bbox['min'][i] + bbox['max'][i]) / 2
            for i in range(3)
        ] if bbox.get('min') and bbox.get('max') else [0, 0, 0],
    }

    return jsonify(result)


# ============ 批量切片 API ============


@app.route('/api/slice_plane_batch')
def api_slice_plane_batch():
    """批量预计算切片"""
    filepath = _normalize_path(request.args.get('file', ''))
    normal_str = request.args.get('normal', '')
    range_min_str = request.args.get('range_min', '')
    range_max_str = request.args.get('range_max', '')
    count_str = request.args.get('count', '100')

    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    allowed_dirs = [
        os.path.abspath(UPLOAD_DIR),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '三维数模')),
    ]
    if not any(abs_path.startswith(d) for d in allowed_dirs):
        return jsonify({'error': 'Access denied'}), 403

    try:
        normal = tuple(float(v.strip()) for v in normal_str.split(','))
        range_min = float(range_min_str)
        range_max = float(range_max_str)
        count = int(count_str)
    except (ValueError, TypeError):
        return jsonify({'error': 'Invalid parameters'}), 400

    if len(normal) != 3 or count < 2 or count > 500:
        return jsonify({'error': 'Invalid parameters'}), 400

    # 缓存加载
    if abs_path in _raw_stp_cache:
        raw = _raw_stp_cache[abs_path]
    else:
        from step_reader import StepReader
        reader = StepReader(linear_deflection=0.5, angular_deflection=0.3)
        raw = reader.read(abs_path)
        _raw_stp_cache[abs_path] = raw

    # 计算原始坐标系质心
    raw_centroid = _compute_raw_centroid(raw)

    bbox = raw.get('bbox', {})
    bbox_min = bbox.get('min', [0, 0, 0])
    bbox_max = bbox.get('max', [0, 0, 0])
    center = [(bbox_min[i] + bbox_max[i]) / 2 for i in range(3)]

    from slice_analyzer import MeshSlicer
    slicer = MeshSlicer(direction=normal, origin=tuple(center))
    slicer.add_solids_from_raw(raw)

    slices = []
    step = (range_max - range_min) / (count - 1)
    for i in range(count):
        depth = range_min + i * step
        origin = list(center)
        # 沿着法线方向偏移
        max_axis = max(range(3), key=lambda k: abs(normal[k]))
        origin[max_axis] = depth
        origin = tuple(origin)
        try:
            res = slicer.intersect_plane(plane_origin=origin, plane_normal=normal)
            slices.append({
                'depth': round(depth, 3),
                'best_length': res.get('best_length', 0),
                'arc_lengths': res.get('arc_lengths', []),
                'contours_3d': res.get('contours_3d', []),
            })
        except Exception:
            slices.append({
                'depth': round(depth, 3),
                'best_length': 0,
                'arc_lengths': [],
                'contours_3d': [],
            })

    result = {
        'normal': list(normal),
        'range': [range_min, range_max],
        'count': count,
        'bbox': {
            'min': bbox_min,
            'max': bbox_max,
            'size': bbox.get('size', [0, 0, 0]),
            'center': center,
        },
        'slices': slices,
    }
    return jsonify(result)


# ============ 模具生成 API ============

_MOLD_RESULTS = {}  # task_id -> result

@app.route('/api/generate_mold', methods=['POST'])
def api_generate_mold():
    """从上传的 STEP 文件自动生成上下模具（凹模 + 凸模）"""
    data = request.get_json(force=True)
    filepath = data.get('filepath', '')
    try:
        side_margin = float(data.get('side_margin', 10))
        base_thickness = float(data.get('base_thickness', 10))
    except (TypeError, ValueError):
        return jsonify({'error': 'Invalid margin or thickness'}), 400

    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    allowed_dirs = [os.path.abspath(UPLOAD_DIR)]
    if not any(abs_path.startswith(d) for d in allowed_dirs):
        return jsonify({'error': 'Access denied'}), 403

    try:
        from mold_generator import MoldGenerator
        gen = MoldGenerator()
        output_dir = os.path.join(UPLOAD_DIR, 'mold_output')
        os.makedirs(output_dir, exist_ok=True)

        # 使用剥离 uid 前缀后的干净文件名作为输出名，避免 hex 前缀
        clean_name = os.path.splitext(_strip_uid_prefix(abs_path))[0]

        result = gen.generate(
            filepath=abs_path,
            side_margin=side_margin,
            base_thickness=base_thickness,
            output_dir=output_dir,
            product_name=clean_name,
        )

        task_id = uuid.uuid4().hex[:12]
        _MOLD_RESULTS[task_id] = result

        info = result.get('info', {})
        return jsonify({
            'task_id': task_id,
            'cavity': result.get('cavity'),
            'punch': result.get('punch'),
            'info': info,
        })

    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/download_mold')
def api_download_mold():
    """下载生成的模具文件"""
    filepath = _normalize_path(request.args.get('file', ''))
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    # 验证文件来自模具输出目录
    allowed_dirs = [os.path.abspath(os.path.join(UPLOAD_DIR, 'mold_output'))]
    if not any(abs_path.startswith(d) for d in allowed_dirs):
        return jsonify({'error': 'Access denied'}), 403

    return send_file(
        abs_path,
        mimetype='application/step',
        as_attachment=True,
        download_name=_strip_uid_prefix(abs_path),
    )


# ============ 智能模具系统 API（模块化新系统）============

_MOLD_SYS_RESULTS = {}  # task_id -> result
_MOLD_SYS_DIR = os.path.join(UPLOAD_DIR, 'mold_system_output')
os.makedirs(_MOLD_SYS_DIR, exist_ok=True)

# ---- 长任务（异步 + 轮询）------------------------------------------------
# 大件模具/芯棒生成要十几分钟，同步 HTTP 请求会被浏览器/代理掐断（页面显示"生成超时"，
# 但服务器其实还在算）。这里统一改成：POST 立刻返回 task_id，前端轮询 /api/task_status。
_TASKS = {}          # task_id -> {state, t0, log(StringIO), label, result, error}
_TASK_KEEP = 30      # 最多保留多少个任务记录


def _task_new(label: str) -> str:
    tid = uuid.uuid4().hex[:12]
    _TASKS[tid] = {"state": "running", "t0": __import__("time").time(),
                   "log": io.StringIO(), "label": label, "result": None, "error": None}
    if len(_TASKS) > _TASK_KEEP:
        for stale in list(_TASKS)[:-_TASK_KEEP]:
            _TASKS.pop(stale, None)
    return tid


def _task_start(tid: str, fn) -> None:
    th = threading.Thread(target=fn, name=f"task-{tid}", daemon=True)
    th.start()


@app.route('/api/task_status')
def api_task_status():
    """查询异步任务状态：running / done / error（done 时返回和同步版一样的结果字段）。"""
    import time
    tid = request.args.get('task_id', '')
    t = _TASKS.get(tid)
    if t is None:
        return jsonify({'error': 'unknown task_id', 'task_id': tid}), 404
    try:
        log = t["log"].getvalue()
    except Exception:  # noqa: BLE001
        log = ""
    out = {'task_id': tid, 'state': t["state"], 'label': t["label"],
           'elapsed_s': round(time.time() - t["t0"], 1), 'log_tail': log[-4000:]}
    if t["state"] == "done" and t["result"] is not None:
        out.update(t["result"])
    elif t["state"] == "error":
        out['error'] = t["error"]
        out['log'] = log
    return jsonify(out)


def _parse_float_list(raw, n, default):
    """把 JSON 数组解析为 n 个 float；缺失或非法时返回 default。"""
    try:
        if raw is None:
            return list(default)
        vals = [float(x) for x in raw]
        if len(vals) != n:
            return list(default)
        return vals
    except (TypeError, ValueError):
        return list(default)


@app.route('/api/mold_generate', methods=['POST'])
def api_mold_generate():
    """调用模块化模具自动生成系统（main.generate_mold）。

    请求 JSON:
      filepath:     上传的产品数模路径（必须位于 uploads 目录内）
      core_mode:    "auto"（产品包围盒+余量） | "custom"（指定 L W H）
      core_size:    [L, W, H]（custom 模式）
      margin:       [mx, my, mz]（auto 模式的双边余量）
      base_thickness: 底部基座厚度 mm（型腔底板实体，默认 20）
      parting_mode: "max_z" | "mid" | "custom"（mold_type=sheet 时生效）
      parting_z:    custom 模式的分模高度
      z_align:      "bottom" | "center"
      mold_type:    "sheet"（板件折弯/液压成型，默认） | "pipe"（管件硬模成型）
      prefix:       输出文件名前缀
    """
    data = request.get_json(force=True) if request.is_json else {}
    filepath = data.get('filepath', '')
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    if not abs_path.startswith(os.path.abspath(UPLOAD_DIR)):
        return jsonify({'error': 'Access denied'}), 403

    core_mode = data.get('core_mode', 'auto')
    if core_mode not in ('auto', 'custom'):
        return jsonify({'error': 'core_mode 必须为 auto 或 custom'}), 400
    core_size = None
    if core_mode == 'custom':
        core_size = _parse_float_list(data.get('core_size'), 3, [200.0, 160.0, 100.0])
        if any(v <= 0 for v in core_size):
            return jsonify({'error': '模芯尺寸必须为正数'}), 400
    margin = _parse_float_list(data.get('margin'), 3, [20.0, 20.0, 20.0])

    try:
        base_thickness = float(data.get('base_thickness', 20.0))
    except (TypeError, ValueError):
        return jsonify({'error': 'base_thickness 必须为数值'}), 400
    if base_thickness < 0:
        return jsonify({'error': 'base_thickness 不能为负数'}), 400

    parting_mode = data.get('parting_mode', 'contour')
    if parting_mode not in ('cavity_bottom', 'max_z', 'mid', 'custom', 'contour', 'silhouette'):
        return jsonify({'error': 'parting_mode 必须为 cavity_bottom / max_z / mid / custom / contour / silhouette'}), 400
    # 分模面高度：板件 custom 模式 与 管件水平分模面 都用它；不传/传 null = 自动
    z_parting = None
    raw_pz = data.get('parting_z')
    if raw_pz is not None and raw_pz != '':
        try:
            z_parting = float(raw_pz)
        except (TypeError, ValueError):
            return jsonify({'error': '分模面高度 parting_z 必须为数值'}), 400
    if parting_mode == 'custom' and z_parting is None:
        return jsonify({'error': 'custom 分模模式需要有效的 parting_z'}), 400

    z_align = data.get('z_align', 'bottom')
    if z_align not in ('bottom', 'center'):
        return jsonify({'error': 'z_align 必须为 bottom / center'}), 400

    # 模具工艺类型: sheet=板件折弯/液压成型, pipe=管件硬模成型
    mold_type = data.get('mold_type', 'sheet')
    if mold_type not in ('sheet', 'pipe'):
        return jsonify({'error': 'mold_type 必须为 sheet / pipe'}), 400

    # 管件模式分模面形式: plane=水平面（默认，最好加工）, silhouette=侧影随形
    parting_surface = data.get('parting_surface', 'plane')
    if parting_surface not in ('plane', 'silhouette'):
        return jsonify({'error': 'parting_surface 必须为 plane / silhouette'}), 400

    # 管件模式：是否做开模/顶出干涉检验（6 次布尔求交，很慢）；是否一起出四件套（含芯棒）
    verify = bool(data.get('verify', True))
    with_core = bool(data.get('with_core', True))
    run_parallel = bool(data.get('run_parallel', False))

    # 输出前缀：默认按**管件编号**自动取（如 24TK_1324-31_ROTATE_x_CW_90deg → 24TK_1324-31），
    # 用户在前端填了就用用户填的
    clean_name = _strip_uid_prefix(abs_path)
    _pfx_raw = str(data.get('prefix', '') or '').strip()
    if not _pfx_raw or _pfx_raw.lower() in ('mold', 'core', 'part'):
        _pfx_raw = part_number_from_filename(clean_name)
    prefix = ''.join(c for c in _pfx_raw
                     if c.isalnum() or c in '-_').strip() or 'mold'

    task_id = _task_new("管件硬模成型（管件）" if mold_type == 'pipe' else "板件折弯成型（板件）")
    output_dir = os.path.join(_MOLD_SYS_DIR, task_id)
    os.makedirs(output_dir, exist_ok=True)

    import time
    from main import generate_mold

    def _work():
        buf = _TASKS[task_id]["log"]
        t0 = time.time()
        try:
            with contextlib.redirect_stdout(buf):
                result = generate_mold(
                    part_path=abs_path,
                    core_size=tuple(core_size) if core_size else None,
                    margin=tuple(margin),
                    base_thickness=base_thickness,
                    parting_mode=parting_mode,
                    z_parting=z_parting,
                    z_align=z_align,
                    output_dir=output_dir,
                    prefix=prefix,
                    mold_type=mold_type,
                    parting_surface=parting_surface,
                    verify=verify,
                    with_core=with_core,
                    run_parallel=run_parallel,
                )
            payload = _mold_payload(task_id, result, mold_type, base_thickness,
                                    time.time() - t0, buf.getvalue())
            _TASKS[task_id].update({"state": "done",
                                    "result": _safe_payload(payload)})
        except Exception as exc:  # noqa: BLE001 —— 后台线程里统一转成任务错误
            _TASKS[task_id].update({"state": "error",
                                    "error": f"{type(exc).__name__}: {exc}"})

    _task_start(task_id, _work)
    # 立刻返回 task_id，前端轮询 /api/task_status（大件要跑十几分钟，同步请求会被浏览器掐断）
    return jsonify({'task_id': task_id, 'async': True, 'state': 'running'})


def _mold_payload(task_id, result, mold_type, base_thickness, elapsed, log):
    from geometry_utils import shape_volume

    product = result['product']
    core = result['core']
    parting = result['parting']
    # 板件: 型腔模芯 cavity_core；管件: 外形包络型腔 die_full（前端统一用 cavity 键）
    cavity_path = result['paths'].get('cavity_core') or result['paths'].get('die_full')
    # 上模 + 产品 + 下模 三件套装配体（两种模式都会导出）
    mold_assembly_path = result['paths'].get('mold_assembly')
    if mold_type == 'pipe':
        pipe = result['pipe']
        info = {
            'mold_type': 'pipe',
            'product_size_mm': [
                product.bbox[3] - product.bbox[0],
                product.bbox[4] - product.bbox[1],
                product.bbox[5] - product.bbox[2],
            ],
            'product_volume_mm3': product.volume,
            'core_size_mm': list(core.size),
            'core_volume_mm3': core.size[0] * core.size[1] * core.size[2],
            'cavity_core_volume_mm3': shape_volume(result['cavity_core']),
            'base_thickness_mm': None,
            'parting_mode': 'plane_pipe' if pipe.parting_kind == 'plane'
                            else 'silhouette_pipe',
            'parting_kind': pipe.parting_kind,
            'parting_axis': 'XY'[pipe.axis],
            'parting_z_mm': pipe.parting_z_mean,
            'parting_z_source': pipe.stats.get('plane', {}).get('source', 'auto'),
            'parting_z_auto_mm': pipe.stats.get('plane', {}).get('z_auto'),
            'parting_z_safe_range_mm': [pipe.stats.get('plane', {}).get('safe_lo'),
                                        pipe.stats.get('plane', {}).get('safe_hi')],
            'upper_mold_height_mm': pipe.stats.get('heights', {}).get('upper_height_mm'),
            'lower_mold_height_mm': pipe.stats.get('heights', {}).get('lower_height_mm'),
            'block_height_mm': pipe.stats.get('heights', {}).get('block_height_mm'),
            'core_pin_size_mm': list(pipe.stats.get('heights', {}).get('block_z') or []),
            'parting_z_range_mm': [min(h for _, h in pipe.profile),
                                   max(h for _, h in pipe.profile)],
            'die_volume_mm3': pipe.die_volume,
            'envelope_extra_volume_mm3': pipe.envelope_extra_volume,
            'upper_mold_volume_mm3': pipe.upper_volume,
            'lower_mold_volume_mm3': pipe.lower_volume,
            'demold_check': {k: round(v, 4) for k, v in pipe.demold.items()},
            'mold_warnings': list(pipe.warnings),
            'pipe_stats': pipe.stats,
            'core_pin_volume_mm3': pipe.core_volume,
            'has_core_pin': pipe.core_pin is not None,
            'assembly_parts': (['upper_mold', 'product', 'core_pin', 'lower_mold']
                               if pipe.core_pin is not None
                               else ['upper_mold', 'product', 'lower_mold']),
            'undercut_warnings': [],
            'elapsed_s': round(elapsed, 2),
        }
        extra_paths = {}
        for k in ('die_full', 'core_assembly', 'mold_assembly'):
            if result['paths'].get(k):
                extra_paths[k] = result['paths'][k]
        if result['paths'].get('mold4_assembly'):
            extra_paths['mold4_assembly'] = result['paths']['mold4_assembly']
            extra_paths['core_pin'] = result['paths']['core_pin']
    else:
        info = {
            'mold_type': 'sheet',
            'product_size_mm': [
                product.bbox[3] - product.bbox[0],
                product.bbox[4] - product.bbox[1],
                product.bbox[5] - product.bbox[2],
            ],
            'product_volume_mm3': product.volume,
            'core_size_mm': list(core.size),
            'core_volume_mm3': core.size[0] * core.size[1] * core.size[2],
            'cavity_core_volume_mm3': shape_volume(result['cavity_core']),
            'base_thickness_mm': base_thickness,
            'parting_z_mm': parting.parting_z,
            'upper_mold_volume_mm3': parting.upper_volume,
            'lower_mold_volume_mm3': parting.lower_volume,
            'undercut_warnings': [w.to_dict() for w in parting.undercut_warnings],
            'elapsed_s': round(elapsed, 2),
        }
        extra_paths = {}
    _MOLD_SYS_RESULTS[task_id] = {
        'assembly': result['paths'].get('core_assembly'),
        'cavity': cavity_path,
        'upper': result['paths']['upper'],
        'lower': result['paths']['lower'],
        'metadata': result['paths']['metadata'],
        'mold_assembly': mold_assembly_path,
        'info': info,
        **extra_paths,
    }
    # 仅保留最近 50 个任务记录，避免内存持续增长
    if len(_MOLD_SYS_RESULTS) > 50:
        for stale in list(_MOLD_SYS_RESULTS)[:-50]:
            _MOLD_SYS_RESULTS.pop(stale, None)
    return {
        'task_id': task_id,
        'assembly': result['paths'].get('core_assembly'),
        'cavity': cavity_path,
        'upper': result['paths']['upper'],
        'lower': result['paths']['lower'],
        'metadata': result['paths']['metadata'],
        'mold_assembly': mold_assembly_path,
        'mold4_assembly': result['paths'].get('mold4_assembly'),
        'core_pin': result['paths'].get('core_pin'),
        'info': info,
        'log': log,
        **extra_paths,
    }


def _safe_payload(payload):
    """接口统一出口：保证返回内容一定能被 JSON 序列化（OCC 实体一律转字符串）。"""
    safe = _json_safe(payload)
    # 只保留日志尾部，避免把几十 KB 日志塞进响应
    if isinstance(safe, dict) and isinstance(safe.get('log'), str):
        safe['log'] = safe['log'][-20000:]
    return safe


@app.route('/api/mold_download')
def api_mold_download():
    """下载智能模具系统生成的文件（上模/下模 STEP 或元数据 JSON）。"""
    filepath = _normalize_path(request.args.get('file', ''))
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    if not abs_path.startswith(os.path.abspath(_MOLD_SYS_DIR)):
        return jsonify({'error': 'Access denied'}), 403

    ext = os.path.splitext(abs_path)[1].lower()
    if ext in ('.step', '.stp'):
        mimetype = 'application/step'
    elif ext == '.json':
        mimetype = 'application/json'
    else:
        mimetype = 'application/octet-stream'
    return send_file(
        abs_path,
        mimetype=mimetype,
        as_attachment=True,
        download_name=_strip_uid_prefix(abs_path),
    )


# ============ 模芯设计（芯棒）API ============

_CORE_SYS_DIR = os.path.join(UPLOAD_DIR, 'core_output')
os.makedirs(_CORE_SYS_DIR, exist_ok=True)


@app.route('/api/core_generate', methods=['POST'])
def api_core_generate():
    """调用 core_design.design_core 生成芯棒（沿产品两端端口面把内孔填满）。

    请求 JSON:
      filepath:    上传的产品数模路径（必须位于 uploads 目录内）
      axis:        "auto"（默认，取水平长轴） | "x" | "y"
      clearance:   芯棒单边装配间隙 mm（0 = 与内孔完全贴合）
      plate_depth: 端口封盖拉伸长度 mm（默认 1.0）
      quick:       抽芯检验只做近距离（更快）
      prefix:      输出文件名前缀
    """
    data = request.get_json(force=True) if request.is_json else {}
    filepath = data.get('filepath', '')
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404
    abs_path = os.path.abspath(filepath)
    if not abs_path.startswith(os.path.abspath(UPLOAD_DIR)):
        return jsonify({'error': 'Access denied'}), 403

    axis_raw = str(data.get('axis', 'auto'))
    if axis_raw not in ('auto', 'x', 'y'):
        return jsonify({'error': 'axis 必须为 auto / x / y'}), 400
    try:
        clearance = float(data.get('clearance', 0.0))
        plate_depth = float(data.get('plate_depth', 1.0))
    except (TypeError, ValueError):
        return jsonify({'error': 'clearance / plate_depth 必须为数值'}), 400
    if clearance < 0:
        return jsonify({'error': 'clearance 不能为负'}), 400
    if plate_depth <= 0:
        return jsonify({'error': 'plate_depth 必须为正'}), 400
    quick = bool(data.get('quick', False))
    exact_overlap = bool(data.get('exact_overlap', False))
    contact_area = bool(data.get('contact_area', False))
    # 输出前缀：默认按**管件编号**自动取（用户填了就用用户填的）
    _pfx_raw = str(data.get('prefix', '') or '').strip()
    if not _pfx_raw or _pfx_raw.lower() in ('mold', 'core', 'part'):
        _pfx_raw = part_number_from_filename(_strip_uid_prefix(abs_path))
    prefix = ''.join(c for c in _pfx_raw
                     if c.isalnum() or c in '-_').strip() or 'core'

    task_id = _task_new("模芯设计（芯棒）")
    output_dir = os.path.join(_CORE_SYS_DIR, task_id)
    os.makedirs(output_dir, exist_ok=True)

    import time
    from core_design import design_core

    def _work():
        buf = _TASKS[task_id]["log"]
        t0 = time.time()
        try:
            with contextlib.redirect_stdout(buf):
                result = design_core(
                    part_path=abs_path, output_dir=output_dir, prefix=prefix,
                    axis=None if axis_raw == 'auto' else 'xy'.index(axis_raw),
                    clearance=clearance, plate_depth=plate_depth,
                    verify=True, quick=quick, exact_overlap=exact_overlap,
                    contact_area=contact_area,
                )
            _TASKS[task_id].update(
                {"state": "done",
                 "result": _safe_payload(_core_payload(result, time.time() - t0))})
        except Exception as exc:  # noqa: BLE001 —— 后台线程统一转成任务错误
            _TASKS[task_id].update({"state": "error",
                                    "error": f"{type(exc).__name__}: {exc}"})

    _task_start(task_id, _work)
    # 立刻返回 task_id，前端轮询 /api/task_status（芯棒生成在大件上也要十几分钟）
    return jsonify({'task_id': task_id, 'async': True, 'state': 'running'})


def _core_payload(result, elapsed):
    def _r(v):
        if v is None:
            return None
        try:
            f = float(v)
        except (TypeError, ValueError):
            return v
        return None if math.isnan(f) else round(f, 4)

    cd = result['core_design']
    info = {
        'core_volume_mm3': cd.volume,
        'core_length_mm': cd.length,
        'core_size_mm': list(cd.dims),
        'axis': 'XYZ'[cd.axis],
        'core_count': cd.core_count,
        'ports': [p.as_dict() for p in cd.ports],
        'sections': cd.sections,
        'fill_ratio': _r(cd.fill.get('fill_ratio')),
        'overlap_volume_mm3': _r(cd.fill.get('overlap_volume')),
        'section_ratio_min': _r(cd.fill.get('section_ratio_min')),
        'section_ratio_max': _r(cd.fill.get('section_ratio_max')),
        'section_stations_checked': _r(cd.fill.get('section_stations_checked')),
        'contact_area_mm2': _r(cd.fill.get('core_contact_area')),
        'lateral_area_mm2': _r(cd.fill.get('core_lateral_area')),
        'mass_balance': cd.stats.get('mass_balance', {}),
        'pull_check': {k: (_r(v) if isinstance(v, float) else v)
                       for k, v in cd.pull.items() if k != 'detail'},
        'pull_detail': cd.pull.get('detail', []),
        'removable': cd.removable,
        'pull_dir': cd.pull_dir,
        'core_warnings': list(cd.warnings),
        'elapsed_s': round(elapsed, 2),
    }
    return {
        'core': result['paths']['core_pin'],
        'assembly': result['paths']['core_assembly'],
        'metadata': result['paths']['metadata'],
        'info': info,
    }


@app.route('/api/pipe_plane_suggest', methods=['POST'])
def api_pipe_plane_suggest():
    """**快速推荐管件水平分模面高度**（不跑布尔，异步任务；几十秒）。

    请求 JSON: filepath（必须位于 uploads 内）、margin [mx,my,mz]（可选）。
    返回：suggest = {parting_z_auto, safe_lo, safe_hi, silhouette_min, up_facing_min,
                     column_low_max, product_z, block_z, block_height_mm,
                     upper_height_auto_mm, lower_height_auto_mm, safe_ok, ...}
    """
    data = request.get_json(force=True) if request.is_json else {}
    filepath = data.get('filepath', '')
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404
    abs_path = os.path.abspath(filepath)
    if not abs_path.startswith(os.path.abspath(UPLOAD_DIR)):
        return jsonify({'error': 'Access denied'}), 403
    margin = _parse_float_list(data.get('margin'), 3, [20.0, 20.0, 20.0])

    import time
    from main import suggest_pipe_plane

    task_id = _task_new("分模面高度推荐")

    def _work():
        buf = _TASKS[task_id]["log"]
        t0 = time.time()
        try:
            with contextlib.redirect_stdout(buf):
                info = suggest_pipe_plane(part_path=abs_path, margin=tuple(margin))
            _TASKS[task_id].update(
                {"state": "done",
                 "result": _safe_payload({"suggest": info,
                                          "elapsed_s": round(time.time() - t0, 2)})})
        except Exception as exc:  # noqa: BLE001
            _TASKS[task_id].update({"state": "error",
                                    "error": f"{type(exc).__name__}: {exc}"})

    _task_start(task_id, _work)
    return jsonify({'task_id': task_id, 'async': True, 'state': 'running'})


@app.route('/api/core_download')
def api_core_download():
    """下载模芯设计生成的文件（芯棒 / 产品+芯棒装配体 STEP 或元数据 JSON）。"""
    filepath = _normalize_path(request.args.get('file', ''))
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    if not abs_path.startswith(os.path.abspath(_CORE_SYS_DIR)):
        return jsonify({'error': 'Access denied'}), 403

    ext = os.path.splitext(abs_path)[1].lower()
    if ext in ('.step', '.stp'):
        mimetype = 'application/step'
    elif ext == '.json':
        mimetype = 'application/json'
    else:
        mimetype = 'application/octet-stream'
    return send_file(
        abs_path,
        mimetype=mimetype,
        as_attachment=True,
        download_name=_strip_uid_prefix(abs_path),
    )


@app.route('/api/model_parse')
def api_model_parse():
    """解析 STEP 文件为前端渲染数据（用于在 3D 查看器中预览生成的模具）。"""
    filepath = _normalize_path(request.args.get('file', ''))
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    if not abs_path.startswith(os.path.abspath(UPLOAD_DIR)):
        return jsonify({'error': 'Access denied'}), 403

    ext = os.path.splitext(abs_path)[1].lower()
    if ext not in ('.stp', '.step'):
        return jsonify({'error': '仅支持 STEP 文件预览'}), 400

    try:
        data = _parse_step(abs_path)
        if isinstance(data, dict):
            # 管件编号：前端用它自动填"输出文件名前缀"
            data['part_number'] = part_number_from_filename(_strip_uid_prefix(abs_path))
            data['file_name'] = _strip_uid_prefix(abs_path)
        return jsonify(data)
    except Exception as exc:
        return jsonify({'error': f'{type(exc).__name__}: {exc}'}), 500


@app.route('/api/model_section')
def api_model_section():
    """半剖 STEP 实体，返回可看到内部型腔剖面的网格数据。

    沿实体中心 X 平面切去左半，保留右半 —— 剖面直接暴露内部空腔形状，
    用于直观验证“模芯中间挖出了产品大小的型腔”。
    """
    filepath = _normalize_path(request.args.get('file', ''))
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    if not abs_path.startswith(os.path.abspath(UPLOAD_DIR)):
        return jsonify({'error': 'Access denied'}), 403

    ext = os.path.splitext(abs_path)[1].lower()
    if ext not in ('.stp', '.step'):
        return jsonify({'error': '仅支持 STEP 文件半剖预览'}), 400

    # 半剖方向：x / y / z，默认 x（沿中心 X 平面切去左半）
    axis = (request.args.get('axis', 'x') or 'x').lower()
    if axis not in ('x', 'y', 'z'):
        return jsonify({'error': 'axis 必须为 x / y / z'}), 400
    ai = {'x': 0, 'y': 1, 'z': 2}[axis]

    # 可选剖切位置比例 0~1（0.5 即正中），默认沿中心切
    try:
        ratio = float(request.args.get('ratio', 0.5))
    except (TypeError, ValueError):
        ratio = 0.5
    ratio = max(0.0, min(1.0, ratio))

    tmp_path = None
    try:
        from geometry_utils import shape_bbox
        from model_reader import read_product_model
        from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
        from OCP.gp import gp_Pnt

        shape = read_product_model(abs_path).shape
        bb = shape_bbox(shape)
        # 沿指定轴在 ratio 位置切去 -半边，保留 +半边：剖面暴露内部型腔
        keep_min = list(bb[0:3])
        keep_max = list(bb[3:6])
        keep_min[ai] = bb[ai] + (bb[ai + 3] - bb[ai]) * ratio
        # 切除的半边再外扩一点，确保整条剖面线干净无残留
        margin = 10.0
        if ratio >= 0.5:
            keep_min[ai] -= margin
        keep = BRepPrimAPI_MakeBox(
            gp_Pnt(keep_min[0], keep_min[1], keep_min[2]),
            gp_Pnt(keep_max[0], keep_max[1], keep_max[2]),
        ).Shape()
        cut = BRepAlgoAPI_Cut(shape, keep)
        cut.SetRunParallel(False)
        cut.SetFuzzyValue(1e-5)
        cut.Build()
        if not cut.IsDone() or cut.Shape().IsNull():
            return jsonify({'error': '半剖布尔运算失败'}), 500

        tmp_path = os.path.join(tempfile.gettempdir(), f"_section_{uuid.uuid4().hex[:8]}.step")
        from result_exporter import _write_step
        _write_step(cut.Shape(), tmp_path)
        data = _parse_step(tmp_path)
        return jsonify(data)
    except Exception as exc:
        return jsonify({'error': f'{type(exc).__name__}: {exc}'}), 500
    finally:
        if tmp_path:
            try:
                os.remove(tmp_path)
            except OSError:
                pass


# ============ 模型特征分析 API ============

@app.route('/api/model_features')
def api_model_features():
    """检测模型特征：折弯/圆孔/凹槽等"""
    filepath = _normalize_path(request.args.get('file', ''))
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404

    abs_path = os.path.abspath(filepath)
    allowed_dirs = [
        os.path.abspath(UPLOAD_DIR),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '三维数模')),
    ]
    if not any(abs_path.startswith(d) for d in allowed_dirs):
        return jsonify({'error': 'Access denied'}), 403

    # 缓存加载原始数据
    if abs_path in _raw_stp_cache:
        raw = _raw_stp_cache[abs_path]
    else:
        from step_reader import StepReader
        reader = StepReader(linear_deflection=0.5, angular_deflection=0.3)
        raw = reader.read(abs_path)
        _raw_stp_cache[abs_path] = raw

    # 使用 bend_detector 和 brep_topology 进行特征分析
    from brep_topology import BrepTopologyExtractor
    from bend_detector import BendDetector

    try:
        extractor = BrepTopologyExtractor(abs_path)
        topo = extractor.extract()

        detector = BendDetector(topo)
        bend_result = detector.detect()

        classification = topo.get('classification', {})
        faces = topo.get('faces', [])
        thickness = topo.get('thickness', 2.0)
        bbox = topo.get('bbox', {})
        bbox_size = bbox.get('size', [0, 0, 0]) if bbox else [0, 0, 0]

        bends = bend_result.get('bends', [])
        has_bends = len(bends) > 0

        # 圆孔检测
        planar_indices = classification.get('planar_faces', [])
        fillet_indices = {f['face_idx'] for f in classification.get('fillet_faces', [])}
        cylindrical_indices = classification.get('cylindrical_faces', [])

        main_normal = None
        max_area = -1
        for fi in planar_indices:
            f = faces[fi]
            area = f.get('area', 0)
            if area > max_area:
                max_area = area
                sf = f.get('surface', {})
                main_normal = sf.get('normal')

        holes = []
        for fi in cylindrical_indices:
            if fi in fillet_indices:
                continue
            f = faces[fi]
            sp = f.get('surface_params', {})
            radius = sp.get('radius')
            if radius is None:
                continue
            cyl_axis = sp.get('axis_direction')

            # 检查垂直方向
            is_vertical = False
            if main_normal and cyl_axis:
                dot = abs(main_normal[0] * cyl_axis[0] + main_normal[1] * cyl_axis[1] + main_normal[2] * cyl_axis[2])
                if dot > 0.9:
                    is_vertical = True

            if radius < 0.5:
                continue
            holes.append({
                'face_index': fi,
                'radius': round(radius, 2),
                'diameter': round(radius * 2, 2),
                'is_through_hole': True,
                'is_vertical': is_vertical,
            })

        # 折弯检测结果
        bend_features = []
        for b in bends:
            bend_features.append({
                'angle': round(b.get('angle', 0), 1),
                'radius': round(b.get('radius', 0), 2),
                'length': round(b.get('length', 0), 2),
                'edge_index': b.get('edge_index', -1),
                'type': b.get('type', 'bend'),
            })

        result = {
            'has_bends': has_bends,
            'bend_count': len(bend_features),
            'bends': bend_features,
            'holes': holes,
            'hole_count': len(holes),
            'thickness': round(thickness, 2),
            'bbox_size': [round(s, 2) for s in bbox_size],
            'classification': {
                'total_faces': len(faces),
                'planar_faces': len(planar_indices),
                'cylindrical_faces': len(cylindrical_indices),
                'fillet_faces': len(fillet_indices),
            },
        }
        return jsonify(result)

    except Exception as e:
        return jsonify({
            'error': str(e),
            'has_bends': False,
            'bend_count': 0,
            'bends': [],
            'holes': [],
            'hole_count': 0,
            'thickness': 0,
            'bbox_size': [0, 0, 0],
            'classification': {},
        }), 500


@app.route('/api/optimize_corners', methods=['POST'])
def api_optimize_corners():
    """产品自动化优化：闭合两端折弯角开口, 输出优化后产品 STEP 供预览/下载。"""
    data = request.get_json(force=True)
    filepath = data.get('filepath', '')
    try:
        fillet = float(data.get('fillet_r', 3.0))
    except (TypeError, ValueError):
        fillet = 3.0
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404
    abs_path = os.path.abspath(filepath)
    if not abs_path.startswith(os.path.abspath(UPLOAD_DIR)):
        return jsonify({'error': 'Access denied'}), 403
    try:
        import cadquery as cq
        from geometry_utils import shape_volume
        from corner_closer import close_corner_openings
        cs = list(cq.importers.importStep(abs_path).vals())[0]
        shape = cs.wrapped
        v0 = shape_volume(shape)
        fixed = close_corner_openings(shape, fillet_r=fillet, verbose=False)
        v1 = shape_volume(fixed)
        out_dir = os.path.join(UPLOAD_DIR, 'opt_output')
        os.makedirs(out_dir, exist_ok=True)
        clean = os.path.splitext(_strip_uid_prefix(abs_path))[0]
        out_path = os.path.join(out_dir, clean + '_closed.stp')
        # 用 cadquery 导出（cq.Shape.cast 才是完整实体；Workplane.newObject 会导出空 STP）
        cq.exporters.export(cq.Shape.cast(fixed), out_path,
                            exportType=cq.exporters.ExportTypes.STEP)
        parsed = _parse_step(out_path)
        parsed['file_name'] = clean + '_closed'
        parsed['filepath'] = out_path
        parsed['optimized_path'] = out_path
        parsed['volume_before_mm3'] = v0
        parsed['volume_after_mm3'] = v1
        return jsonify(parsed)
    except Exception as exc:
        return jsonify({'error': f'{type(exc).__name__}: {exc}'}), 500


@app.route('/api/download_opt')
def api_download_opt():
    """下载端角闭合优化后的产品 STEP。"""
    filepath = _normalize_path(request.args.get('file', ''))
    if not filepath or not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404
    abs_path = os.path.abspath(filepath)
    allowed = [os.path.abspath(os.path.join(UPLOAD_DIR, 'opt_output'))]
    if not any(abs_path.startswith(d) for d in allowed):
        return jsonify({'error': 'Access denied'}), 403
    return send_file(abs_path, mimetype='application/step', as_attachment=True,
                     download_name=_strip_uid_prefix(abs_path))


if __name__ == '__main__':
    # 端口：默认 5002。
    # 如需同时运行多个系统，用 --port 指定不同端口，无需改代码：
    #   python app.py --port 5001   → http://127.0.0.1:5001
    #   python app.py --port 5002   → http://127.0.0.1:5002
    #   python app.py --port 5003   → http://127.0.0.1:5003
    _port = 5002
    if '--port' in sys.argv:
        try:
            _port = int(sys.argv[sys.argv.index('--port') + 1])
        except (ValueError, IndexError):
            print("[App] --port 参数无效，使用默认端口 5002")
    # 开发服务器：0.0.0.0 允许局域网访问，debug 关闭避免双进程加载。
    # threaded=True：模具修复(约30-60秒)期间不阻塞其它请求(下载/查看等)。
    # ---- 启动自检：解析子进程池能不能建起来 ----------------------------------
    # 在受限/沙箱终端里启动时，系统不允许创建 multiprocessing 需要的管道/进程，
    # 池建不起来会让 /upload 直接 500（前端报 "Unexpected token '<' ... not valid JSON"）。
    # 这里提前自检并打印结论，便于一眼看出"是不是启动方式不对"。
    try:
        _get_parse_pool()
        print("[启动自检] ✔ 解析子进程池可用（上传/解析不会阻塞主进程）")
    except Exception as _exc:  # noqa: BLE001
        print(f"[启动自检] ✗ 解析子进程池不可用：{type(_exc).__name__}: {_exc}")
        print("           上传仍可用，但会在主进程内解析（大模型会卡住页面）。")
        print("           请改用**普通 CMD / PowerShell 窗口**启动（或双击 启动Web服务.bat），")
        print("           不要在受限的/沙箱化的终端里启动。")
    print(f"[App] 启动地址: http://127.0.0.1:{_port}   (局域网: http://<本机IP>:{_port})")
    app.run(host='0.0.0.0', port=_port, debug=False, threaded=True)
