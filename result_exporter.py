"""
结果导出器 (result_exporter.py) —— 模块 6

作者: 模具自动生成系统开发组
用途: 将上模、下模导出为 STEP 文件，并同时输出 JSON 元数据文件。

输出:
  {prefix}_upper_mold.step
  {prefix}_lower_mold.step
  {prefix}_metadata.json

说明:
  - STEP 优先使用 AP214（完整高级 BREP），失败时自动降级重试；
  - 目标路径含非 ASCII 字符时，先写到 ASCII 临时路径再移动回原路径
    （OCC 底层文件 API 对中文路径支持不佳）。
"""

import json
from pathlib import Path
from typing import Any, Dict, Optional

from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer

from errors import ExportError
from io_utils import make_ascii_write_path, move_into_place

# AP214 默认（完整高级 BREP），按顺序降级尝试
STEP_SCHEMAS = ("AP214IS", "AP214DIS", "AP203")


def _write_step(shape, path: str) -> None:
    """将实体写入 STEP 文件（AP214，含 schema 降级重试）。"""
    target, is_tmp = make_ascii_write_path(path)
    last_error = None
    for schema in STEP_SCHEMAS:
        try:
            Interface_Static.SetCVal_s("write.step.schema", schema)
            writer = STEPControl_Writer()
            status = writer.Transfer(shape, STEPControl_AsIs)
            if status != IFSelect_RetDone:
                last_error = f"Transfer 失败 (schema={schema}, status={status})"
                continue
            write_status = writer.Write(target)
            if write_status != IFSelect_RetDone:
                last_error = f"Write 失败 (schema={schema}, status={write_status})"
                continue
            if not Path(target).exists() or Path(target).stat().st_size == 0:
                last_error = f"写出文件为空 (schema={schema})"
                continue
            if is_tmp:
                move_into_place(target, path)
            return
        except Exception as exc:  # noqa: BLE001 —— 逐个 schema 兜底重试
            last_error = f"{type(exc).__name__}: {exc} (schema={schema})"
            continue
    raise ExportError(f"STEP 导出失败: {path}，最后错误: {last_error}")


def _sanitize(obj: Any) -> Any:
    """将元数据中的 tuple / 非 JSON 类型转换为 JSON 可序列化对象。"""
    if isinstance(obj, dict):
        return {str(k): _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(v) for v in obj]
    if isinstance(obj, (int, float, str, bool)) or obj is None:
        return obj
    return str(obj)


def export_molds(
    upper_mold,
    lower_mold,
    output_dir: str = "./output",
    prefix: str = "mold",
    metadata: Optional[Dict[str, Any]] = None,
    extra_shapes: Optional[Dict[str, Any]] = None,
) -> Dict[str, str]:
    """导出模具相关 STEP 文件与 JSON 元数据。

    参数:
        upper_mold: 上模实体 (TopoDS_Shape)。
        lower_mold: 下模实体 (TopoDS_Shape)。
        output_dir: 输出目录（不存在时自动创建）。
        prefix: 文件名前缀，导出为 {prefix}_upper_mold.step 等。
        metadata: 附加元数据（模芯尺寸、产品尺寸、分模高度、倒扣警告等）。
        extra_shapes: 额外实体字典 {名称: TopoDS_Shape}，导出为 {prefix}_{名称}.step
            （如型腔模芯 {"cavity_core": ...}）。

    返回:
        {"upper": 上模文件路径, "lower": 下模文件路径, "metadata": 元数据文件路径,
         **{额外名称: 对应 STEP 文件路径}}
    """
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    upper_path = out_dir / f"{prefix}_upper_mold.step"
    lower_path = out_dir / f"{prefix}_lower_mold.step"

    print(f"  - 导出上模 -> {upper_path}")
    _write_step(upper_mold, str(upper_path))
    print(f"  - 导出下模 -> {lower_path}")
    _write_step(lower_mold, str(lower_path))

    meta = dict(metadata or {})
    meta.setdefault("upper_mold_file", upper_path.name)
    meta.setdefault("lower_mold_file", lower_path.name)

    paths = {"upper": str(upper_path), "lower": str(lower_path)}
    for name, shape in (extra_shapes or {}).items():
        safe_name = "".join(c for c in str(name) if c.isalnum() or c in "-_") or "shape"
        shape_path = out_dir / f"{prefix}_{safe_name}.step"
        print(f"  - 导出{safe_name} -> {shape_path}")
        _write_step(shape, str(shape_path))
        meta.setdefault(f"{safe_name}_file", shape_path.name)
        paths[safe_name] = str(shape_path)

    meta_path = out_dir / f"{prefix}_metadata.json"
    with open(meta_path, "w", encoding="utf-8") as fh:
        json.dump(_sanitize(meta), fh, ensure_ascii=False, indent=2)
    print(f"  - 元数据 -> {meta_path}")

    paths["metadata"] = str(meta_path)
    return paths


def export_shapes(
    shapes: Dict[str, Any],
    output_dir: str = "./output",
    prefix: str = "shape",
    metadata: Optional[Dict[str, Any]] = None,
) -> Dict[str, str]:
    """导出任意一组实体：{prefix}_{名称}.step + {prefix}_metadata.json。

    用于模块化管线之外的独立功能模块（如模芯设计 core_design 的芯棒）。
    """
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    meta = dict(metadata or {})
    paths: Dict[str, str] = {}
    for name, shape in shapes.items():
        safe_name = "".join(c for c in str(name) if c.isalnum() or c in "-_") or "shape"
        shape_path = out_dir / f"{prefix}_{safe_name}.step"
        print(f"  - 导出{safe_name} -> {shape_path}")
        _write_step(shape, str(shape_path))
        meta.setdefault(f"{safe_name}_file", shape_path.name)
        paths[safe_name] = str(shape_path)
    meta_path = out_dir / f"{prefix}_metadata.json"
    with open(meta_path, "w", encoding="utf-8") as fh:
        json.dump(_sanitize(meta), fh, ensure_ascii=False, indent=2)
    print(f"  - 元数据 -> {meta_path}")
    paths["metadata"] = str(meta_path)
    return paths


if __name__ == "__main__":
    # 自测入口
    from io_utils import ensure_utf8_stdout
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt

    ensure_utf8_stdout()
    box = BRepPrimAPI_MakeBox(gp_Pnt(-30.0, -30.0, 0.0), gp_Pnt(30.0, 30.0, 10.0)).Shape()
    paths = export_molds(
        box, box, output_dir="./_export_test", prefix="demo",
        metadata={"note": "自测导出", "volume_mm3": 36000.0},
    )
    print(paths)
