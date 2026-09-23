"""
产品数模读取器 (model_reader.py) —— 模块 2

作者: 模具自动生成系统开发组
用途: 读取产品三维数模并提取几何数据：
      - TopoDS_Shape 产品实体
      - 包围盒 (x_min, y_min, z_min, x_max, y_max, z_max)
      - 体积、表面积（用于校验）
      支持 STEP (.stp/.step)、IGES (.igs/.iges) 与 STL (.stl) 格式自动识别；
      STL 是三角网格，会先缝合成闭合壳体、再做成实体（见 mesh_solid.py）；
      读取失败时抛出 ModelReadError（明确异常信息）。
"""

from dataclasses import dataclass
from pathlib import Path

from OCP.IFSelect import IFSelect_RetDone
from OCP.IGESControl import IGESControl_Reader
from OCP.STEPControl import STEPControl_Reader

from errors import ModelReadError
from geometry_utils import BBox, shape_bbox, shape_surface_area, shape_volume
from io_utils import make_ascii_read_path

STEP_EXTS = {".stp", ".step"}
IGES_EXTS = {".iges", ".igs"}
STL_EXTS = {".stl"}


@dataclass
class ProductModel:
    """产品数模的几何信息。

    shape:        产品实体 (TopoDS_Shape)。
    bbox:         包围盒 (x_min, y_min, z_min, x_max, y_max, z_max)。
    volume:       体积（mm³）。
    surface_area: 表面积（mm²）。
    file_path:    来源文件路径（内存构造时如 "<generated test part>"）。
    file_format:  "step" / "iges" / "stl" / "memory"。
    mesh_info:    仅 STL 有：{"sew_tolerance", "free_edges", "faces",
                  "volume", "valid"} —— 用来在界面/日志里说明"多面体近似"。
    """

    shape: object          # TopoDS_Shape
    bbox: BBox
    volume: float
    surface_area: float
    file_path: str
    file_format: str
    mesh_info: dict = None


def _detect_format(file_path: str) -> str:
    """按扩展名自动识别格式。"""
    suffix = Path(file_path).suffix.lower()
    if suffix in STEP_EXTS:
        return "step"
    if suffix in IGES_EXTS:
        return "iges"
    if suffix in STL_EXTS:
        return "stl"
    raise ModelReadError(
        f"不支持的文件格式: {suffix or '(无扩展名)'}，"
        f"支持 {sorted(STEP_EXTS | IGES_EXTS | STL_EXTS)}"
    )


def _read_step(path: str):
    """读取 STEP 文件，返回 TopoDS_Shape。"""
    reader = STEPControl_Reader()
    status = reader.ReadFile(path)
    if status != IFSelect_RetDone:
        raise ModelReadError(f"STEP 读取失败 (status={status}): {path}")
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape is None or shape.IsNull():
        raise ModelReadError(f"STEP 文件中未解析到有效几何体: {path}")
    return shape


def _read_iges(path: str):
    """读取 IGES 文件，返回 TopoDS_Shape。"""
    reader = IGESControl_Reader()
    status = reader.ReadFile(path)
    if status != IFSelect_RetDone:
        raise ModelReadError(f"IGES 读取失败 (status={status}): {path}")
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape is None or shape.IsNull():
        raise ModelReadError(f"IGES 文件中未解析到有效几何体: {path}")
    return shape


def read_product_model(file_path: str, verbose: bool = True) -> ProductModel:
    """读取产品数模并提取几何数据。

    参数:
        file_path: 产品数模路径 (.stp/.step/.iges/.igs/.stl)。
        verbose:   STL 缝合时是否打印提示（面数/多面体近似警告）。

    返回:
        ProductModel。

    异常:
        ModelReadError: 文件不存在、格式不支持或读取失败。
                       （STL 不闭合 / 体积为 0 也会在这里明确报错）
    """
    src = Path(file_path)
    if not src.exists():
        raise ModelReadError(f"产品数模文件不存在: {file_path}")
    fmt = _detect_format(file_path)

    # 非 ASCII 路径先复制到 ASCII 临时路径（OCC 底层文件 API 对中文路径支持不佳）
    read_path, is_tmp = make_ascii_read_path(str(src))
    stl_info = None
    try:
        if fmt == "step":
            shape = _read_step(read_path)
        elif fmt == "iges":
            shape = _read_iges(read_path)
        else:
            from mesh_solid import mesh_to_solid, read_stl_shape
            shape, stl_info = mesh_to_solid(read_stl_shape(read_path), verbose=verbose)
    finally:
        if is_tmp:
            try:
                Path(read_path).unlink(missing_ok=True)
            except OSError:
                pass

    bbox = shape_bbox(shape)
    volume = shape_volume(shape)
    area = shape_surface_area(shape)
    if volume <= 0.0:
        print(f"[警告] 产品体积 = {volume:.3f} mm³，请确认数模为闭合实体")
    return ProductModel(
        shape=shape, bbox=bbox, volume=volume, surface_area=area,
        file_path=str(src.resolve()), file_format=fmt, mesh_info=stl_info,
    )


def product_model_from_shape(shape, source: str = "<generated>") -> ProductModel:
    """把内存中的形状包装为 ProductModel（用于内置测试产品等场景）。"""
    bbox = shape_bbox(shape)
    return ProductModel(
        shape=shape, bbox=bbox,
        volume=shape_volume(shape),
        surface_area=shape_surface_area(shape),
        file_path=source, file_format="memory",
    )


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("用法: python model_reader.py <产品数模路径>")
        raise SystemExit(1)
    model = read_product_model(sys.argv[1])
    print(f"文件: {model.file_path} ({model.file_format})")
    print(f"包围盒: {model.bbox}")
    print(f"体积: {model.volume:.3f} mm³, 表面积: {model.surface_area:.3f} mm²")
