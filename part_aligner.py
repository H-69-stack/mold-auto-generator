"""
产品居中定位器 (part_aligner.py) —— 模块 3

作者: 模具自动生成系统开发组
用途: 将产品自动移动到模芯中心：
      1. 计算产品包围盒中心 (cx, cy, cz)；
      2. 目标位置：XY 方向与模芯中心对齐（移到 XY 原点），
         Z 方向可选“底面贴合模芯底面”或“Z 向居中”；
      3. 用 gp_Trsf 做平移变换；
      4. 校验平移后产品是否完全位于模芯内部。
"""

from dataclasses import dataclass
from typing import Tuple

from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Pnt, gp_Trsf, gp_Vec

from errors import MoldGenerationError
from geometry_utils import BBox, shape_bbox

Size3 = Tuple[float, float, float]


@dataclass(frozen=True)
class AlignmentResult:
    """对齐结果。

    aligned_shape: 平移后的产品实体 (TopoDS_Shape)。
    offset:        平移量 (dx, dy, dz)。
    aligned_bbox:  平移后的产品包围盒。
    """

    aligned_shape: object                  # TopoDS_Shape
    offset: Tuple[float, float, float]     # (dx, dy, dz)
    aligned_bbox: BBox


def align_product_to_core(
    product_shape,
    core_size: Size3,
    z_align_mode: str = "bottom",
    base_thickness: float = 20.0,
    tolerance: float = 1e-4,
) -> AlignmentResult:
    """将产品平移到模芯中心。

    参数:
        product_shape: 产品实体 (TopoDS_Shape)。
        core_size: 模芯尺寸 (L, W, H)，底面中心位于原点、Z∈[0, H]。
        z_align_mode:
            "bottom" —— 产品底面落在基座顶面 (z_min -> base_thickness)，默认；
            "center" —— 产品 Z 向居中。
        base_thickness: 底部基座厚度（mm）。产品底面必须位于模芯底板上方，
            型腔底部才有实体底板材料；若为 0，产品底面将贴模芯底面，
            型腔会在底部被挖穿（非正确模具结构）。
        tolerance: 产品“完全在模芯内部”检查的容差。

    返回:
        AlignmentResult。

    异常:
        MoldGenerationError: 模式非法，或平移后产品超出模芯范围。
    """
    if z_align_mode not in ("bottom", "center"):
        raise MoldGenerationError(
            f"非法 z_align_mode: {z_align_mode!r}（可选 bottom / center）",
            stage="part_aligner",
        )

    L, W, H = core_size
    x_min, y_min, z_min, x_max, y_max, z_max = shape_bbox(product_shape)

    dx = -0.5 * (x_min + x_max)
    dy = -0.5 * (y_min + y_max)
    if z_align_mode == "bottom":
        dz = base_thickness - z_min          # 产品底面 -> 基座顶面 z=base_thickness
    else:
        dz = H / 2.0 - 0.5 * (z_min + z_max)

    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(dx, dy, dz))
    aligned = BRepBuilderAPI_Transform(product_shape, trsf, True, False).Shape()

    aligned_bbox = shape_bbox(aligned)
    if not _is_inside_core(aligned_bbox, core_size, tolerance):
        raise MoldGenerationError(
            "产品平移后超出模芯范围，请增大模芯尺寸或余量。\n"
            f"  对齐后产品包围盒: {[round(v, 3) for v in aligned_bbox]}\n"
            f"  模芯范围: x∈[{-L/2:.1f}, {L/2:.1f}], "
            f"y∈[{-W/2:.1f}, {W/2:.1f}], z∈[0, {H:.1f}]",
            stage="part_aligner",
        )
    return AlignmentResult(
        aligned_shape=aligned, offset=(dx, dy, dz), aligned_bbox=aligned_bbox
    )


def _is_inside_core(bbox: BBox, core_size: Size3, tolerance: float) -> bool:
    """判断包围盒是否完全位于模芯内部（含容差）。"""
    L, W, H = core_size
    x_min, y_min, z_min, x_max, y_max, z_max = bbox
    return (
        x_min >= -L / 2.0 - tolerance
        and x_max <= L / 2.0 + tolerance
        and y_min >= -W / 2.0 - tolerance
        and y_max <= W / 2.0 + tolerance
        and z_min >= -tolerance
        and z_max <= H + tolerance
    )


if __name__ == "__main__":
    # 自测入口
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

    demo = BRepPrimAPI_MakeBox(gp_Pnt(10.0, 20.0, 5.0), gp_Pnt(70.0, 60.0, 30.0)).Shape()
    for mode in ("bottom", "center"):
        res = align_product_to_core(demo, core_size=(200.0, 160.0, 100.0),
                                    z_align_mode=mode)
        print(f"mode={mode:6s} 平移量={tuple(round(v, 3) for v in res.offset)} "
              f"对齐后包围盒={[round(v, 3) for v in res.aligned_bbox]}")
    try:
        align_product_to_core(demo, core_size=(50.0, 50.0, 20.0), z_align_mode="bottom")
    except MoldGenerationError as exc:
        print(f"越界校验生效: {exc}")
