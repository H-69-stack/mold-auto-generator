"""
模芯生成器 (core_block_generator.py) —— 模块 1

作者: 模具自动生成系统开发组
用途: 生成长方体模芯实体。
      - 支持直接指定尺寸 (L, W, H)；
      - 或仅输入余量 (margin_x, margin_y, margin_z)，根据产品包围盒自动计算。
约束: 模芯尺寸必须 >= 产品包围盒尺寸 + 双边余量。
输出: 底面中心位于 XY 原点、Z 从 0 到 H 的长方体实体。
"""

from dataclasses import dataclass
from typing import Optional, Tuple

from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt

from errors import MoldGenerationError
from geometry_utils import BBox

Size3 = Tuple[float, float, float]


@dataclass(frozen=True)
class CoreBlock:
    """模芯实体及其尺寸。

    solid: 底面中心位于原点、Z∈[0, H] 的长方体实体 (TopoDS_Solid)。
    size:  (L, W, H)。
    """

    solid: object   # TopoDS_Solid
    size: Size3     # (L, W, H)

    @property
    def volume(self) -> float:
        """模芯体积（mm³）。"""
        return self.size[0] * self.size[1] * self.size[2]


def bbox_size(bbox: BBox) -> Size3:
    """从包围盒计算尺寸 (L, W, H)。"""
    x_min, y_min, z_min, x_max, y_max, z_max = bbox
    return (x_max - x_min, y_max - y_min, z_max - z_min)


def compute_core_size(
    product_bbox: BBox,
    margin: Size3 = (20.0, 20.0, 20.0),
    base_thickness: float = 20.0,
) -> Size3:
    """根据产品包围盒 + 余量 + 底部基座厚度自动计算模芯尺寸。

    公式:
      L = 产品 L + 2 * margin_x
      W = 产品 W + 2 * margin_y
      H = base_thickness(底部基座/型腔底板) + 产品 H + margin_z(顶部余量)

    说明: Z 向采用“下基座 + 上余量”的不对称结构 —— 产品底面必须落在
    模芯底板上方，型腔底部才有实体材料（否则型腔会在模芯底面被挖穿，
    下模变成无底的通腔，这是错误的模具结构）。
    """
    lp, wp, hp = bbox_size(product_bbox)
    mx, my, mz = margin
    return (lp + 2.0 * mx, wp + 2.0 * my, base_thickness + hp + mz)


def _validate_core_size(
    core_size: Size3,
    product_bbox: BBox,
    margin: Size3,
    base_thickness: float = 20.0,
    tolerance: float = 1e-6,
) -> None:
    """校验模芯尺寸是否足够容纳产品 + 余量 + 底部基座。

    L/W 需 >= 产品 + 2*余量；H 需 >= base_thickness + 产品高 + 顶部余量。
    """
    lp, wp, hp = bbox_size(product_bbox)
    mx, my, mz = margin
    require = (lp + 2.0 * mx, wp + 2.0 * my, base_thickness + hp + mz)
    for name, core_dim, req_dim in zip(("L", "W", "H"), core_size, require):
        if core_dim < req_dim - tolerance:
            raise MoldGenerationError(
                f"模芯尺寸 {name}={core_dim:.2f} 小于所需尺寸 {req_dim:.2f} "
                f"(产品尺寸+余量+基座厚度)，请增大模芯尺寸或余量",
                stage="core_block_generator",
            )


def generate_core_block(
    core_size: Optional[Size3] = None,
    margin: Size3 = (20.0, 20.0, 20.0),
    product_bbox: Optional[BBox] = None,
    base_thickness: float = 20.0,
) -> CoreBlock:
    """生成模芯长方体实体。

    参数:
        core_size: 模芯尺寸 (L, W, H)。为 None 时若提供 product_bbox 则自动计算。
        margin: 自动计算时使用的余量 (mx, my, mz)。
        product_bbox: 产品包围盒 (x_min, y_min, z_min, x_max, y_max, z_max)。
        base_thickness: 自动计算时 Z 向底部基座厚度（型腔底板实体厚度，mm）。

    返回:
        CoreBlock —— solid 底面中心位于 XY 原点，Z 从 0 到 H。

    异常:
        MoldGenerationError: 尺寸缺失、非正数，或小于产品尺寸+双边余量。
    """
    if core_size is None:
        if product_bbox is None:
            raise MoldGenerationError(
                "generate_core_block: core_size 与 product_bbox 至少提供一个",
                stage="core_block_generator",
            )
        core_size = compute_core_size(product_bbox, margin, base_thickness)

    if any(dim <= 0.0 for dim in core_size):
        raise MoldGenerationError(
            f"模芯尺寸必须为正数: {core_size}",
            stage="core_block_generator",
        )

    if product_bbox is not None:
        _validate_core_size(core_size, product_bbox, margin, base_thickness)

    L, W, H = core_size
    box = BRepPrimAPI_MakeBox(
        gp_Pnt(-L / 2.0, -W / 2.0, 0.0), gp_Pnt(L / 2.0, W / 2.0, H)
    )
    return CoreBlock(solid=box.Shape(), size=(L, W, H))


if __name__ == "__main__":
    # 自测入口
    from io_utils import ensure_utf8_stdout

    ensure_utf8_stdout()
    demo_bbox = (0.0, 0.0, 0.0, 60.0, 40.0, 25.0)
    core = generate_core_block(product_bbox=demo_bbox, margin=(20.0, 20.0, 20.0))
    print(f"自动计算模芯尺寸: L×W×H = {core.size}, 体积 = {core.volume:.1f} mm³")
    core2 = generate_core_block(core_size=(200.0, 150.0, 100.0))
    print(f"指定尺寸模芯: {core2.size}")
    try:
        generate_core_block(core_size=(50.0, 50.0, 50.0),
                            product_bbox=demo_bbox, margin=(20.0, 20.0, 20.0))
    except MoldGenerationError as exc:
        print(f"约束校验生效: {exc}")
