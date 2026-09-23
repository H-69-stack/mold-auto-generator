"""
型腔挖除器 (cavity_cutter.py) —— 模块 4

作者: 模具自动生成系统开发组
用途: 在模芯中挖出产品形状的型腔：BRepAlgoAPI_Cut(模芯, 产品)。
      带容错与重试（尝试不同 FuzzyValue 容差），
      并校验结果有效性（IsNull / BRepCheck）与体积合理性。
"""

from typing import Iterable, Optional, Tuple

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

from errors import BooleanOpError
from geometry_utils import is_valid_shape, shape_volume

DEFAULT_FUZZY_VALUES: Tuple[float, ...] = (0.0, 1e-6, 1e-5, 1e-4)


def bool_cut(
    shape1,
    shape2,
    fuzzy_values: Iterable[float] = DEFAULT_FUZZY_VALUES,
    run_parallel: bool = False,
    label: str = "布尔求差",
):
    """执行 shape1 - shape2 布尔求差，带 FuzzyValue 容错重试。

    采用本工程验证过的稳定调用方式：
        op = BRepAlgoAPI_Cut(shape1, shape2)
        op.SetRunParallel(False); op.SetFuzzyValue(fv); op.Build()

    参数:
        shape1: 被减形状。
        shape2: 减去的形状。
        fuzzy_values: 依次尝试的容差序列（0 表示默认容差）。
        run_parallel: 是否开启并行布尔（默认关闭，更稳定）。
        label: 用于错误信息的操作名。

    返回:
        TopoDS_Shape。

    异常:
        BooleanOpError: 所有容差尝试均失败或结果为空。
    """
    last_error = None
    for fv in fuzzy_values:
        try:
            op = BRepAlgoAPI_Cut(shape1, shape2)
            op.SetRunParallel(run_parallel)
            op.SetFuzzyValue(fv)
            op.Build()
            if not op.IsDone():
                last_error = f"BOP 未完成 (fuzzy={fv})"
                continue
            result = op.Shape()
            if result is None or result.IsNull():
                last_error = f"结果为空 (fuzzy={fv})"
                continue
            return result
        except Exception as exc:  # noqa: BLE001 —— 重试型容错，需要兜住所有异常
            last_error = f"{type(exc).__name__}: {exc} (fuzzy={fv})"
            continue
    raise BooleanOpError(
        f"{label}失败：已尝试 FuzzyValue={list(fuzzy_values)}，最后错误: {last_error}"
    )


def bool_common(
    shape1,
    shape2,
    fuzzy_values: Iterable[float] = DEFAULT_FUZZY_VALUES,
    run_parallel: bool = False,
    label: str = "布尔交集",
):
    """执行 shape1 ∩ shape2 布尔交集，带 FuzzyValue 容错重试。

    调用方式与 bool_cut 一致，用于沿轮廓柱体提取上模等场景。
    """
    last_error = None
    for fv in fuzzy_values:
        try:
            op = BRepAlgoAPI_Common(shape1, shape2)
            op.SetRunParallel(run_parallel)
            op.SetFuzzyValue(fv)
            op.Build()
            if not op.IsDone():
                last_error = f"BOP 未完成 (fuzzy={fv})"
                continue
            result = op.Shape()
            if result is None or result.IsNull():
                last_error = f"结果为空 (fuzzy={fv})"
                continue
            return result
        except Exception as exc:  # noqa: BLE001
            last_error = f"{type(exc).__name__}: {exc} (fuzzy={fv})"
            continue
    raise BooleanOpError(
        f"{label}失败：已尝试 FuzzyValue={list(fuzzy_values)}，最后错误: {last_error}"
    )


def bool_fuse(
    shape1,
    shape2,
    fuzzy_values: Iterable[float] = DEFAULT_FUZZY_VALUES,
    run_parallel: bool = False,
    label: str = "布尔并集",
):
    """执行 shape1 ∪ shape2 布尔并集，带 FuzzyValue 容错重试。

    用于把多个朝下面拉伸成的柱体融合成"型腔下表面以下区域"。
    """
    last_error = None
    for fv in fuzzy_values:
        try:
            op = BRepAlgoAPI_Fuse(shape1, shape2)
            op.SetRunParallel(run_parallel)
            op.SetFuzzyValue(fv)
            op.Build()
            if not op.IsDone():
                last_error = f"BOP 未完成 (fuzzy={fv})"
                continue
            result = op.Shape()
            if result is None or result.IsNull():
                last_error = f"结果为空 (fuzzy={fv})"
                continue
            return result
        except Exception as exc:  # noqa: BLE001
            last_error = f"{type(exc).__name__}: {exc} (fuzzy={fv})"
            continue
    raise BooleanOpError(
        f"{label}失败：已尝试 FuzzyValue={list(fuzzy_values)}，最后错误: {last_error}"
    )


def _ensure_single_solid(shape):
    """若结果为复合体，返回其中唯一实体；存在多个分离实体时尝试融合。"""
    solids = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        solids.append(exp.Current())
        exp.Next()
    if not solids:
        raise BooleanOpError("布尔运算结果中不包含实体 (solid)")
    if len(solids) == 1:
        return solids[0]
    fused = solids[0]
    for s in solids[1:]:
        op = BRepAlgoAPI_Fuse(fused, s)
        op.SetRunParallel(False)
        op.Build()
        if not op.IsDone() or op.Shape().IsNull():
            raise BooleanOpError("复合体结果融合失败")
        fused = op.Shape()
    return fused


def cut_cavity(
    core_solid,
    product_solid,
    expected_core_volume: Optional[float] = None,
    expected_product_volume: Optional[float] = None,
    fuzzy_values: Iterable[float] = DEFAULT_FUZZY_VALUES,
):
    """在模芯中挖出产品形状的型腔。

    参数:
        core_solid: 模芯实体 (TopoDS_Shape)。
        product_solid: 居中后的产品实体 (TopoDS_Shape)。
        expected_core_volume: 模芯原始体积（用于校验，可选）。
        expected_product_volume: 产品体积（用于校验，可选）。

    返回:
        带型腔的模芯实体 (TopoDS_Shape)。

    异常:
        BooleanOpError: 布尔运算失败或结果无效。
    """
    result = bool_cut(core_solid, product_solid, fuzzy_values, label="型腔挖除")
    result = _ensure_single_solid(result)

    if not is_valid_shape(result, check_geometry=True):
        print("[警告] 型腔布尔运算结果未通过 BRepCheck 几何校验，请检查模型")

    vol = shape_volume(result)
    if vol <= 0.0:
        raise BooleanOpError(f"型腔模芯体积异常 ({vol:.3f})，布尔运算结果无效")

    # 核心校验：挖掉的体积（模芯 - 型腔模芯）必须等于产品体积，
    # 即型腔是与产品一模一样大小的空腔
    if expected_core_volume and expected_product_volume:
        removed = expected_core_volume - vol
        dev = abs(removed - expected_product_volume) / expected_product_volume
        print(
            f"    校验: 挖掉体积 {removed:.2f} mm³ ≈ 产品体积 "
            f"{expected_product_volume:.2f} mm³ (偏差 {dev * 100:.4f}%)"
        )
        if dev > 0.05:
            raise BooleanOpError(
                f"型腔挖除校验失败：挖掉体积 {removed:.3f} 与产品体积 "
                f"{expected_product_volume:.3f} 偏差 {dev * 100:.2f}%，"
                "请检查产品是否完全位于模芯内部"
            )
    return result


if __name__ == "__main__":
    # 自测入口
    from io_utils import ensure_utf8_stdout

    ensure_utf8_stdout()
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    core = BRepPrimAPI_MakeBox(gp_Pnt(-50.0, -50.0, 0.0), gp_Pnt(50.0, 50.0, 60.0)).Shape()
    product = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 15.0, 30.0
    ).Shape()
    cavity = cut_cavity(
        core, product,
        expected_core_volume=shape_volume(core),
        expected_product_volume=shape_volume(product),
    )
    print(f"模芯体积: {shape_volume(core):.3f} mm³")
    print(f"型腔模芯体积: {shape_volume(cavity):.3f} mm³")
    print(f"校验通过: {is_valid_shape(cavity)}")
