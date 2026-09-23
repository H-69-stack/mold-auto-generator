"""
模具自动生成系统主程序 (main.py)

作者: 模具自动生成系统开发组
用途: 串联 6 个模块，完成“产品数模 -> 上模/下模 STEP”的完整流程：
      1. 产品读取     (model_reader)
      2. 模芯生成     (core_block_generator)
      3. 居中定位     (part_aligner)
      4. 型腔挖除     (cavity_cutter)
      5. 分模         (parting_splitter)
      6. 结果导出     (result_exporter)

入口:
  generate_mold(part_path, core_size=None, margin=(20,20,20),
                parting_mode="max_z", z_align="bottom", output_dir="./output")
  main()  —— 命令行入口；--part 指定产品数模，缺省使用内置测试产品跑通全流程。
"""

import argparse
import sys
import time
from typing import Optional, Tuple

from OCP.BRep import BRep_Builder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.TopoDS import TopoDS_Compound

from cavity_cutter import cut_cavity
from core_block_generator import compute_core_size, generate_core_block
from errors import MoldGenerationError
from geometry_utils import shape_bbox, shape_volume
from io_utils import ensure_utf8_stdout
from model_reader import product_model_from_shape, read_product_model
from part_aligner import align_product_to_core
from parting_splitter import split_mold
from result_exporter import export_molds

Size3 = Tuple[float, float, float]


def build_core_assembly(product_shape, cavity_core):
    """构建模芯装配体（compound）：产品实体 + 型腔模芯实体。

    两个结构体互补，共同组成一个完整长方体模芯：
        product ∪ cavity_core = 完整模芯
    产品放置于型腔之中，二者不重叠（产品体积 + 型腔模芯体积 = 模芯体积），
    预览时可直接看到“产品嵌在挖出的型腔里”的整体效果。

    返回:
        TopoDS_Compound —— [产品实体, 型腔模芯实体]
    """
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    builder.Add(compound, product_shape)   # 先产品，便于前端按序着色
    builder.Add(compound, cavity_core)     # 再型腔模芯
    return compound


def build_mold_assembly(upper_mold, product_shape, lower_mold):
    """构建“上模 + 产品 + 下模”三件套装配体（compound，按实际配合位置摆好）。

    三个实体在同一坐标系中，产品正好嵌在上下模的型腔里（合模状态，互不重叠），
    导出为一个 STEP 文件，便于直接打开核对装配关系或做干涉检查。

    返回:
        TopoDS_Compound —— [上模, 产品, 下模]
    """
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    builder.Add(compound, upper_mold)
    builder.Add(compound, product_shape)
    builder.Add(compound, lower_mold)
    return compound


def suggest_pipe_plane(part_path: Optional[str] = None, core_size: Optional[Size3] = None,
                       margin: Size3 = (20.0, 20.0, 20.0),
                       axis: Optional[int] = None) -> dict:
    """**快速给出管件水平分模面的推荐高度 + 安全区间 + 对应上/下模高度**。

    不跑任何布尔运算（只是表面采样 + 侧影/成形面分析，大件约几十秒），
    所以可以在正式生成模具前先算出来给用户参考/微调。
    """
    from pipe_mold import suggest_plane_height

    if not part_path:
        raise MoldGenerationError("需要指定产品数模（--part）", stage="main")
    product = read_product_model(part_path)
    if core_size is None:
        core_size = compute_core_size(product.bbox, margin, base_thickness=margin[2])
    core = generate_core_block(core_size=core_size, margin=margin,
                               product_bbox=product.bbox, base_thickness=margin[2])
    aligned = align_product_to_core(product.shape, core.size, z_align_mode="center",
                                    base_thickness=0.0)
    bz = shape_bbox(core.solid)
    info = suggest_plane_height(aligned.aligned_shape, block_z=(bz[2], bz[5]),
                                axis=axis)
    info.update({
        "product_file": product.file_path,
        "core_size_mm": list(core.size),
        "align_offset_mm": list(aligned.offset),
        "product_z_mm": [aligned.aligned_bbox[2], aligned.aligned_bbox[5]],
    })
    return info


def _shift_shape(shape, vec):
    """把形状平移 vec（用于把导出件从"模芯坐标系"搬回输入数模的原始坐标系）。"""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf, gp_Vec

    tr = gp_Trsf()
    tr.SetTranslation(gp_Vec(float(vec[0]), float(vec[1]), float(vec[2])))
    return BRepBuilderAPI_Transform(shape, tr, True, False).Shape()


def build_mold4_assembly(upper_mold, product_shape, core_pin, lower_mold):
    """构建“上模 + 产品 + 芯棒 + 下模”四件套装配体（合模状态，同一坐标系）。

    芯棒来自"挖外形包络时被剔除的内芯条"—— 它和上下模是**同一次布尔运算**切出来的
    互补两块，所以四者严格互不重叠、天然正确配合，不需要额外做干涉检查：

        (取芯块 − 产品) = 模具主体 + 内孔腔(芯棒) + 封盖切掉的料

    返回:
        TopoDS_Compound —— [上模, 产品, 芯棒, 下模]
    """
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    builder.Add(compound, upper_mold)
    builder.Add(compound, product_shape)
    builder.Add(compound, core_pin)
    builder.Add(compound, lower_mold)
    return compound


def make_test_product():
    """生成内置测试产品：带圆柱凸台的长方体（底面中心位于原点，Z∈[0, 25]）。

    长方体 60×40×10 + 圆柱凸台(Φ24×15)，体积 = 30785.84 mm³。
    """
    base = BRepPrimAPI_MakeBox(
        gp_Pnt(-30.0, -20.0, 0.0), gp_Pnt(30.0, 20.0, 10.0)
    ).Shape()
    boss = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 10.0), gp_Dir(0.0, 0.0, 1.0)), 12.0, 15.0
    ).Shape()
    fuse = BRepAlgoAPI_Fuse(base, boss)
    fuse.SetRunParallel(False)
    fuse.Build()
    if not fuse.IsDone() or fuse.Shape().IsNull():
        raise MoldGenerationError("测试产品生成失败", stage="main")
    return fuse.Shape()


def generate_mold(
    part_path: Optional[str] = None,
    core_size: Optional[Size3] = None,
    margin: Size3 = (20.0, 20.0, 20.0),
    base_thickness: float = 20.0,
    parting_mode: str = "max_z",
    z_parting: Optional[float] = None,
    z_align: str = "bottom",
    output_dir: str = "./output",
    prefix: str = "mold",
    close_corners_r: float = 0.0,
    mold_type: str = "sheet",
    parting_surface: str = "plane",
    verify: bool = True,
    with_core: bool = True,
    run_parallel: bool = False,
    full_outputs: bool = False,
) -> dict:
    """模具自动生成完整流程。

    参数:
        part_path: 产品数模路径 (.stp/.step/.iges/.igs)；None 时使用内置测试产品。
        core_size: 模芯尺寸 (L, W, H)；None 时按产品包围盒 + 余量自动计算。
        margin: 自动计算模芯时的余量 (mx, my, mz)。
        base_thickness: 底部基座厚度（mm，默认 20）。产品底面落在基座顶面，
            型腔底部保留实体底板材料，避免型腔在模芯底面被挖穿。
        parting_mode: 分模模式 "cavity_bottom"（以型腔下表面为分模面）/
            "contour"（沿产品投影轮廓，max_z 高度）/
            "max_z" / "mid" / "custom"。
        z_parting: custom 模式下的分模高度。
        z_align: 产品 Z 向定位方式 "bottom"（底面落在基座顶面）或 "center"（居中）。
        output_dir: 输出目录。
        prefix: 导出文件名前缀。
        close_corners_r: >0 时先闭合产品两端折弯角开口（板件用）。
        mold_type: 模具工艺类型
            "sheet" —— 板件折弯/液压成型（原管线：挖产品实体 + 平面/轮廓分模）；
            "pipe"  —— 管件硬模成型（产品居中 + 外形包络型腔 + 分模面 + 芯棒 + 装配体）。
        parting_surface: 仅管件模式使用——分模面形式
            "plane"（默认，水平面，最好加工）/ "silhouette"（沿产品侧影随形）。
        verify: 仅管件模式使用——是否做开模/顶出干涉检验（6 次布尔求交，很慢；关掉省 ~5 min）。
        with_core: 仅管件模式使用——是否把"内孔腔（芯棒）"一起取出并导出
            **四件套装配体（上模 + 产品 + 芯棒 + 下模）**。芯棒是挖外形包络时顺带切出来的，
            不额外花布尔时间（默认开）。
        run_parallel: 布尔运算是否开 OCC 并行（默认关；多核机器上可能明显提速，但偶有不稳）。

    返回:
        dict: 包含各步骤结果与导出路径的摘要。

    异常:
        MoldGenerationError: 任一步骤失败（完整错误信息）。
    """
    if mold_type not in ("sheet", "pipe"):
        raise MoldGenerationError(
            f"未知模具类型 mold_type={mold_type!r}（可选 sheet / pipe）",
            stage="main",
        )
    if mold_type == "pipe":
        return _generate_pipe_mold(
            part_path=part_path, core_size=core_size, margin=margin,
            output_dir=output_dir, prefix=prefix, parting_surface=parting_surface,
            verify=verify, with_core=with_core, run_parallel=run_parallel,
            parting_z=z_parting, full_outputs=full_outputs,
        )

    step = 0

    def progress(message: str) -> None:
        nonlocal step
        step += 1
        print(f"[{step}/6] {message}")

    t0 = time.perf_counter()

    # ---- 1. 产品读取 ----
    if part_path:
        progress(f"读取产品数模: {part_path}")
        product = read_product_model(part_path)
    else:
        progress("未指定产品文件，生成内置测试产品（带凸台长方体）")
        product = product_model_from_shape(
            make_test_product(), source="<generated test part>"
        )

    # ---- 1.5 端角闭合优化(可选) ----
    if close_corners_r > 0.0:
        from corner_closer import close_corner_openings
        progress(f"闭合两端折弯角开口 (fillet_r={close_corners_r:.2f} mm)")
        fixed = close_corner_openings(product.shape, fillet_r=close_corners_r)
        if fixed is not None:
            product = product_model_from_shape(fixed, source=product.file_path)
            print(f"    端角闭合后产品体积 = {product.volume:.2f} mm³")

    p_x, p_y, p_z = (
        product.bbox[3] - product.bbox[0],
        product.bbox[4] - product.bbox[1],
        product.bbox[5] - product.bbox[2],
    )
    print(
        f"    产品尺寸 L×W×H = {p_x:.2f} × {p_y:.2f} × {p_z:.2f} mm, "
        f"体积 = {product.volume:.2f} mm³, 表面积 = {product.surface_area:.2f} mm²"
    )

    # ---- 2. 模芯生成 ----
    if core_size is None:
        core_size = compute_core_size(product.bbox, margin, base_thickness)
    progress(
        f"生成模芯: L×W×H = {core_size[0]:.2f} × {core_size[1]:.2f} × "
        f"{core_size[2]:.2f} mm (底部基座 {base_thickness:.1f} mm)"
    )
    core = generate_core_block(core_size=core_size, margin=margin,
                               product_bbox=product.bbox,
                               base_thickness=base_thickness)
    core_volume = shape_volume(core.solid)
    print(f"    模芯体积 = {core_volume:.2f} mm³")

    # ---- 3. 居中定位 ----
    progress(f"产品居中定位 (z_align_mode={z_align}, base_thickness={base_thickness:.1f} mm)")
    aligned = align_product_to_core(
        product.shape, core.size,
        z_align_mode=z_align, base_thickness=base_thickness,
    )
    print(f"    平移量 (dx, dy, dz) = {tuple(round(v, 3) for v in aligned.offset)}")
    print(
        f"    对齐后产品包围盒 Z ∈ "
        f"[{aligned.aligned_bbox[2]:.3f}, {aligned.aligned_bbox[5]:.3f}]"
    )

    # ---- 4. 型腔挖除 ----
    progress("挖除型腔 (Cut 模芯 - 产品)")
    cavity_core = cut_cavity(
        core.solid, aligned.aligned_shape,
        expected_core_volume=core_volume,
        expected_product_volume=product.volume,
    )
    cavity_volume = shape_volume(cavity_core)
    print(
        f"    型腔模芯体积 = {cavity_volume:.2f} mm³ "
        f"(理论 ≈ {core_volume - product.volume:.2f})"
    )

    # ---- 5. 分模 ----
    progress(f"分模 (parting_mode={parting_mode})")
    parting = split_mold(
        cavity_core, aligned.aligned_shape, core.size,
        parting_mode=parting_mode, z_parting=z_parting,
    )
    print(f"    分模高度 z_parting = {parting.parting_z:.3f} mm")
    print(
        f"    上模体积 = {parting.upper_volume:.2f} mm³, "
        f"下模体积 = {parting.lower_volume:.2f} mm³"
    )
    if parting.undercut_warnings:
        print(f"    [警告] 检测到 {len(parting.undercut_warnings)} 个倒扣候选区域：")
        for w in parting.undercut_warnings:
            print(
                f"      - 面#{w.face_index} pos={w.position} "
                f"area={w.area:.2f} mm² angle={w.angle_deg:.1f}° ({w.message})"
            )

    # ---- 6. 导出 ----
    progress("导出模芯装配体 / 型腔模芯 / 三件套装配体 / 上模 / 下模 STEP 与 JSON 元数据")
    core_assembly = build_core_assembly(aligned.aligned_shape, cavity_core)
    mold_assembly = build_mold_assembly(
        parting.upper_mold, aligned.aligned_shape, parting.lower_mold)
    metadata = {
        "product_file": product.file_path,
        "product_format": product.file_format,
        "product_size_mm": [p_x, p_y, p_z],
        "product_bbox": list(product.bbox),
        "product_volume_mm3": product.volume,
        "product_surface_area_mm2": product.surface_area,
        "core_size_mm": list(core.size),
        "core_volume_mm3": core_volume,
        "cavity_core_volume_mm3": cavity_volume,
        "core_assembly_volume_mm3": cavity_volume + product.volume,
        "base_thickness_mm": base_thickness,
        "align_offset_mm": list(aligned.offset),
        "z_align_mode": z_align,
        "parting_mode": parting_mode,
        "parting_z_mm": parting.parting_z,
        "upper_mold_volume_mm3": parting.upper_volume,
        "lower_mold_volume_mm3": parting.lower_volume,
        "undercut_warnings": [w.to_dict() for w in parting.undercut_warnings],
    }
    paths = export_molds(
        parting.upper_mold, parting.lower_mold,
        output_dir=output_dir, prefix=prefix, metadata=metadata,
        extra_shapes={"core_assembly": core_assembly, "cavity_core": cavity_core,
                      "mold_assembly": mold_assembly},
    )

    elapsed = time.perf_counter() - t0
    print(f"\n=== 模具生成完成，耗时 {elapsed:.2f}s ===")
    print(f"  模芯装配体(产品+型腔模芯): {paths['core_assembly']}")
    print(f"  三件套装配体(上模+产品+下模): {paths['mold_assembly']}")
    print(f"  型腔模芯(纯挖空): {paths['cavity_core']}")
    print(f"  上模: {paths['upper']}")
    print(f"  下模: {paths['lower']}")
    print(f"  元数据: {paths['metadata']}")

    return {
        "product": product,
        "core": core,
        "aligned": aligned,
        "cavity_core": cavity_core,
        "mold_assembly": mold_assembly,
        "parting": parting,
        "paths": paths,
    }


def _generate_pipe_mold(
    part_path: Optional[str] = None,
    core_size: Optional[Size3] = None,
    margin: Size3 = (20.0, 20.0, 20.0),
    output_dir: str = "./output",
    prefix: str = "mold",
    verify: bool = True,
    parting_surface: str = "plane",
    with_core: bool = True,
    run_parallel: bool = False,
    parting_z: Optional[float] = None,
    full_outputs: bool = False,
) -> dict:
    """管件硬模成型管线（外形包络型腔 + 分模面 + 芯棒 + 装配体）。

    与板件管线的差别（详见 pipe_mold.py 模块头）:
      * 产品**居中**放在长方体模芯中心（不含下基座偏移）；
      * 型腔 = 产品**外形包络**（管件内孔被填实），避免内孔里的"内芯条"把产品卡死；
      * 挖包络时被剔除的"内芯条"就是产品**内孔腔 = 芯棒**，直接白捡（不再多跑一次布尔）；
      * 分模面可选 **水平面**（默认）/ **侧影随形面**；水平面高度可**用户指定**
        （`parting_z`，不给则用自动推荐值），并回报上/下模各自的高度；
      * 额外导出：模芯装配体、三件套（上模+产品+下模）、**四件套（上模+产品+芯棒+下模）**、
        芯棒单件；
      * verify=True 时做开模/顶出干涉检验（较慢，约 6 次布尔求交）。
    """
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    from pipe_mold import build_pipe_mold

    if parting_surface not in ("plane", "silhouette"):
        raise MoldGenerationError(
            f"未知分模面形式 parting_surface={parting_surface!r}（plane / silhouette）",
            stage="main",
        )

    step = 0

    def progress(message: str) -> None:
        nonlocal step
        step += 1
        print(f"[{step}/6] {message}")

    t0 = time.perf_counter()

    # ---- 1. 产品读取 ----
    if part_path:
        progress(f"读取产品数模: {part_path}")
        product = read_product_model(part_path)
    else:
        progress("未指定产品文件，生成内置测试产品（带凸台长方体）")
        product = product_model_from_shape(
            make_test_product(), source="<generated test part>"
        )
    p_x, p_y, p_z = (
        product.bbox[3] - product.bbox[0],
        product.bbox[4] - product.bbox[1],
        product.bbox[5] - product.bbox[2],
    )
    print(
        f"    产品尺寸 L×W×H = {p_x:.2f} × {p_y:.2f} × {p_z:.2f} mm, "
        f"体积 = {product.volume:.2f} mm³"
    )

    # ---- 2. 模芯（产品居中：四周余量相同）----
    if core_size is None:
        core_size = compute_core_size(product.bbox, margin, base_thickness=margin[2])
    progress(
        f"生成模芯(产品居中): L×W×H = {core_size[0]:.2f} × {core_size[1]:.2f} × "
        f"{core_size[2]:.2f} mm"
    )
    core = generate_core_block(core_size=core_size, margin=margin,
                               product_bbox=product.bbox,
                               base_thickness=margin[2])
    core_volume = shape_volume(core.solid)
    print(f"    模芯体积 = {core_volume:.2f} mm³")

    # ---- 3. 居中定位 ----
    progress("产品居中定位 (z_align_mode=center)")
    aligned = align_product_to_core(
        product.shape, core.size, z_align_mode="center", base_thickness=0.0,
    )
    print(f"    平移量 (dx, dy, dz) = {tuple(round(v, 3) for v in aligned.offset)}")
    print(
        f"    对齐后产品包围盒 Z ∈ "
        f"[{aligned.aligned_bbox[2]:.3f}, {aligned.aligned_bbox[5]:.3f}]"
    )

    # ---- 4~5. 外形包络型腔 + 分模（pipe_mold 一次完成并自校验）----
    progress("挖外形包络型腔 + 分模（%s）+ 开模/顶出校验"
             % ("水平分模面" if parting_surface == "plane" else "侧影随形分模面"))
    res = build_pipe_mold(
        core.solid, aligned.aligned_shape, core.size, verify=verify,
        parting=parting_surface, run_parallel=run_parallel, keep_core=with_core,
        plane_z=parting_z,
    )
    print(f"    上模体积 = {res.upper_volume:.2f} mm³, "
          f"下模体积 = {res.lower_volume:.2f} mm³")
    print(f"    型腔模体积 = {res.die_volume:.2f} mm³ "
          f"(模芯-型腔-产品 = 被填实的孔腔 {res.envelope_extra_volume:.1f} mm³)")
    if res.warnings:
        for w in res.warnings:
            print(f"    [警告] {w}")

    # ---- 6. 导出 ----
    # 注意：内部计算都在"模芯坐标系"（产品被平移居中）。**导出时整体平移回输入数模的
    # 原始坐标系**，这样用户把上模/下模/芯棒和原产品放在一起时位置是**对得上的**
    # （之前导出在模芯坐标系里，用户拿原产品一比就以为"管件和上下模重叠"）。
    progress("导出型腔模 / 上下模 STEP 与 JSON 元数据")
    core_pin = res.core_pin if with_core else None
    back = tuple(-float(v) for v in aligned.offset)
    prod_ex = _shift_shape(aligned.aligned_shape, back)
    upper_ex = _shift_shape(res.upper_mold, back)
    lower_ex = _shift_shape(res.lower_mold, back)
    die_ex = _shift_shape(res.die_full, back)
    core_ex = _shift_shape(core_pin, back) if core_pin is not None else None

    builder = BRep_Builder()
    assembly = TopoDS_Compound()
    builder.MakeCompound(assembly)
    builder.Add(assembly, prod_ex)
    builder.Add(assembly, die_ex)
    mold_assembly = build_mold_assembly(upper_ex, prod_ex, lower_ex)
    mold4_assembly = None
    core_pin_size = None
    if core_ex is not None:
        mold4_assembly = build_mold4_assembly(upper_ex, prod_ex, core_ex, lower_ex)
        cb = shape_bbox(core_ex)
        core_pin_size = [cb[i + 3] - cb[i] for i in range(3)]
        print("    四件套装配体: 上模 + 产品 + 芯棒 + 下模（芯棒 = 挖包络时剔除的内芯条，"
              "与上下模天然互补、互不重叠）")
        print("    芯棒(内孔腔)体积 = %.1f mm³，尺寸 = %.1f × %.1f × %.1f mm"
              % (res.core_volume, core_pin_size[0], core_pin_size[1], core_pin_size[2]))
    metadata = {
        "mold_type": "pipe",
        "product_file": product.file_path,
        "product_format": product.file_format,
        "product_size_mm": [p_x, p_y, p_z],
        "product_bbox": list(product.bbox),
        "product_volume_mm3": product.volume,
        "core_size_mm": list(core.size),
        "core_volume_mm3": core_volume,
        "align_offset_mm": list(aligned.offset),
        "z_align_mode": "center",
        "parting_mode": "plane_pipe" if res.parting_kind == "plane" else "silhouette_pipe",
        "parting_kind": res.parting_kind,
        "parting_axis": "XY"[res.axis],
        "parting_profile": [[round(x, 3), round(h, 3)] for x, h in res.profile],
        "parting_z_mm": res.parting_z_mean,
        "parting_z_source": res.stats.get("plane", {}).get("source", "auto"),
        "parting_z_auto_mm": res.stats.get("plane", {}).get("z_auto"),
        "parting_z_safe_range_mm": [res.stats.get("plane", {}).get("safe_lo"),
                                    res.stats.get("plane", {}).get("safe_hi")],
        "upper_mold_height_mm": res.stats.get("heights", {}).get("upper_height_mm"),
        "lower_mold_height_mm": res.stats.get("heights", {}).get("lower_height_mm"),
        "die_volume_mm3": res.die_volume,
        "envelope_extra_volume_mm3": res.envelope_extra_volume,
        "upper_mold_volume_mm3": res.upper_volume,
        "lower_mold_volume_mm3": res.lower_volume,
        "core_pin_volume_mm3": res.core_volume,
        "core_pin_size_mm": core_pin_size,
        "assembly_parts": (["upper_mold", "product", "core_pin", "lower_mold"]
                           if core_pin is not None else ["upper_mold", "product", "lower_mold"]),
        "demold_check": {k: round(v, 4) for k, v in res.demold.items()},
        "mold_warnings": list(res.warnings),
        "pipe_stats": res.stats,
    }
    # 默认只导出"要用的那几件"：上模 / 下模 / 芯棒 / 四件套 + 元数据
    # （型腔模、模芯装配体、三件套各是一份大 STEP，写盘很花时间；需要时用 full_outputs=True）
    extra: Dict[str, object] = {}
    if core_ex is not None:
        extra["mold4_assembly"] = mold4_assembly
        extra["core_pin"] = core_ex
    if full_outputs:
        extra.update({"die_full": die_ex, "core_assembly": assembly,
                      "mold_assembly": mold_assembly})
    paths = export_molds(
        upper_ex, lower_ex,
        output_dir=output_dir, prefix=prefix, metadata=metadata,
        extra_shapes=extra,
    )

    elapsed = time.perf_counter() - t0
    print(f"\n=== 管件模具生成完成，耗时 {elapsed:.2f}s ===")
    print(f"  上模: {paths['upper']}")
    print(f"  下模: {paths['lower']}")
    if core_pin is not None:
        print(f"  芯棒(内孔腔): {paths['core_pin']}")
        print(f"  ★ 四件套装配体(上模+产品+芯棒+下模): {paths['mold4_assembly']}")
    if full_outputs:
        print(f"  型腔模(外形包络): {paths['die_full']}")
        print(f"  模芯装配体(产品+型腔模): {paths['core_assembly']}")
        print(f"  三件套装配体(上模+产品+下模): {paths['mold_assembly']}")
    print(f"  元数据: {paths['metadata']}")
    print("  （上模/下模/芯棒/装配体都已在**输入数模的原始坐标系**里，可直接和原产品装配）")

    return {
        "product": product,
        "core": core,
        "aligned": aligned,
        "pipe": res,
        "cavity_core": res.die_full,
        "mold_assembly": mold_assembly,
        "mold4_assembly": mold4_assembly,
        "core_pin": core_ex,
        "parting": res,
        "paths": paths,
    }


def main(argv=None) -> int:
    """命令行入口。"""
    ensure_utf8_stdout()
    parser = argparse.ArgumentParser(
        description="模具自动生成系统 —— 产品数模 -> 上模/下模 STEP",
    )
    parser.add_argument("--part", type=str, default=None,
                        help="产品数模路径 (.stp/.step/.iges/.igs)，缺省使用内置测试产品")
    parser.add_argument("--core", type=float, nargs=3, metavar=("L", "W", "H"),
                        default=None, help="模芯尺寸 (L W H)")
    parser.add_argument("--margin", type=float, nargs=3, metavar=("MX", "MY", "MZ"),
                        default=(20.0, 20.0, 20.0), help="自动计算模芯时的余量")
    parser.add_argument("--base-thickness", type=float, default=20.0,
                        help="底部基座厚度 (mm)，型腔底板实体厚度")
    parser.add_argument("--parting-mode", choices=("cavity_bottom", "max_z", "mid", "custom", "contour", "silhouette"),
                        default="contour", help="分模模式（cavity_bottom = 以型腔下表面为分模面；contour / silhouette = 沿产品投影轮廓分模，silhouette 自动取投影面积最大处为分模面）")
    parser.add_argument("--parting-z", type=float, default=None,
                        help="custom 模式的分模高度")
    parser.add_argument("--z-align", choices=("bottom", "center"), default="bottom",
                        help="产品 Z 向定位方式")
    parser.add_argument("--output", type=str, default="./output", help="输出目录")
    parser.add_argument("--prefix", type=str, default="mold", help="输出文件名前缀")
    parser.add_argument("--close-corners", type=float, default=0.0,
                        help="闭合两端折弯角开口的圆角半径 (mm)；>0 时启用（例：--close-corners 3.0）")
    parser.add_argument("--mold-type", choices=("sheet", "pipe"), default="sheet",
                        help="模具工艺类型: sheet=板件折弯/液压成型（默认）, "
                             "pipe=管件硬模成型（产品居中 + 外形包络型腔 + 侧影分模面）")
    parser.add_argument("--parting-surface", choices=("plane", "silhouette"),
                        default="plane",
                        help="管件模式分模面形式: plane=水平面（默认，最好加工）, "
                             "silhouette=沿产品侧影随形")
    parser.add_argument("--no-verify", action="store_true",
                        help="管件模式：跳过开模/顶出干涉检验（省 ~5 min，只出几何与装配体）")
    parser.add_argument("--no-core", action="store_true",
                        help="管件模式：不导出四件套（上模+产品+芯棒+下模）与芯棒单件")
    parser.add_argument("--parallel", action="store_true",
                        help="布尔运算开 OCC 并行（多核可能提速；若结果异常请去掉本参数）")
    parser.add_argument("--mode", choices=("mold", "core"), default="mold",
                        help="任务类型: mold=生成上下模（默认）, "
                             "core=模芯设计（沿产品两端端口面把内孔填满，生成芯棒）")
    parser.add_argument("--core-clearance", type=float, default=0.0,
                        help="模芯设计: 芯棒单边装配间隙 (mm)，0=与内孔完全贴合")
    parser.add_argument("--core-plate-depth", type=float, default=1.0,
                        help="模芯设计: 端口封盖拉伸长度 (mm)")
    parser.add_argument("--core-quick", action="store_true",
                        help="模芯设计: 抽芯检验只做近距离（更快）")
    parser.add_argument("--core-axis", choices=("auto", "x", "y"), default="auto",
                        help="模芯设计: 内孔长轴方向，默认自动")
    args = parser.parse_args(argv)

    try:
        if args.mode == "core":
            from core_design import design_core
            design_core(
                part_path=args.part,
                output_dir=args.output,
                prefix=args.prefix if args.prefix != "mold" else "core",
                axis=None if args.core_axis == "auto" else "xy".index(args.core_axis),
                clearance=args.core_clearance,
                plate_depth=args.core_plate_depth,
                quick=args.core_quick,
            )
            return 0
        generate_mold(
            part_path=args.part,
            core_size=tuple(args.core) if args.core else None,
            margin=tuple(args.margin),
            base_thickness=args.base_thickness,
            parting_mode=args.parting_mode,
            z_parting=args.parting_z,
            z_align=args.z_align,
            output_dir=args.output,
            prefix=args.prefix,
            close_corners_r=args.close_corners,
            mold_type=args.mold_type,
            parting_surface=args.parting_surface,
            verify=not args.no_verify,
            with_core=not args.no_core,
            run_parallel=args.parallel,
        )
    except MoldGenerationError as exc:
        print(f"\n[错误] {exc}", file=sys.stderr)
        return 1
    except Exception as exc:  # noqa: BLE001 —— 兜底捕获未预期异常
        print(f"\n[未预期错误] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    # 测试示例：无参数运行即使用内置测试产品完整跑通流程
    raise SystemExit(main())

