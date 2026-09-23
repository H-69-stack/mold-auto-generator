"""
分模器 (parting_splitter.py) —— 模块 5（核心模块）

作者: 模具自动生成系统开发组
用途: 将带型腔的模芯按开模方向（默认 +Z）分割为上模与下模，并检测倒扣区域。

分模模式 (parting_mode):
  - "cavity_bottom": 分模面 = 型腔下表面（产品底面轮廓，可为自由曲面）。
       型腔下表面以上全部为上模，型腔下表面以下（基座）为下模 —— 默认推荐
  - "max_z": 分模面 = 产品最高点 z_max（型腔全在下模，上模为平板）
  - "mid":   分模面 = 产品高度中点 (z_min + z_max) / 2
  - "custom": 分模面 = 用户指定 z_parting
  - "contour": 分模面 = 产品在分模高度处的投影轮廓（轮廓内为上模，轮廓外为下模）

分割算法（稳健版，等效于用 z = z_parting 的无限平面分割）:
  1. 构造记录用分模面 gp_Pln(z = z_parting)；
  2. 用两个“半空间”长方体切割工具（远大于模芯，保证完全贯穿）切出上下两半；
     工具面刻意避开分模面 z_parting（上下各留 eps 重叠余量），
     避免切割工具与产品在分模面处的平面重合导致布尔运算退化；
  3. 再用两个薄板工具把重叠层精确修掉，使上模底面 / 下模顶面
     严格落在 z = z_parting。

倒扣检测 (undercut, 可选，仅警告不阻塞):
  遍历产品所有面，计算面法向与开模方向的夹角；
  若夹角 > 90°（法向朝下）且面中心 Z < z_parting，标记为倒扣候选区域。

后续可扩展：基于 z = z_parting 平面截产品得到轮廓、将轮廓沿法向延伸至
模芯侧壁生成覆盖模芯横截面的分模曲面（进阶分模面构建策略）。
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Transform,
)
from OCP.BRepGProp import BRepGProp
from OCP.BRepLProp import BRepLProp_SLProps
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Dir, gp_Pln, gp_Pnt, gp_Trsf, gp_Vec

from cavity_cutter import DEFAULT_FUZZY_VALUES, bool_common, bool_cut, bool_fuse
from errors import PartingError
from geometry_utils import BBox, shape_bbox, shape_volume

Size3 = Tuple[float, float, float]


@dataclass
class UndercutWarning:
    """倒扣候选区域警告（仅提示，不阻塞流程）。"""

    face_index: int
    position: Tuple[float, float, float]   # 面中心 (x, y, z)
    area: float                            # 面面积 mm²
    normal: Tuple[float, float, float]     # 面法向 (nx, ny, nz)
    angle_deg: float                       # 法向与 +Z 夹角
    is_horizontal: bool                    # 是否近似水平（法向近 ±Z）
    message: str

    def to_dict(self) -> Dict[str, object]:
        """转换为 JSON 可序列化字典。"""
        return {
            "face_index": self.face_index,
            "position": list(self.position),
            "area": round(self.area, 3),
            "normal": list(self.normal),
            "angle_deg": self.angle_deg,
            "is_horizontal": self.is_horizontal,
            "message": self.message,
        }


@dataclass
class PartingResult:
    """分模结果。"""

    upper_mold: object                 # 上模实体 TopoDS_Shape
    lower_mold: object                 # 下模实体 TopoDS_Shape
    parting_z: float                   # 分模高度 z_parting
    parting_plane: object              # 记录用 gp_Pln(z = z_parting)
    upper_volume: float                # 上模体积 mm³
    lower_volume: float                # 下模体积 mm³
    undercut_warnings: List[UndercutWarning]


def determine_parting_z(
    product_bbox: BBox,
    core_height: float,
    parting_mode: str = "max_z",
    z_parting: Optional[float] = None,
) -> float:
    """确定分模高度 z_parting。

    三种模式:
      "max_z"  —— 产品最高点 z_max；
      "mid"    —— 产品高度中点 (z_min + z_max) / 2；
      "custom" —— 用户指定 z_parting。

    校验: z_parting ∈ [0, core_height]，且上模/下模均保留非零厚度。

    异常:
        PartingError: 模式非法、参数缺失或高度越界。
    """
    p_zmin, p_zmax = product_bbox[2], product_bbox[5]
    if parting_mode == "max_z":
        z = p_zmax
    elif parting_mode == "mid":
        z = 0.5 * (p_zmin + p_zmax)
    elif parting_mode == "custom":
        if z_parting is None:
            raise PartingError("parting_mode='custom' 时必须提供 z_parting")
        z = float(z_parting)
    else:
        raise PartingError(
            f"未知分模模式: {parting_mode!r}（可选 max_z / mid / custom）"
        )

    if z < 0.0 or z > core_height:
        raise PartingError(f"分模高度 z={z:.3f} 超出范围 [0, {core_height:.3f}]")
    if z < 1e-6 or z > core_height - 1e-6:
        raise PartingError(
            f"分模高度 z={z:.3f} 将导致上模或下模厚度为零，请调整分模模式/高度"
        )
    return z


def split_mold(
    core_with_cavity,
    product_shape,
    core_size: Size3,
    parting_mode: str = "max_z",
    z_parting: Optional[float] = None,
    z_axis=None,
    split_eps: Optional[float] = None,
    fuzzy_values=DEFAULT_FUZZY_VALUES,
) -> PartingResult:
    """将带型腔的模芯分割为上模与下模。

    参数:
        core_with_cavity: 模块 4 输出的带型腔模芯 (TopoDS_Shape)。
        product_shape: 居中后的产品实体（用于分模高度计算与倒扣检测）。
        core_size: 模芯尺寸 (L, W, H)。
        parting_mode: "max_z" / "mid" / "custom"。
        z_parting: custom 模式下的分模高度。
        z_axis: 开模方向，默认 +Z (gp_Dir(0, 0, 1))。
        split_eps: 分模面附近的防退化重叠余量（mm，默认按模芯尺寸自适应）。
        fuzzy_values: 布尔运算容差重试序列。

    返回:
        PartingResult。

    异常:
        PartingError: 分模高度非法或分模后体积异常。
    """
    if parting_mode in ("contour", "silhouette"):
        # 沿产品投影轮廓分模（轮廓内保留为上模，轮廓外切开为下模）
        # silhouette 与 contour 同实现；轮廓/侧影模式下默认分模高度是
        # 产品投影面积最大处（自动），而非固定最高点。
        return split_mold_by_contour(
            core_with_cavity, product_shape, core_size,
            z_parting=z_parting, z_axis=z_axis,
            fuzzy_values=fuzzy_values,
        )

    if parting_mode == "cavity_bottom":
        # 以型腔下表面（=产品底面轮廓）为分模面：型腔下表面以上全部为上模，
        # 型腔下表面以下（基座）为下模
        return split_mold_by_cavity_bottom(
            core_with_cavity, product_shape, core_size,
            z_axis=z_axis, fuzzy_values=fuzzy_values,
        )

    if z_axis is None:
        z_axis = gp_Dir(0.0, 0.0, 1.0)

    L, W, H = core_size
    product_bbox = shape_bbox(product_shape)
    z_part = determine_parting_z(product_bbox, H, parting_mode, z_parting)
    parting_plane = gp_Pln(gp_Pnt(0.0, 0.0, z_part), gp_Dir(0.0, 0.0, 1.0))

    core_bbox = shape_bbox(core_with_cavity)
    cx_min, cy_min, cz_min = core_bbox[0], core_bbox[1], core_bbox[2]
    cx_max, cy_max, cz_max = core_bbox[3], core_bbox[4], core_bbox[5]

    max_dim = max(L, W, H)
    inflate = max_dim * 2.0                     # 切割工具足够大，完全贯穿模芯
    if split_eps is None:
        split_eps = min(1.0, max(0.1, 0.005 * max_dim))
    eps = float(split_eps)

    print(f"  - 分模高度 z_parting = {z_part:.3f} mm, 防退化余量 eps = {eps:.3f} mm")

    # ---- 1) 粗切：工具避开分模面，上下各留 eps 重叠余量 ----
    lower_tool = BRepPrimAPI_MakeBox(
        gp_Pnt(cx_min - inflate, cy_min - inflate, cz_min - inflate),
        gp_Pnt(cx_max + inflate, cy_max + inflate, z_part - eps),
    ).Shape()
    upper_tool = BRepPrimAPI_MakeBox(
        gp_Pnt(cx_min - inflate, cy_min - inflate, z_part + eps),
        gp_Pnt(cx_max + inflate, cy_max + inflate, cz_max + inflate),
    ).Shape()

    print("  - 切割上模 (Cut 带型腔模芯, 下半空间长方体) ...")
    upper_mold = bool_cut(core_with_cavity, lower_tool, fuzzy_values, label="上模切割")
    print("  - 切割下模 (Cut 带型腔模芯, 上半空间长方体) ...")
    lower_mold = bool_cut(core_with_cavity, upper_tool, fuzzy_values, label="下模切割")

    # ---- 2) 精修：把重叠层修掉，使分模面精确落在 z = z_part ----
    if eps > 1e-9:
        trim_upper_tool = BRepPrimAPI_MakeBox(
            gp_Pnt(cx_min - inflate, cy_min - inflate, z_part - eps),
            gp_Pnt(cx_max + inflate, cy_max + inflate, z_part),
        ).Shape()
        trim_lower_tool = BRepPrimAPI_MakeBox(
            gp_Pnt(cx_min - inflate, cy_min - inflate, z_part),
            gp_Pnt(cx_max + inflate, cy_max + inflate, z_part + eps),
        ).Shape()
        print("  - 修整上模底面 / 下模顶面至精确分模面 ...")
        upper_mold = bool_cut(upper_mold, trim_upper_tool, fuzzy_values, label="上模精修")
        lower_mold = bool_cut(lower_mold, trim_lower_tool, fuzzy_values, label="下模精修")

    upper_vol = shape_volume(upper_mold)
    lower_vol = shape_volume(lower_mold)
    if upper_vol <= 0.0 or lower_vol <= 0.0:
        raise PartingError(
            f"分模后上模/下模体积异常 (上模={upper_vol:.3f}, 下模={lower_vol:.3f})，"
            "请检查分模高度与产品位置"
        )

    # ---- 3) 倒扣检测（仅警告，不阻塞）----
    warnings = detect_undercuts(product_shape, z_part, z_axis)

    return PartingResult(
        upper_mold=upper_mold,
        lower_mold=lower_mold,
        parting_z=z_part,
        parting_plane=parting_plane,
        upper_volume=upper_vol,
        lower_volume=lower_vol,
        undercut_warnings=warnings,
    )


def _edge_endpoints(edge):
    """返回边端点 ((x1,y1,z1),(x2,y2,z2))。"""
    vf = TopoDS.Vertex_s(TopExp.FirstVertex_s(edge))
    vl = TopoDS.Vertex_s(TopExp.LastVertex_s(edge))
    p1 = BRep_Tool.Pnt_s(vf)
    p2 = BRep_Tool.Pnt_s(vl)
    return (p1.X(), p1.Y(), p1.Z()), (p2.X(), p2.Y(), p2.Z())


def _pt_dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _assemble_wires(edges, tol: float = 1e-5):
    """贪心端点匹配，把截面边组装成闭合 wire。

    截面边的顶点可能不完全重合（容差问题），且顺序随机；
    BRepBuilderAPI_MakeWire / ShapeAnalysis_FreeBounds 在这种
    情况下不可靠，这里按端点距离贪心连接，返回 TopoDS_Wire 列表。
    """
    eps = [_edge_endpoints(e) for e in edges]
    used = [False] * len(edges)
    wires = []
    for i in range(len(edges)):
        if used[i]:
            continue
        used[i] = True
        chain = [i]
        head = eps[i][0]
        tail = eps[i][1]
        closed = _pt_dist(head, tail) <= tol
        progressed = True
        while not closed and progressed:
            progressed = False
            for j in range(len(edges)):
                if used[j]:
                    continue
                s, e_ = eps[j]
                if _pt_dist(s, tail) <= tol:
                    chain.append(j)
                    used[j] = True
                    tail = e_
                    progressed = True
                elif _pt_dist(e_, tail) <= tol:
                    chain.append(j)
                    used[j] = True
                    tail = s
                    progressed = True
                if progressed:
                    break
            closed = _pt_dist(head, tail) <= tol
        if len(chain) >= 1 and closed:
            mw = BRepBuilderAPI_MakeWire()
            for idx in chain:
                try:
                    mw.Add(edges[idx])
                except Exception:  # noqa: BLE001
                    pass
            if mw.IsDone() and not mw.Shape().IsNull():
                wires.append(TopoDS.Wire_s(mw.Shape()))
    return wires


def _section_contour_wire(product_shape, z: float, min_area: float = 1.0):
    """求产品在平面 z 处的截面轮廓线（外轮廓闭合 wire）。

    返回 TopoDS_Wire 或 None（截面为空 / 退化 / 面积过小）。
    截面可能包含多个环（外轮廓 + 内孔），取面积最大的外环。
    """
    plane = gp_Pln(gp_Pnt(0.0, 0.0, z), gp_Dir(0.0, 0.0, 1.0))
    sec = BRepAlgoAPI_Section(product_shape, plane)
    sec.ComputePCurveOn1(True)
    sec.Approximation(True)
    sec.Build()
    if not sec.IsDone():
        return None

    edges = []
    exp = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while exp.More():
        edges.append(TopoDS.Edge_s(exp.Current()))
        exp.Next()
    if not edges:
        return None

    best_wire = None
    best_area = -1.0
    for w in _assemble_wires(edges):
        try:
            face = BRepBuilderAPI_MakeFace(w).Shape()
            gprops = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, gprops)
            area = gprops.Mass()
            if area > best_area:
                best_area, best_wire = area, w
        except Exception:  # noqa: BLE001
            continue
    if best_area < min_area:
        return None
    return best_wire


def _contour_column(wire, z_base: float, height: float):
    """把分模轮廓 wire 平移到 z=z_base 平面，生成面并沿 +Z 拉伸成柱体。

    关键: 必须先平移轮廓线到目标平面再建面，否则投影建面会产生畸形柱体。
    """
    bb = shape_bbox(wire)
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(0.0, 0.0, z_base - 0.5 * (bb[2] + bb[5])))
    moved = TopoDS.Wire_s(
        BRepBuilderAPI_Transform(wire, trsf, True, False).Shape()
    )
    face = BRepBuilderAPI_MakeFace(
        gp_Pln(gp_Pnt(0.0, 0.0, z_base), gp_Dir(0.0, 0.0, 1.0)), moved
    ).Shape()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0.0, 0.0, height)).Shape()


def _section_points_2d(product_shape, z: float, npts: int = 24):
    """求平面 z 处截面所有边的离散点（2D，取 XY）。"""
    from OCP.BRepAdaptor import BRepAdaptor_Curve

    plane = gp_Pln(gp_Pnt(0.0, 0.0, z), gp_Dir(0.0, 0.0, 1.0))
    sec = BRepAlgoAPI_Section(product_shape, plane)
    sec.Approximation(False)
    sec.Build()
    if not sec.IsDone():
        return []
    pts = []
    exp = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while exp.More():
        edge = TopoDS.Edge_s(exp.Current())
        exp.Next()
        try:
            c = BRepAdaptor_Curve(edge)
            u0, u1 = c.FirstParameter(), c.LastParameter()
            if not (math.isfinite(u0) and math.isfinite(u1)) or u1 <= u0:
                continue
            for u in (u0 + (u1 - u0) * i / (npts - 1) for i in range(npts)):
                p = c.Value(u)
                pts.append((p.X(), p.Y()))
        except Exception:  # noqa: BLE001
            continue
    return pts


def _hull_area_2d(pts) -> float:
    """2D 点云凸包面积（即该高度的"投影轮廓面积"）。"""
    if len(pts) < 3:
        return 0.0
    ps = sorted(set(pts))
    if len(ps) < 3:
        return 0.0

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in ps:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(ps):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        return 0.0
    area = 0.0
    for i in range(len(hull)):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % len(hull)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def find_max_projection_z(
    product_shape,
    z_min: Optional[float] = None,
    z_max: Optional[float] = None,
    nsteps: int = 24,
    min_area: float = 1.0,
    z_axis=None,
):
    """扫描开模方向，返回产品**投影轮廓面积**最大处的分模高度与该层外轮廓。

    对薄板冲压 / 折弯成型件，最大投影截面即"侧影最外"处：产品在此层最宽，
    往上下均收缩。把分模面放这里，上模与下模都能从该处顺利分离（拔模不卡），
    这是冲压开模的标准位置；而"在最高点一刀切"会让上模变成只盖住窄凸起的空盖。

    注意（重要修正）: 判据用截面点云的**凸包面积**（= 投影轮廓面积），
    而不是截面"材料面积"。薄壁中空件（管件/盒形件）的材料面积在**切线高度**
    （平面与曲面相切处）反而最大，旧写法会把分模面选到产品最顶端/最底端的
    切点高度上，导致上下模严重失衡、甚至产生倒扣。

    返回:
        (z_parting, wire) —— 分模面高度（mm）与该层截面外轮廓 wire；
        若产品无法取到有效截面，返回 (None, None)。
    """
    if z_axis is None:
        z_axis = gp_Dir(0.0, 0.0, 1.0)
    bb = shape_bbox(product_shape)
    if z_min is None:
        z_min = bb[2]
    if z_max is None:
        z_max = bb[5]
    if z_max - z_min < 1e-9:
        return z_min, None

    samples = []          # (z, area, wire)
    for i in range(nsteps + 1):
        z = z_min + (z_max - z_min) * (i / nsteps)
        w = _section_contour_wire(product_shape, z, min_area=min_area)
        if w is None:
            continue
        area = _hull_area_2d(_section_points_2d(product_shape, z))
        if area <= 0.0:
            try:
                face = BRepBuilderAPI_MakeFace(w).Shape()
                gprops = GProp_GProps()
                BRepGProp.SurfaceProperties_s(face, gprops)
                area = gprops.Mass()
            except Exception:  # noqa: BLE001 —— 单层失败跳过
                continue
        samples.append((z, area, w))
    if not samples:
        return None, None
    best_area = max(a for _, a, _ in samples)
    # 面积最大往往是一段平台（等宽段）；取平台中位高度，避免把分模面
    # 贴到产品最底/最顶的切点高度上。
    plateau = [(z, w) for z, a, w in samples if a >= 0.98 * best_area]
    plateau.sort(key=lambda t: t[0])
    z_best, w_best = plateau[len(plateau) // 2]
    return z_best, w_best


def split_mold_by_contour(
    core_with_cavity,
    product_shape,
    core_size: Size3,
    z_parting: Optional[float] = None,
    contour_drop: float = 0.5,
    max_drop_steps: int = 5,
    z_axis=None,
    fuzzy_values=DEFAULT_FUZZY_VALUES,
) -> PartingResult:
    """沿产品在分模面的投影轮廓分模（保留轮廓内，切开轮廓外）。

    与"一刀切"（无限平面分割）不同，分模线沿产品在分模高度处的
    截面轮廓（分模线）：

      1. 分模高度 z_parting = 产品最高点 z_max（max_z），可自定义；
      2. 求产品在 z_parting 处的截面轮廓线；若产品最高点为点/退化，
         自动下移 contour_drop 直到获得有效轮廓；
      3. 把轮廓线平移到分模面 z=z_parting，拉伸成"轮廓柱体"
         （覆盖 z∈[z_parting, 模芯顶]）；
      4. 上模 = Common(型腔模芯, 轮廓柱体) —— 保留轮廓之内的部分
         （产品正上方的"盖"，下表面贴合产品顶面）；
      5. 下模 = Cut(型腔模芯, 轮廓柱体) —— 轮廓之外的部分切开，
         与上模分离。

    返回:
        PartingResult。
    """
    if z_axis is None:
        z_axis = gp_Dir(0.0, 0.0, 1.0)

    product_bbox = shape_bbox(product_shape)
    p_zmax = product_bbox[5]
    if z_parting is None:
        # 自动选分模面：产品投影轮廓面积最大处（侧影最外，冲压分模标准位置）。
        # 产品必须在分层扫描前已完成 Z 向对齐（模型位于其真实 bbox 范围内）。
        auto_z, _auto_wire = find_max_projection_z(product_shape)
        if auto_z is not None:
            z_parting = float(auto_z)
            print(f"  - 自动分模高度 = 产品投影最大截面 z={z_parting:.3f} mm")
        else:
            z_parting = p_zmax

    L, W, H = core_size
    if z_parting <= 0.0 or z_parting >= H:
        raise PartingError(
            f"分模高度 z={z_parting:.3f} 超出范围 (0, {H:.3f})"
        )

    # ---- 求分模线轮廓（必要时下移）----
    wire = None
    section_z = None
    for k in range(max_drop_steps + 1):
        zz = z_parting - k * contour_drop
        w = _section_contour_wire(product_shape, zz)
        if w is not None:
            wire, section_z = w, zz
            break
    if wire is None:
        raise PartingError(
            f"无法获得产品在分模高度附近的截面轮廓（z≈{z_parting:.2f}），"
            "请检查产品数模或调整分模高度"
        )

    core_bbox = shape_bbox(core_with_cavity)
    core_top = core_bbox[5]
    print(f"  - 分模线轮廓: z={section_z:.3f} (分模面 z_parting={z_parting:.3f})")

    # ---- 轮廓柱体 ----
    col = _contour_column(wire, z_parting, core_top - z_parting + 0.01)

    # ---- 上模：保留轮廓之内 ----
    print("  - 沿轮廓提取上模 (Common 型腔模芯, 轮廓柱体) ...")
    upper_mold = bool_common(core_with_cavity, col, fuzzy_values, label="轮廓上模")
    print("  - 切开下模 (Cut 型腔模芯, 轮廓柱体) ...")
    lower_mold = bool_cut(core_with_cavity, col, fuzzy_values, label="轮廓下模")

    upper_vol = shape_volume(upper_mold)
    lower_vol = shape_volume(lower_mold)
    if upper_vol <= 0.0 or lower_vol <= 0.0:
        raise PartingError(
            f"沿轮廓分模后上模/下模体积异常 (上模={upper_vol:.3f}, "
            f"下模={lower_vol:.3f})"
        )

    warnings = detect_undercuts(product_shape, z_parting, z_axis)
    return PartingResult(
        upper_mold=upper_mold,
        lower_mold=lower_mold,
        parting_z=z_parting,
        parting_plane=gp_Pln(gp_Pnt(0.0, 0.0, z_parting), gp_Dir(0.0, 0.0, 1.0)),
        upper_volume=upper_vol,
        lower_volume=lower_vol,
        undercut_warnings=warnings,
    )


def split_mold_by_cavity_bottom(
    core_with_cavity,
    product_shape,
    core_size: Size3,
    z_axis=None,
    fuzzy_values=DEFAULT_FUZZY_VALUES,
    split_z: Optional[float] = None,
    normal_threshold: float = 0.3,
    sweep_eps: Optional[float] = None,
) -> PartingResult:
    """以型腔下表面（产品下表面）为分模面做随形分模。

    关键原理：type = 模芯 - 产品，type 的边界就是产品表面。所以在 product
    投影范围内，type ∩ (z≤split_z) 的顶面**自动**就是产品下表面（被产品本体
    挡住，type 穿不进去），即随形；在投影外，顶面才是 split_z 水平面（分模台面）。
    因此"在 split_z 处平面切割 + 把高于 split_z 的下表面区域补进下模"就
    实现了用户要求的"分模面沿弯曲下表面走，到型腔最高点后水平外切"。

    实现：
      1. split_z = 产品投影最大截面高度（分模线高度），可显式指定；
      2. 基准 = 平面切割 custom z_parting=split_z（已含内随形 + 外水平台面）；
      3. 遍历 z_max > split_z 的朝下面，向下拉伸棱柱把该区域补进下模
         （下表面能"爬到最高点"才水平外切）。

    参数:
        split_z: 分模台面高度（水平外切面）；默认取产品投影面积最大处。
        normal_threshold / sweep_eps: 保留兼容。
    """
    if z_axis is None:
        z_axis = gp_Dir(0.0, 0.0, 1.0)

    product_bbox = shape_bbox(product_shape)
    core_bbox = shape_bbox(core_with_cavity)
    core_zmin = core_bbox[2]
    core_zmax = core_bbox[5]
    p_zmin = product_bbox[2]

    if core_zmax - p_zmin < 1e-6:
        raise PartingError(
            f"型腔下表面 z={p_zmin:.3f} 与模芯顶面 z={core_zmax:.3f} 过近，上模厚度不足"
        )

    # ---- 分模台面高度 split_z ----
    auto_wire = None
    if split_z is None:
        auto_z, auto_wire = find_max_projection_z(product_shape)
        split_z = float(auto_z) if auto_z is not None else p_zmin
    split_z = max(core_zmin + 1e-3, min(core_zmax - 1e-3, float(split_z)))
    print(f"  - 随形分模: 分模台面 split_z = {split_z:.3f} mm")

    # ---- 1) 基准：平面切割 custom z=split_z ----
    print("  - 随形分模: 基准平面切割 (split_z) ...")
    base = split_mold(
        core_with_cavity, product_shape, core_size,
        parting_mode="custom", z_parting=split_z,
        z_axis=z_axis, fuzzy_values=fuzzy_values,
    )
    lower = base.lower_mold
    type_vol = shape_volume(core_with_cavity)

    # ---- 1.5) 外缘补到最高点：产品投影外、z∈[split_z, 产品最高点] 的水平带 ----
    p_zmax = product_bbox[5]
    if auto_wire is not None and p_zmax > split_z + 1e-6:
        cx_min, cy_min = core_bbox[0], core_bbox[1]
        cx_max, cy_max = core_bbox[3], core_bbox[4]
        max_dim = max(core_size)
        inflate = max_dim * 2.0
        top_band = BRepPrimAPI_MakeBox(
            gp_Pnt(cx_min - inflate, cy_min - inflate, split_z),
            gp_Pnt(cx_max + inflate, cy_max + inflate, p_zmax),
        ).Shape()
        proj_col = _contour_column(
            auto_wire, core_zmin - inflate,
            (core_zmax + inflate) - (core_zmin - inflate),
        )
        try:
            outer_band = bool_cut(top_band, proj_col, fuzzy_values, label="外缘带")
            outer_part = bool_common(core_with_cavity, outer_band, fuzzy_values, label="外缘块")
            if shape_volume(outer_part) > 1e-6:
                lower = bool_fuse(lower, outer_part, fuzzy_values, label="外缘熔接")
                print(f"  - 外缘补到产品最高点 z={p_zmax:.2f} mm ...")
        except Exception as exc:  # noqa: BLE001
            print(f"  - [跳过] 外缘块: {type(exc).__name__}")

    # ---- 2) 补 z>split_z 的下表面（爬过最高点再水平外切）----
    down_faces = []
    explorer = TopExp_Explorer(product_shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        explorer.Next()
        try:
            adaptor = BRepAdaptor_Surface(face)
            u0, u1 = adaptor.FirstUParameter(), adaptor.LastUParameter()
            v0, v1 = adaptor.FirstVParameter(), adaptor.LastVParameter()
            props = BRepLProp_SLProps(adaptor, (u0 + u1) * 0.5, (v0 + v1) * 0.5, 2, 1e-7)
            if not props.IsNormalDefined():
                continue
            normal = props.Normal()
            if face.Orientation() == TopAbs_REVERSED:
                normal = normal.Reversed()
            if -normal.Z() <= normal_threshold:
                continue
            fb = shape_bbox(face)
            if fb[5] <= split_z + 1e-6:
                continue
            down_faces.append((face, fb[5]))
        except Exception:  # noqa: BLE001
            continue

    n_block = 0
    for face, fzmax in down_faces:
        depth = fzmax - split_z
        if depth <= 0.0:
            continue
        try:
            prism = BRepPrimAPI_MakePrism(face, gp_Vec(0.0, 0.0, -depth)).Shape()
        except Exception:  # noqa: BLE001
            continue
        try:
            part = bool_common(core_with_cavity, prism, fuzzy_values, label="上壁随形块")
            pv = shape_volume(part)
            if pv > 1e-6 and pv < 0.95 * type_vol:
                lower = bool_fuse(lower, part, fuzzy_values, label="上壁随形融合")
                n_block += 1
        except Exception:  # noqa: BLE001
            continue
    if n_block:
        print(f"  - 补 {n_block} 个上壁随形块 ...")

    # ---- 3) 上模 = 型腔 - 下模 ----
    upper = bool_cut(core_with_cavity, lower, fuzzy_values, label="上模提取")
    upper_vol = shape_volume(upper)
    lower_vol = shape_volume(lower)
    if upper_vol <= 0.0 or lower_vol <= 0.0:
        raise PartingError(
            f"随形分模后体积异常 (上模={upper_vol:.3f}, 下模={lower_vol:.3f})"
        )

    warnings = detect_undercuts(product_shape, split_z, z_axis)
    return PartingResult(
        upper_mold=upper,
        lower_mold=lower,
        parting_z=split_z,
        parting_plane=gp_Pln(gp_Pnt(0.0, 0.0, split_z), gp_Dir(0.0, 0.0, 1.0)),
        upper_volume=upper_vol,
        lower_volume=lower_vol,
        undercut_warnings=warnings,
    )

def detect_undercuts(product_shape, z_parting: float, z_axis=None) -> List[UndercutWarning]:
    """倒扣区域检测（可选，仅输出警告，不阻塞流程）。

    判定规则（开模方向 +Z，分模面 z = z_parting）:
      * 法向朝上（nz > 0）的面必须由**上模**成形，因此必须位于分模面**以上**；
        若某个朝上的面落在分模面以下 —— 下模的料压在产品下方，
        产品沿 +Z 顶出时会被挡住 → 倒扣候选。
      * 法向朝下（nz < 0）的面必须由**下模**成形，必须位于分模面**以下**；
        若某个朝下的面落在分模面以上 —— 上模的料钩在产品下方，
        上模沿 +Z 抬起时会被挡住 → 倒扣候选。

    说明:
      水平面（法向近 ±Z）如果出现在"错误的一侧"，通常是分模高度选得不合理，
      而不是真的需要侧抽芯；此处仍会列出，message 中给出提示。

    注意: 旧版本这里判据写反了（把朝下面且低于分模面当成倒扣），
      会把普通型腔底面全部误报为倒扣，因此本条规则已修正。
    """
    if z_axis is None:
        z_axis = gp_Dir(0.0, 0.0, 1.0)

    warnings: List[UndercutWarning] = []
    explorer = TopExp_Explorer(product_shape, TopAbs_FACE)
    index = 0
    while explorer.More():
        index += 1
        face = TopoDS.Face_s(explorer.Current())
        try:
            adaptor = BRepAdaptor_Surface(face)
            u0, u1 = adaptor.FirstUParameter(), adaptor.LastUParameter()
            v0, v1 = adaptor.FirstVParameter(), adaptor.LastVParameter()
            props = BRepLProp_SLProps(adaptor, (u0 + u1) * 0.5, (v0 + v1) * 0.5, 2, 1e-7)
            if not props.IsNormalDefined():
                explorer.Next()
                continue
            normal = props.Normal()
            # 面法向应取实体外法向：几何法向需按面朝向（FORWARD/REVERSED）翻转
            if face.Orientation() == TopAbs_REVERSED:
                normal = normal.Reversed()
            gprops = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, gprops)
            area = gprops.Mass()
            center = gprops.CentreOfMass()
        except Exception:  # noqa: BLE001 —— 单个面探测失败不影响整体流程
            explorer.Next()
            continue

        nz = normal.Dot(z_axis)                     # = cos(法向与 +Z 夹角)
        angle_deg = math.degrees(normal.Angle(z_axis))
        # 面中心正好落在分模面上（与分模面共面）时不算倒扣，给 1e-6 容差
        if abs(center.Z() - z_parting) <= 1e-6:
            explorer.Next()
            continue
        above = center.Z() > z_parting
        is_horizontal = abs(nz) > 0.995
        if nz > 0.02 and not above:
            msg = ("朝上表面低于分模面：产品顶出时会被下模挡住"
                   + ("（水平面，建议调整分模高度）" if is_horizontal
                      else "，需侧抽芯/斜顶或随形分模面"))
        elif nz < -0.02 and above:
            msg = ("朝下表面高于分模面：上模抬起时会被钩住"
                   + ("（水平面，建议调整分模高度）" if is_horizontal
                      else "，需侧抽芯/斜顶或随形分模面"))
        else:
            explorer.Next()
            continue
        warnings.append(
            UndercutWarning(
                face_index=index,
                position=(
                    round(center.X(), 3), round(center.Y(), 3), round(center.Z(), 3)
                ),
                area=area,
                normal=(round(normal.X(), 5), round(normal.Y(), 5), round(normal.Z(), 5)),
                angle_deg=round(angle_deg, 2),
                is_horizontal=is_horizontal,
                message=msg,
            )
        )
        explorer.Next()
    return warnings


if __name__ == "__main__":
    # 自测入口
    from io_utils import ensure_utf8_stdout
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2

    ensure_utf8_stdout()
    # 构造一个带型腔的模芯（挖掉一个圆柱）
    core = BRepPrimAPI_MakeBox(gp_Pnt(-50.0, -50.0, 0.0), gp_Pnt(50.0, 50.0, 60.0)).Shape()
    product = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), 15.0, 30.0
    ).Shape()
    cavity = bool_cut(core, product, label="自测挖腔")
    for mode, zp in (("max_z", None), ("mid", None), ("custom", 20.0), ("contour", None)):
        result = split_mold(cavity, product, (100.0, 100.0, 60.0),
                            parting_mode=mode, z_parting=zp)
        print(f"mode={mode:8s} z_parting={result.parting_z:7.3f} "
              f"上模={result.upper_volume:9.3f} 下模={result.lower_volume:9.3f} "
              f"倒扣={len(result.undercut_warnings)}")

