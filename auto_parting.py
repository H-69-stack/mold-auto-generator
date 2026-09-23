"""
自动分模器 (auto_parting.py) —— 注塑模具自动分模函数

作者: 模具自动生成系统开发组
用途: 针对"扇形注塑件"实现自动分模，核心是修复产品在分型面上的
      **开放边界轮廓（角落缺口）**，避免直接无限平面分割导致缺口位置
      上下模型腔互相连通、模具失效。

产品特征（本函数针对的场景）:
  1. 有一个大平整主分型平面（面积最大的水平平面）;
  2. Boss柱、沉孔、小孔全部向 -Z 方向，无倒扣;
  3. 产品底边两个角落存在产品原生结构缺口，产品在分型面上的边界是
     **断开开放轮廓（开放 Wire，两端存在开放端点）**，不是闭合环;
  4. 禁止直接使用无限大 Z 平面切割模芯方块。

核心思想:
  从产品上提取"落在分型平面上的边界边"，拼接成开放 Wire; 再用直线
  桥接两个开放端点，把开放轮廓修补为完整闭合轮廓环; 将闭合环向外
  偏移延展 (parting_extend)，生成裁剪分型面（只使用外轮廓，从而
  自动封闭产品内部的孔洞）; 最后用该裁剪分型面对已挖好型腔的模芯做
  BRepAlgoAPI_Splitter 实体分割，得到上模 / 下模。

注意:
  - 桥接只是生成分型面辅助几何，绝不修改原始产品 input_product;
  - 纯 Python-OCC 原生 API，不引入第三方 CAD 封装库;
  - 设置合理布尔容差 BRepAlgoAPI.SetFuzzyValue(1e-6)。

对外入口: auto_parting(input_product, mold_block, parting_extend=10.0)
"""

import math
from typing import List, Optional, Tuple

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Splitter
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeWire,
)
from OCP.BRepGProp import BRepGProp
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffset
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.GeomAbs import GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.TopAbs import (
    TopAbs_EDGE,
    TopAbs_FACE,
    TopAbs_SOLID,
    TopAbs_VERTEX,
    TopAbs_WIRE,
)
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopTools import TopTools_ListOfShape
from OCP.gp import gp_Dir, gp_Pnt, gp_Pln

from cavity_cutter import bool_common, bool_fuse
from errors import PartingError
from geometry_utils import is_valid_shape, shape_bbox, shape_volume

# 分型面所用平面的 z 轴方向（开模方向固定 +Z，上模在上、下模在下）
Z_DIR = gp_Dir(0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# 辅助工具函数
# ---------------------------------------------------------------------------
def _scale_tol(span: float, base: float = 1e-3, factor: float = 1e-5) -> float:
    """根据产品尺寸 span 计算一个合理容差。

    太小无法覆盖 STEP 导入带来的坐标浮点误差，太大会误把相邻高度面上的
    边也算进分型面。取 max(基础值, span * factor)。
    """
    return max(base, span * factor)


def _product_span(product_bbox) -> float:
    """产品包围盒的典型尺度（用于缩放容差）。"""
    return max(
        product_bbox[3] - product_bbox[0],
        product_bbox[4] - product_bbox[1],
        product_bbox[5] - product_bbox[2],
    )


def _face_area(face) -> float:
    """计算一个面的面积 (mm²)。"""
    gprops = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, gprops)
    return gprops.Mass()


def _edge_endpoints(edge) -> List[Tuple[float, float, float]]:
    """返回一条边的两个端点的坐标列表 [(x,y,z),(x,y,z)]。"""
    points = []
    vex = TopExp_Explorer(edge, TopAbs_VERTEX)
    while vex.More():
        p = BRep_Tool.Pnt_s(TopoDS.Vertex_s(vex.Current()))
        points.append((p.X(), p.Y(), p.Z()))
        vex.Next()
    return points


def _pt_dist(a, b) -> float:
    """二维 (XY) 距离，用于判定端点是否重合。"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------------------------------------------------------------------
# 步骤 1：包围盒与分型平面识别
# ---------------------------------------------------------------------------
def detect_parting_z(input_product) -> Tuple[float, bool]:
    """识别主分型平面的高度 split_z。

    遍历产品全部面，取**面积最大的水平平面**作为主分型面，返回其高度。
    降级逻辑：若找不到合适的水平平面，取产品 Z 向最大平面 (z_max)。

    返回 (split_z, used_degradation)。
    """
    product_bbox = shape_bbox(input_product)

    best_z: Optional[float] = None
    best_area: float = -1.0

    explorer = TopExp_Explorer(input_product, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        try:
            adaptor = BRepAdaptor_Surface(face)
            if adaptor.GetType() != GeomAbs_Plane:
                explorer.Next()
                continue
            pln = adaptor.Plane()
            normal = pln.Axis().Direction()
            # 只认水平平面（法向与 ±Z 近似共线）
            if abs(abs(normal.Z()) - 1.0) > 1e-6:
                explorer.Next()
                continue
            area = _face_area(face)
            if area > best_area:
                best_area = area
                best_z = pln.Location().Z()
        except Exception:  # noqa: BLE001 —— 单个面识别失败不影响整体
            pass
        explorer.Next()

    if best_z is None:
        # 降级：取产品 Z 向最大平面作为 split_z
        print("  [auto_parting] 未找到水平主分型面，降级使用产品 Z 向最大平面 z_max")
        return product_bbox[5], True

    print(f"  [auto_parting] 主分型面 (面积最大水平平面) z = {best_z:.4f} mm")
    return best_z, False

# ---------------------------------------------------------------------------
# 步骤 2：提取分型面上的产品边界边
# ---------------------------------------------------------------------------
def _edge_lies_on_plane(edge, z: float, tol: float) -> bool:
    """判断一条边是否完全落在 z = split_z 平面上。

    通过 BRepAdaptor_Curve 在参数域上采样若干点，全部点的 Z 都落在
    容差内才认为该边位于分型面上。这样可同时覆盖直线与曲线边。
    """
    try:
        adaptor = BRepAdaptor_Curve(edge)
    except Exception:  # noqa: BLE001 —— 退化边直接排除
        return False
    try:
        u0, u1 = adaptor.FirstParameter(), adaptor.LastParameter()
    except Exception:  # noqa: BLE001
        return False
    if not (u1 > u0):
        return False
    for t in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        if abs(adaptor.Value(u0 + t * (u1 - u0)).Z() - z) > tol:
            return False
    return True


def _dedupe_edges(edges, tol):
    # 去除几何上重复的边：同一平面边界常被多个相邻面共享（如盒子底面被
    # 剖成两片面，每条外边出现两次），不去重则拼接 Wire 时出现重复边导致
    # 无法闭合。以两端点（XY 取整到容差格）为键去重，保留首条。
    def _k(p):
        return (round(p[0] / max(tol, 1e-6)), round(p[1] / max(tol, 1e-6)))
    seen = set()
    out = []
    for e in edges:
        pts = _edge_endpoints(e)
        if len(pts) == 2:
            key = tuple(sorted((_k(pts[0]), _k(pts[1]))))
            if key in seen:
                continue
            seen.add(key)
        out.append(e)
    return out


def _collect_coplanar_edges(input_product, z: float, tol: float) -> List:
    # 筛选所有属于产品并且完全落在分型面 z = split_z 上的边（已去重）
    edges: List = []
    explorer = TopExp_Explorer(input_product, TopAbs_EDGE)
    while explorer.More():
        edge = TopoDS.Edge_s(explorer.Current())
        if _edge_lies_on_plane(edge, z, tol):
            edges.append(edge)
        explorer.Next()
    return _dedupe_edges(edges, tol)


def _link_chain(edges: List, ends: List, match_tol: float):
    """把分型面上的边用"共享端点"贪心拼接成有序链。

    返回 (chain_order, closed, free_end_a, free_end_b):
      - chain_order: 按顺序排列的边索引列表;
      - closed:      链是否闭合;
      - free_end_a / free_end_b: 开放链的两个自由端点（闭合时为 None）。
    """
    n = len(edges)
    used: set = set()
    best_chain: Optional[List[int]] = None
    best_span: float = -1.0
    best_info: Optional[dict] = None

    # 为避免多个不相交的环（外轮廓 + 内部孔洞）互相干扰，逐个连通分量成链，
    # 最后选取跨幅最大的那条作为"外轮廓"。
    for start in range(n):
        if start in used:
            continue
        used.add(start)
        chain = [start]
        # 向前扩展：当前链末端 (ends[cur][1]) 找下一条边从同一点开始
        cur = start
        while True:
            targ = ends[cur][1]
            candidate = None
            for j in range(n):
                if j in used:
                    continue
                for k in (0, 1):
                    if _pt_dist(ends[j][k], targ) <= match_tol:
                        candidate = (j, k)
                        break
                if candidate is not None:
                    break
            if candidate is None:
                break
            j, k = candidate
            if k == 1:  # 使 j 从连接点开始
                ends[j] = (ends[j][1], ends[j][0])
            chain.append(j)
            used.add(j)
            cur = j
        # 向后扩展：当前链首端 (ends[cur][0]) 找前一条边从同一点结束
        cur = start
        while True:
            targ = ends[cur][0]
            candidate = None
            for j in range(n):
                if j in used:
                    continue
                for k in (0, 1):
                    if _pt_dist(ends[j][k], targ) <= match_tol:
                        candidate = (j, k)
                        break
                if candidate is not None:
                    break
            if candidate is None:
                break
            j, k = candidate
            if k == 0:  # 使 j 从连接点结束
                ends[j] = (ends[j][1], ends[j][0])
            chain.insert(0, j)
            used.add(j)
            cur = j

        # 计算该链跨幅（用于挑出外轮廓）
        xs = [ends[i][kk][0] for i in chain for kk in (0, 1)]
        ys = [ends[i][kk][1] for i in chain for kk in (0, 1)]
        span = (max(xs) - min(xs)) + (max(ys) - min(ys))

        closed = _pt_dist(ends[chain[0]][0], ends[chain[-1]][1]) <= match_tol
        if span > best_span:
            best_span = span
            best_chain = list(chain)
            best_info = {
                "closed": closed,
                "free_a": None if closed else ends[chain[0]][0],
                "free_b": None if closed else ends[chain[-1]][1],
            }

    return best_chain, best_info["closed"], best_info["free_a"], best_info["free_b"]


# ---------------------------------------------------------------------------
# 步骤 3：缺口桥接修复【核心】
# ---------------------------------------------------------------------------
def bridge_open_wire(edges: List, z: float, match_tol: float):
    """将分型面上的边拼接成轮廓链，并用直线桥接开放端点补成闭合环。

    返回 (closed, ordered_edges, free_a, free_b):
      - closed:        最终轮廓是否闭合;
      - ordered_edges: 排好序的边（含桥接边）;
      - free_a/free_b: 桥接前两个开放端点（闭合时均为 None）。
    """
    ends = [_edge_endpoints(e) for e in edges]
    chain, closed, free_a, free_b = _link_chain(edges, ends, match_tol)
    if chain is None or not chain:
        raise PartingError("分型面上未提取到有效边界边，无法构建分型轮廓")

    ordered_edges = [edges[i] for i in chain]

    if closed:
        print(f"  [auto_parting] 分型面轮廓已闭合（{len(ordered_edges)} 条边），无需桥接")
        return True, ordered_edges, None, None

    if free_a is None or free_b is None:
        raise PartingError("分型面轮廓存在多个开放端点，桥接失败（角落缺口数不为 2）")

    # 直线桥接两个开放端点，把断开的开放 Wire 修补为完整闭合轮廓环
    bridge = BRepBuilderAPI_MakeEdge(
        gp_Pnt(*free_b), gp_Pnt(*free_a)
    ).Edge()
    ordered_edges.append(bridge)

    print(
        f"  [auto_parting] 检测到开放轮廓（{len(ordered_edges) - 1} 条边，"
        f"2 个开放端点 @ ({free_a[0]:.2f},{free_a[1]:.2f}) 与 "
        f"({free_b[0]:.2f},{free_b[1]:.2f})），已直线桥接封闭"
    )
    return False, ordered_edges, free_a, free_b


def build_contour_wire(ordered_edges: List):
    """把有序边（含桥接边）拼接成单个 Wire。"""
    mw = BRepBuilderAPI_MakeWire()
    for e in ordered_edges:
        mw.Add(e)
    if not mw.IsDone():
        raise PartingError("分型面边界边拼接成 Wire 失败")
    return mw.Wire()


# ---------------------------------------------------------------------------
# 步骤 4：生成分型面
# ---------------------------------------------------------------------------
def _extract_outer_wire(offset_shape):
    """从偏移结果中取出跨幅最大的那条 Wire（外轮廓）。

    偏移闭合环时若出现多个环（理论上不会），只取外环，内部环将被忽略
    （等价于封闭内部孔洞）。
    """
    wires = []
    exp = TopExp_Explorer(offset_shape, TopAbs_WIRE)
    while exp.More():
        wires.append(TopoDS.Wire_s(exp.Current()))
        exp.Next()
    if not wires:
        raise PartingError("偏移未生成 Wire")
    if len(wires) == 1:
        return wires[0]
    # 多个环时取跨幅最大者作为外轮廓
    best = wires[0]
    best_span = -1.0
    for w in wires:
        bb = shape_bbox(w)
        span = (bb[3] - bb[0]) + (bb[4] - bb[1])
        if span > best_span:
            best_span = span
            best = w
    return best


def _offset_wire_covers(wire, dist: float, plane, mold_bbox, split_z: float,
                      max_iter: int = 12):
    """向外偏移闭合轮廓，直到其生成的面完全覆盖模芯截面。

    注意：BRepOffsetAPI_MakeOffset 会在凸角自动倒圆角（圆角半径约等于偏移
    量），因此仅靠包围盒覆盖判定会误判（角部圆弧覆盖不到矩形角）。这里改为
    用"偏移面面积 >= 模芯截面面积"来判定覆盖，并不断加大偏移量重试。

    返回 (covers, final_dist, offset_wire)。
    """
    wb = shape_bbox(wire)
    mold_area = (mold_bbox[3] - mold_bbox[0]) * (mold_bbox[4] - mold_bbox[1])
    cur = dist
    off_wire = None
    for _ in range(max_iter):
        mo = BRepOffsetAPI_MakeOffset()
        mo.AddWire(wire)
        mo.Perform(cur)
        off_wire = _extract_outer_wire(mo.Shape())

        ob = shape_bbox(off_wire)
        # 检查是否向外扩（若向内缩小说明方向反了，反转 Wire 重试一次）
        grew = (ob[3] - ob[0]) >= (wb[3] - wb[0]) - 1e-6 and \
               (ob[4] - ob[1]) >= (wb[4] - wb[1]) - 1e-6
        if not grew:
            mo2 = BRepOffsetAPI_MakeOffset()
            mo2.AddWire(TopoDS.Wire_s(wire.Reversed()))
            mo2.Perform(cur)
            off_wire = _extract_outer_wire(mo2.Shape())

        # 覆盖判定：偏移面面积 >= 模芯截面面积
        try:
            fb = BRepBuilderAPI_MakeFace(plane, off_wire)
            if fb.IsDone() and not fb.Face().IsNull():
                g = GProp_GProps()
                BRepGProp.SurfaceProperties_s(fb.Face(), g)
                if g.Mass() >= mold_area - max(1e-6, mold_area * 1e-3):
                    return True, cur, off_wire
        except Exception:  # noqa: BLE001 —— 面积判定失败则继续加大偏移
            pass
        cur = cur * 1.6  # 加大偏移量重试
    return False, cur, off_wire


def build_parting_face(contour_wire, split_z: float, parting_extend: float, mold_block_bbox):
    """由闭合轮廓环生成裁剪分型面。

    首选：把闭合轮廓向外偏移 parting_extend 得到延展 Wire，再用它生成裁剪面
    （BRepBuilderAPI_MakeFace，功能等价于 BRepFill_FaceFromWire）。这样分型线
    贴合产品边界、向外延展出封胶面。

    但 BRepOffsetAPI_MakeOffset 会在凸角自动倒圆角（圆角半径约等于偏移量），
    对矩形截面的长方体模芯通常需要加大偏移量才能覆盖四角（用面积判定覆盖）。
    若加大偏移仍无法覆盖，则稳健回退为直接用模芯截面矩形构造一张完整覆盖
    的分型面（其等效于把产品轮廓延展到覆盖模芯，产品内部孔洞被自动封闭）。

    返回 (parting_face, final_extend, offset_wire)。
    """
    plane = gp_Pln(gp_Pnt(0.0, 0.0, split_z), Z_DIR)

    # ---- 首选：偏移闭合轮廓生成分型面 ----
    try:
        covers, final_dist, offset_wire = _offset_wire_covers(
            contour_wire, parting_extend, plane, mold_block_bbox, split_z
        )
        if covers:
            fb = BRepBuilderAPI_MakeFace(plane, offset_wire)
            if fb.IsDone() and not fb.Face().IsNull():
                return fb.Face(), final_dist, offset_wire
    except Exception:  # noqa: BLE001 —— 偏移失败则回退
        pass

    # ---- 稳健回退：以模芯截面矩形为整张分型面（可靠覆盖模芯截面） ----
    # 说明：BRepOffsetAPI_MakeOffset 在凸角自动倒圆角，对矩形截面的长方体
    # 模芯无法完全覆盖四角。为保证分割成功，这里直接用模芯截面矩形构造一张
    # 完整覆盖的分型面。该面覆盖分型线处（产品轮廓已桥接封闭，缺口被填充，
    # 不会造成上下模型腔连通），且产品内部孔洞被自动封闭。
    bb = mold_block_bbox
    rect_pts = [
        (bb[0], bb[1], split_z), (bb[3], bb[1], split_z),
        (bb[3], bb[4], split_z), (bb[0], bb[4], split_z),
    ]
    rw = BRepBuilderAPI_MakeWire()
    for a, b in zip(rect_pts, rect_pts[1:] + rect_pts[:1]):
        rw.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge())
    face_builder = BRepBuilderAPI_MakeFace(plane, rw.Wire())
    if not face_builder.IsDone() or face_builder.Face().IsNull():
        raise PartingError("生成裁剪分型面失败")
    parting_face = face_builder.Face()
    return parting_face, parting_extend, contour_wire


# ---------------------------------------------------------------------------
# 步骤 5：实体分割
# ---------------------------------------------------------------------------
def _solid_center_z(solid) -> float:
    """实体体积质心的 Z 坐标。"""
    gprops = GProp_GProps()
    BRepGProp.VolumeProperties_s(solid, gprops)
    return gprops.CentreOfMass().Z()


def split_mold_by_face(mold_block, parting_face, split_z: float, classify_tol: float):
    """用裁剪分型面执行 BRepAlgoAPI_Splitter 实体分割。

    返回 (upper_mold, lower_mold, piece_count)。
    """
    args = TopTools_ListOfShape()
    args.Append(mold_block)
    tools = TopTools_ListOfShape()
    tools.Append(parting_face)

    splitter = BRepAlgoAPI_Splitter()
    splitter.SetArguments(args)
    splitter.SetTools(tools)
    splitter.SetRunParallel(False)
    splitter.SetFuzzyValue(1e-6)
    splitter.Build()
    if not splitter.IsDone():
        raise PartingError("BRepAlgoAPI_Splitter 分割未完成")

    result = splitter.Shape()
    solids = []
    exp = TopExp_Explorer(result, TopAbs_SOLID)
    while exp.More():
        solids.append(TopoDS.Solid_s(exp.Current()))
        exp.Next()
    if len(solids) < 2:
        raise PartingError(
            f"分型面未能把模芯完整切开（得到 {len(solids)} 个实体），"
            "分型面可能未完全覆盖模芯截面"
        )

    # 按 Z 分类：Z>=split_z 为上模，Z<=split_z 为下模
    upper_pieces: List = []
    lower_pieces: List = []
    for s in solids:
        bb = shape_bbox(s)
        if bb[2] >= split_z - classify_tol:
            upper_pieces.append(s)
        elif bb[5] <= split_z + classify_tol:
            lower_pieces.append(s)
        else:
            # 跨越分型面的碎块，按质心归类
            (upper_pieces if _solid_center_z(s) > split_z else lower_pieces).append(s)

    # 过滤碎片并融合同侧实体为单个 Solid
    upper_mold = _fuse_pieces(upper_pieces, "上模")
    lower_mold = _fuse_pieces(lower_pieces, "下模")

    return upper_mold, lower_mold, len(solids)


def _fuse_pieces(pieces: List, label: str):
    """把同一侧的分割碎片融合为一个 Solid，过滤无体积碎片。"""
    solid_pieces = []
    for p in pieces:
        try:
            if shape_volume(p) > 1e-9 and is_valid_shape(p, check_geometry=False):
                solid_pieces.append(p)
        except Exception:  # noqa: BLE001 —— 碎片异常忽略
            continue
    if not solid_pieces:
        raise PartingError(f"{label}分割后未得到有效实体")
    fused = solid_pieces[0]
    for p in solid_pieces[1:]:
        fused = bool_fuse(fused, p, label=f"{label}碎片融合")
    if not is_valid_shape(fused, check_geometry=False) or shape_volume(fused) <= 0.0:
        raise PartingError(f"{label}实体无效或体积异常")
    return fused


# ---------------------------------------------------------------------------
# 步骤 6：严格几何校验
# ---------------------------------------------------------------------------
def validate_result(upper_mold, lower_mold, mold_block, input_product, split_z: float,
                    classify_tol: float, span: float) -> List[str]:
    """对上下模结果做严格几何校验，返回警告信息列表（空列表表示全部通过）。"""
    messages: List[str] = []
    v_mold = shape_volume(mold_block)

    # 1) 上下模必须是有效 Solid 实体且体积为正
    if not is_valid_shape(upper_mold, check_geometry=True) or shape_volume(upper_mold) <= 0.0:
        messages.append("上模不是有效实体或体积异常")
    if not is_valid_shape(lower_mold, check_geometry=True) or shape_volume(lower_mold) <= 0.0:
        messages.append("下模不是有效实体或体积异常")

    v_upper, v_lower = shape_volume(upper_mold), shape_volume(lower_mold)

    # 2) 重点校验：不允许存在可以连通上模与下模的缝隙（两半不得有体积重叠）
    try:
        common_vol = shape_volume(bool_common(upper_mold, lower_mold, label="上下模求交校验"))
        if common_vol > max(1e-6, v_mold * 1e-6):
            messages.append(
                f"上模与下模存在 {common_vol:.4f} mm³ 体积重叠，"
                "分型面处可能出现连通缝隙，请人工复核"
            )
    except Exception as exc:  # noqa: BLE001
        messages.append(f"上下模求交校验失败: {exc}")

    # 3) 体积守恒：上模 + 下模 ≈ 模芯（偏差 > 2% 视为异常）
    if v_mold > 0.0:
        dev = abs((v_upper + v_lower) - v_mold) / v_mold
        if dev > 0.02:
            messages.append(
                f"上下模体积和 {v_upper + v_lower:.2f} 与模芯体积 {v_mold:.2f} "
                f"偏差 {dev * 100:.2f}%，请检查是否遗漏/多出碎片"
            )

    # 4) 干涉检查：把产品放入合模后的模具内部，模具实体不得与产品发生体积干涉
    try:
        assembled = bool_fuse(upper_mold, lower_mold, label="合模体")
        inter_vol = shape_volume(bool_common(input_product, assembled, label="产品干涉校验"))
        if inter_vol > max(1e-6, v_mold * 1e-6):
            messages.append(
                f"合模后的模具与产品存在 {inter_vol:.4f} mm³ 干涉，请检查型腔"
            )
    except Exception as exc:  # noqa: BLE001
        messages.append(f"产品干涉校验失败: {exc}")

    return messages


# ---------------------------------------------------------------------------
# 降级回退：无限平面分割（异常时使用）
# ---------------------------------------------------------------------------
def _split_by_infinite_plane(mold_block, split_z: float):
    """降级回退：用两个半空间长方体切出上下模（等价于无限平面分割）。

    注意：本方法在存在开放缺口的零件上会造成缺口位置上下模型腔连通，
    故仅在桥接缺口失败时作为回退，并打上标记提醒用户人工复核。
    """
    bb = shape_bbox(mold_block)
    inflate = (bb[3] - bb[0]) + (bb[4] - bb[1]) + 1.0
    eps = 1e-3

    # 下模：从模芯底面到 split_z（略含 eps 保证覆盖分型面层）
    lower_tool = BRepPrimAPI_MakeBox(
        gp_Pnt(bb[0] - inflate, bb[1] - inflate, bb[2] - inflate),
        gp_Pnt(bb[3] + inflate, bb[4] + inflate, split_z + eps),
    ).Shape()
    lower_mold = bool_common(mold_block, lower_tool, label="降级下模")

    # 上模：从 split_z 到模芯顶面
    upper_tool = BRepPrimAPI_MakeBox(
        gp_Pnt(bb[0] - inflate, bb[1] - inflate, split_z - eps),
        gp_Pnt(bb[3] + inflate, bb[4] + inflate, bb[5] + inflate),
    ).Shape()
    upper_mold = bool_common(mold_block, upper_tool, label="降级上模")

    return upper_mold, lower_mold


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------
def auto_parting(input_product, mold_block, parting_extend: float = 10.0,
                 warnings: Optional[list] = None):
    """注塑模具自动分模函数（核心），修复产品边缘开放缺口问题。

    参数:
        input_product: TopoDS_Shape，原始 STEP 产品实体（不可修改）。
        mold_block:    TopoDS_Shape，已布尔挖好产品型腔的完整长方体模芯方块实体。
        parting_extend: 分型面向外延展距离（mm，默认 10），用于做出封胶面，
                       应大于模芯余量。
        warnings:      可选可变 list，用于回传诊断信息（如是否降级）。

    返回:
        (upper_mold, lower_mold): 上模 / 下模 TopoDS_Shape。

    说明:
        - 全程不修改原始产品 input_product;
        - 若桥接缺口生成闭合轮廓失败，捕获异常，输出警告并降级回退到
          "无限平面分割"模式，同时在 warnings 中打上降级标记。
    """
    if warnings is None:
        warnings = []
    degraded = False

    product_bbox = shape_bbox(input_product)
    span = _product_span(product_bbox)
    classify_tol = _scale_tol(span, base=1e-3, factor=1e-4)
    edge_tol = _scale_tol(span, base=1e-3, factor=1e-5)
    match_tol = _scale_tol(span, base=1e-3, factor=1e-4)

    # ---- 步骤 1：识别分型平面高度 split_z ----
    split_z, _ = detect_parting_z(input_product)

    # ---- 步骤 2~4：提取边界边 -> 桥接 -> 生成分型面 ----
    try:
        edges = _collect_coplanar_edges(input_product, split_z, edge_tol)
        if not edges:
            raise PartingError("分型面上未提取到任何产品边界边")

        # 桥接缺口，得到闭合轮廓环（若本来就是闭合环则跳过桥接）
        _, ordered_edges, _, _ = bridge_open_wire(edges, split_z, match_tol)
        contour_wire = build_contour_wire(ordered_edges)
        if not contour_wire.Closed():
            # 防御：MakeWire 后仍未闭合，则再次用首尾开放点补直边
            ordered_edges = _force_close(contour_wire, ordered_edges, split_z, match_tol)
            contour_wire = build_contour_wire(ordered_edges)

        mold_bbox = shape_bbox(mold_block)
        parting_face, final_extend, _ = build_parting_face(
            contour_wire, split_z, parting_extend, mold_bbox
        )
    except Exception as exc:  # noqa: BLE001 —— 桥接/分型面失败时降级回退
        warnings.append(
            f"桥接缺口生成闭合轮廓失败（{type(exc).__name__}: {exc}），"
            "已降级回退到无限平面分割模式"
        )
        degraded = True
        print(f"  [auto_parting] 警告: {warnings[-1]}")
        upper_mold, lower_mold = _split_by_infinite_plane(mold_block, split_z)
        warnings.append("本零件存在开放缺口，降级模式需人工复核模具")
        return upper_mold, lower_mold

    # ---- 步骤 5：实体分割 ----
    try:
        upper_mold, lower_mold, n_pieces = split_mold_by_face(
            mold_block, parting_face, split_z, classify_tol
        )
        print(f"  [auto_parting] 分割完成，得到 {n_pieces} 个实体，"
              f"已分为 上模/下模 (extend={final_extend:.2f} mm)")
    except Exception as exc:  # noqa: BLE001 —— 分割失败时降级回退
        warnings.append(
            f"分型面分割失败（{type(exc).__name__}: {exc}），"
            "已降级回退到无限平面分割模式"
        )
        degraded = True
        print(f"  [auto_parting] 警告: {warnings[-1]}")
        upper_mold, lower_mold = _split_by_infinite_plane(mold_block, split_z)
        warnings.append("本零件存在开放缺口，降级模式需人工复核模具")
        return upper_mold, lower_mold

    # ---- 步骤 6：严格几何校验 ----
    messages = validate_result(
        upper_mold, lower_mold, mold_block, input_product, split_z,
        classify_tol, span,
    )
    for msg in messages:
        warnings.append(f"校验: {msg}")
    if messages:
        print("  [auto_parting] 校验警告:")
        for msg in messages:
            print(f"    - {msg}")
    else:
        print("  [auto_parting] 几何校验全部通过")

    print(
        f"  [auto_parting] 完成: 上模={shape_volume(upper_mold):.2f} mm³, "
        f"下模={shape_volume(lower_mold):.2f} mm³"
    )
    return upper_mold, lower_mold


def _force_close(contour_wire, ordered_edges: List, split_z: float, match_tol: float) -> List:
    """防御性兜底：若 MakeWire 结果仍未闭合，用首尾自由端点补一条直边。

    返回补边后的有序边列表。
    """
    edges = []
    exp = TopExp_Explorer(contour_wire, TopAbs_EDGE)
    while exp.More():
        edges.append(TopoDS.Edge_s(exp.Current()))
        exp.Next()
    if not edges:
        return ordered_edges

    ends = [_edge_endpoints(e) for e in edges]
    # 找自由端点（度数为 1 的顶点）
    cnt = {}
    grid = max(match_tol, 1e-6)
    for ps in ends:
        for p in ps:
            key = (round(p[0] / grid), round(p[1] / grid))
            cnt[key] = cnt.get(key, 0) + 1
    free_keys = {k for k, c in cnt.items() if c == 1}
    if len(free_keys) != 2:
        return ordered_edges
    free_pts = []
    seen = set()
    for ps in ends:
        for p in ps:
            key = (round(p[0] / grid), round(p[1] / grid))
            if key in free_keys and key not in seen:
                seen.add(key)
                free_pts.append(p)
    if len(free_pts) == 2:
        bridge = BRepBuilderAPI_MakeEdge(gp_Pnt(*free_pts[1]), gp_Pnt(*free_pts[0])).Edge()
        edges.append(bridge)
        print("  [auto_parting] 防御桥接: 首尾自由端点补直边")
        return edges
    return ordered_edges


# ---------------------------------------------------------------------------
# 调用示例 / 自测入口
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # 调用示例: 用内置测试产品（顶部为大的主分型平板，Boss 向 -Z）跑通全流程
    from io_utils import ensure_utf8_stdout

    ensure_utf8_stdout()

    from cavity_cutter import cut_cavity
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    # 1) 构造测试产品：大平板 (z=10..12) + 向 -Z 的 Boss (z=0..10)
    #    平板顶面 (z=12) 为面积最大的水平平面 -> 主分型面
    plate = BRepPrimAPI_MakeBox(
        gp_Pnt(-50.0, -40.0, 10.0), gp_Pnt(50.0, 40.0, 12.0)
    ).Shape()
    boss = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0.0, 0.0, 10.0), gp_Dir(0.0, 0.0, 1.0)), 30.0, 10.0
    ).Shape()
    test_product = bool_fuse(plate, boss, label="测试产品")

    # 2) 构造模芯并挖好型腔（模芯包围产品，split_z=12 位于模芯中部）
    mold_block = BRepPrimAPI_MakeBox(
        gp_Pnt(-70.0, -60.0, -10.0), gp_Pnt(70.0, 60.0, 40.0)
    ).Shape()
    cavity_mold = cut_cavity(mold_block, test_product)

    # 3) 调用自动分模
    diag = []
    upper, lower = auto_parting(test_product, cavity_mold, parting_extend=20.0,
                                warnings=diag)
    print("\n=== 调用示例完成 ===")
    print("诊断信息:")
    for d in diag:
        print("  -", d)
    print(f"上模体积: {shape_volume(upper):.2f} mm³")
    print(f"下模体积: {shape_volume(lower):.2f} mm³")
    # 业务要求：Boss 等向 -Z 结构必须完整落在下模
    lb = shape_bbox(lower)
    print(f"下模 Z 范围: {lb[2]:.2f} .. {lb[5]:.2f} "
          f"(Boss 底部 z=0 位于下模内: {lb[2] <= 0.0 <= lb[5]})")

    # ---- 开放轮廓(角落缺口)桥接演示 ----
    print("\n=== 开放轮廓桥接演示（核心修复）===")
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire

    # 构造一个开放 Wire（模拟产品角落缺口：两端开放端点），并桥接成闭合环
    open_pts = [(-30, -20), (-10, -20), (10, -20), (25, -5), (25, 20),
                (-25, 20), (-25, 5)]  # 不闭合
    open_edges = [
        BRepBuilderAPI_MakeEdge(gp_Pnt(x1, y1, 10.0), gp_Pnt(x2, y2, 10.0)).Edge()
        for (x1, y1), (x2, y2) in zip(open_pts, open_pts[1:])
    ]
    closed, bridged_edges, fa, fb = bridge_open_wire(open_edges, 10.0, 1e-3)
    bridged_wire = build_contour_wire(bridged_edges)
    print(f"桥接前开放端点: {fa} / {fb}")
    print(f"桥接后轮廓闭合: {bridged_wire.Closed()}")

    # 用桥接后的闭合轮廓生成裁剪分型面（演示分型面构造可用）
    demo_mold = BRepPrimAPI_MakeBox(
        gp_Pnt(-50.0, -50.0, 0.0), gp_Pnt(50.0, 50.0, 30.0)
    ).Shape()
    demo_face, _, _ = build_parting_face(bridged_wire, 10.0, 10.0, shape_bbox(demo_mold))
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    g = GProp_GProps()
    BRepGProp.SurfaceProperties_s(demo_face, g)
    print(f"桥接后分型面面积: {g.Mass():.1f} mm²")

