# -*- coding: utf-8 -*-
"""管件硬模成型模具生成 (pipe_mold.py)

适用对象: 薄壁中空件（管件、型材件）—— 硬模（刚性模）成型，模具只成形产品**外形**，
          不形成内孔（内孔由管坯自身/内压/芯棒保证）。

与板件折弯管线（cavity_cutter + parting_splitter）的关键差别
------------------------------------------------------------------
1. **型腔 = 产品外形包络**（把内孔"填实"后再挖），而不是"模芯-产品实体"。
   原因: 直接挖产品实体时，管件内孔里会留下一根与主体相连的"内芯条"，
        上下模都夹着它 → **产品既顶不出、模也合不拢**（实测开模/顶出干涉 2.4e4 mm³）。
   本模块做法: 在产品端面内环（内孔的开口）上加"封盖薄片"，挖腔时把内芯条与
   主体切断，再只保留主体（最大实体）→ 得到的就是"外形包络型腔"。

2. **分模面 = 产品侧影线 h(轴向站位)**，随管件弯曲/上翘而变化，而不是一刀水平面。
   原理: 开模方向 +Z 时，分型线必须落在产品表面**法向水平**处（侧影线，nz≈0）：
         nz>0 的面必须在上模、nz<0 的面必须在下模。只用一个水平面切，
         管件末端上翘段会出现"台阶/倒扣"，上模抬不起来。
   本模块从精确 BRep 表面采样求侧影高度，并用体积/干涉检验闭环验证。

3. **闭环校验**: 分模后用点采样法验证
     · 上模沿 +Z 抬起 vs 产品        · 产品沿 +Z 顶出 vs 下模
   两者干涉量都应为 0 —— 保证"开得开、顶得出"。
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import time

import numpy as np

from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon,
                                BRepBuilderAPI_Transform)
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepGProp import BRepGProp
from OCP.BRepLProp import BRepLProp_SLProps
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism
from OCP.BRepTools import BRepTools
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Plane
from OCP.TopAbs import (TopAbs_EDGE, TopAbs_FACE, TopAbs_IN, TopAbs_ON, TopAbs_OUT,
                        TopAbs_REVERSED, TopAbs_SOLID, TopAbs_VERTEX, TopAbs_WIRE)
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt, gp_Trsf, gp_Vec

from cavity_cutter import DEFAULT_FUZZY_VALUES, bool_common, bool_cut, bool_fuse
from errors import PartingError
from geometry_utils import shape_bbox, shape_volume

EPS_NZ = 0.02          # 法向 z 分量判定阈值（≈88.9°）
SIL_TOL = 0.08         # 侧影点判定 |nz| < SIL_TOL


@dataclass
class PipeMoldResult:
    """管件模具生成结果。"""

    upper_mold: object
    lower_mold: object
    die_full: object                       # 完整型腔模（= 模芯 - 外形包络）
    core_size: Tuple[float, float, float]
    axis: int                              # 分模面变化所沿的轴（0=X, 1=Y）
    profile: List[Tuple[float, float]]     # 分模高度曲线 [(站位, h)]（水平面时两端点等高）
    parting_z_mean: float
    upper_volume: float
    lower_volume: float
    die_volume: float
    product_volume: float
    envelope_extra_volume: float           # 被"填实"的孔腔体积 = 模芯 - 型腔模 - 产品
    parting_kind: str = "plane"            # "plane" 水平面 / "silhouette" 侧影随形
    core_pin: object = None                # 内孔腔实体（= 芯棒，白捡的：挖包络时被剔除的那块）
    core_volume: float = 0.0
    demold: Dict[str, float] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)
    stats: Dict[str, object] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.warnings


# --------------------------------------------------------------------- 基础工具
def _count(shape, kind) -> int:
    exp = TopExp_Explorer(shape, kind)
    n = 0
    while exp.More():
        n += 1
        exp.Next()
    return n


def _solids(shape) -> List[object]:
    out = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(TopoDS.Solid_s(exp.Current()))
        exp.Next()
    return out


def _translate(shape, vec):
    tr = gp_Trsf()
    tr.SetTranslation(gp_Vec(*vec))
    return BRepBuilderAPI_Transform(shape, tr, True, False).Shape()


def sample_surface(shape, nuv: int = 40):
    """在 shape 所有面上做 UV 网格采样。

    返回 (points(N,3), normals(N,3), face_index(N,))，法向为实体外法向。
    """
    pts, nrms, fidx = [], [], []
    fi = -1
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        exp.Next()
        fi += 1
        try:
            ad = BRepAdaptor_Surface(face)
            u0, u1 = ad.FirstUParameter(), ad.LastUParameter()
            v0, v1 = ad.FirstVParameter(), ad.LastVParameter()
            if not all(np.isfinite(x) for x in (u0, u1, v0, v1)):
                continue
            rev = (face.Orientation() == TopAbs_REVERSED)
            for u in np.linspace(u0, u1, nuv):
                for v in np.linspace(v0, v1, nuv):
                    try:
                        pr = BRepLProp_SLProps(ad, float(u), float(v), 1, 1e-9)
                        if not pr.IsNormalDefined():
                            continue
                        p = pr.Value()
                        n = pr.Normal()
                        nx, ny, nz = n.X(), n.Y(), n.Z()
                        if rev:
                            nx, ny, nz = -nx, -ny, -nz
                        pts.append((p.X(), p.Y(), p.Z()))
                        nrms.append((nx, ny, nz))
                        fidx.append(fi)
                    except Exception:  # noqa: BLE001 —— 个别点失败不影响整体
                        continue
        except Exception:  # noqa: BLE001
            continue
    if not pts:
        raise PartingError("产品表面采样失败，无法计算侧影分模面")
    return np.asarray(pts), np.asarray(nrms), np.asarray(fidx, dtype=int)


def contact_face_flags(shape, die, pts=None, nrms=None, fidx=None,
                       probe: float = 0.25, n_probe: int = 9):
    """逐面判断"该面是否由模具成形"（贴模接触面）。

    在每个面上取若干采样点，沿其外法向偏移 probe 毫米：
    只要有一点落在模具实体内部，就认为该面由模具成形。
    管件内孔面（模具没有料）会被整面排除 —— 内孔不参与成形。
    逐面判断（每个面几个探针）既快又完整，避免逐点抽样漏点导致夹紧失效。
    返回 bool 数组（按面序号）。
    """
    if pts is None or nrms is None or fidx is None:
        pts, nrms, fidx = sample_surface(shape)
    n_faces = int(fidx.max()) + 1 if len(fidx) else 0
    flags = np.ones(n_faces, dtype=bool)
    if n_faces == 0:
        return flags
    cls = BRepClass3d_SolidClassifier(die)
    for fi in range(n_faces):
        idx = np.flatnonzero(fidx == fi)
        if len(idx) == 0:
            continue
        if len(idx) > n_probe:
            idx = idx[np.linspace(0, len(idx) - 1, n_probe).astype(int)]
        hit = False
        for i in idx:
            p = pts[i] + probe * nrms[i]
            try:
                cls.Perform(gp_Pnt(float(p[0]), float(p[1]), float(p[2])), 1e-6)
                if cls.State() == TopAbs_IN:
                    hit = True
                    break
            except Exception:  # noqa: BLE001
                hit = True
                break
        flags[fi] = hit
    return flags


def pick_parting_axis(bbox) -> int:
    """选择分模面变化所沿的水平长轴：包围盒较长的水平方向。"""
    lx = bbox[3] - bbox[0]
    ly = bbox[4] - bbox[1]
    return 0 if lx >= ly else 1


def outer_contact_flags(product, pts=None, nrms=None, fidx=None, n_probe: int = 3,
                        max_dist: float = 500.0, tol: float = 1e-6):
    """**不依赖布尔结果**判断"哪些面是模具真正成形到的表面"（= 位于产品外形之外）。

    判据（薄壁件很有效）：面的外法向**朝上**时，从面上一点沿 +Z 打一条射线；
    若前方还能打到产品材料，说明这个面处在产品的**腔体/内孔**里（上方还压着一层壁）
    → 不是成形面；打不到 → 该点在产品外形之外 → 是成形面。朝下的面同理沿 −Z。
    （逐点用实体分类器判"是否在材料里"在薄壁件上不可靠：壁内的点常被判成 ON；
      垂直射线求交则可以准确区分"内表面 vs 外表面"。）

    用 `IntCurvesFace_ShapeIntersector` 做精确的射线-形状求交，几十~几百次调用即可。

    用途：给"自动推荐分模面高度"用（不需要先跑一次 4~5 分钟的挖腔布尔）。
    实测在 757 mm 薄壁管件上与"用型腔模判定"的结果一致（成形面 37/52 个、朝上最低 z 相同）。
    """
    from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector

    if pts is None or nrms is None or fidx is None:
        pts, nrms, fidx = sample_surface(product)
    n_faces = int(fidx.max()) + 1 if len(fidx) else 0
    si = IntCurvesFace_ShapeIntersector()
    si.Load(product, tol)
    flags = np.ones(n_faces, dtype=bool)
    for fi in range(n_faces):
        idx = np.flatnonzero(fidx == fi)
        if len(idx) == 0:
            continue
        if len(idx) > n_probe:
            idx = idx[np.linspace(0, len(idx) - 1, n_probe).astype(int)]
        inner_votes = tot = n_fail = 0
        for i in idx:
            nz = float(nrms[i][2])
            if nz > 0.15:
                d = (0.0, 0.0, 1.0)
            elif nz < -0.15:
                d = (0.0, 0.0, -1.0)
            else:
                continue                     # 侧向面：不影响分模面高度，按成形面处理
            tot += 1
            try:
                si.Perform(gp_Lin(gp_Pnt(float(pts[i][0]), float(pts[i][1]),
                                         float(pts[i][2])), gp_Dir(*d)),
                           0.05, max_dist)
                if si.NbPnt() > 0:
                    inner_votes += 1
            except Exception:  # noqa: BLE001
                n_fail += 1
                continue
        if tot and n_fail == tot:
            raise PartingError(
                "射线求交全部失败（%d 个探针），无法判断面上/面下是否有料" % n_fail)
        flags[fi] = True if tot == 0 else (inner_votes < max(1, int(0.5 * tot)))
    return flags


def suggest_plane_height(aligned_shape, block_z=None, axis: Optional[int] = None,
                         pts=None, nrms=None, fidx=None, margin: float = 0.2,
                         verbose: bool = True) -> Dict[str, object]:
    """**快速给出"水平分模面高度"的推荐值 + 安全区间**（不跑任何布尔运算）。

    步骤：表面采样 → `outer_contact_flags` 找出成形面 → 侧影曲线 →
    `choose_plane_height` 取安全值；再结合模芯 Z 范围给出上/下模高度。

    block_z=(z0, z1) 给定时，还会返回：上模高 = z1 − z_p、下模高 = z_p − z0（合计 = 模芯高）。
    """
    t0 = time.perf_counter()
    if pts is None or nrms is None or fidx is None:
        pts, nrms, fidx = sample_surface(aligned_shape)
    if axis is None:
        axis = pick_parting_axis(shape_bbox(aligned_shape))
    flags = outer_contact_flags(aligned_shape, pts, nrms, fidx)
    contact = flags[np.clip(fidx, 0, len(flags) - 1)] if len(flags) else \
        np.ones(len(pts), dtype=bool)
    grid, hs, pts, nrms, sinfo = silhouette_profile(
        aligned_shape, axis=axis, pts=pts, nrms=nrms, fidx=fidx, contact=contact)
    z_p, pinfo = choose_plane_height(pts, nrms, fidx, contact, hs, grid,
                                     margin=margin, verbose=False)
    out: Dict[str, object] = {
        "axis": axis,
        "parting_z_auto": float(z_p),
        "safe_lo": float(pinfo["column_low_max"] + margin),
        "safe_hi": float(pinfo["up_facing_min"]),
        "silhouette_min": float(pinfo["z_from_silhouette_min"]),
        "up_facing_min": float(pinfo["up_facing_min"]),
        "down_facing_max": float(pinfo["down_facing_max"]),
        "column_low_max": float(pinfo["column_low_max"]),
        "safe_ok": bool(pinfo["flat_plane_ok"]),
        "contact_points": int(sinfo["contact_points"]),
        "sample_points": int(sinfo["sample_points"]),
        "product_z": [float(pts[:, 2].min()), float(pts[:, 2].max())],
        "elapsed_s": round(time.perf_counter() - t0, 2),
    }
    if block_z is not None:
        z0, z1 = float(block_z[0]), float(block_z[1])
        out.update({
            "block_z": [z0, z1],
            "block_height_mm": z1 - z0,
            "upper_height_auto_mm": z1 - float(z_p),
            "lower_height_auto_mm": float(z_p) - z0,
        })
    if verbose:
        print("  - 分模面推荐高度 z = %.2f mm（安全区间 %.2f ~ %.2f；侧影最低 %.2f）"
              % (z_p, out["safe_lo"], out["safe_hi"], out["silhouette_min"]))
        if block_z is not None:
            print("    对应上模高 %.2f mm + 下模高 %.2f mm = 模芯高 %.2f mm"
                  % (out["upper_height_auto_mm"], out["lower_height_auto_mm"],
                     out["block_height_mm"]))
        print("    （用 %d 个采样点、%d 个成形面点，耗时 %.1fs，未跑布尔）"
              % (out["sample_points"], out["contact_points"], out["elapsed_s"]))
    return out


def silhouette_profile(shape, axis: int = 0, nbins: int = 160, nuv: int = 40,
                       smooth: int = 5, die=None, pts=None, nrms=None, fidx=None,
                       contact=None):
    """求侧影分模高度曲线 h(轴站位)。

    每个站位取该站位内"法向水平(侧影点)"的 z 中位数；空站位用插值补齐。
    若给出 die（外形包络型腔）或用 contact 传入"贴模成形面"标记，还会把 h 夹在
    材料厚度区间内：
      * h ≤ min(朝上接触点的 z)  —— 否则上模在分模面同侧多出一层料（台阶）；
      * h ≥ max(朝下接触点的 z)  —— 否则下模多出一层料（仅统计，不参与夹紧）。
    返回 (xs, hs, pts, nrms, info)。
    """
    if pts is None or nrms is None or fidx is None:
        pts, nrms, fidx = sample_surface(shape, nuv=nuv)
    a = pts[:, axis]
    z = pts[:, 2]
    nz = nrms[:, 2]
    na = nrms[:, axis]
    lo, hi = float(a.min()), float(a.max())

    is_sil = (np.abs(nz) < SIL_TOL) & (np.abs(na) < 0.7)
    edges = np.linspace(lo, hi, nbins + 1)

    # 贴模接触点（模具真正成形到的产品表面；内孔面不算）
    if contact is None:
        if die is not None:
            flags = contact_face_flags(shape, die, pts=pts, nrms=nrms, fidx=fidx)
            contact = flags[np.clip(fidx, 0, len(flags) - 1)] if len(flags) else \
                np.ones(len(pts), dtype=bool)
        else:
            contact = np.ones(len(pts), dtype=bool)
    have_contact = die is not None or contact is not None

    xs, hs = [], []
    for k in range(nbins):
        m = (a >= edges[k]) & (a <= edges[k + 1])
        ms = m & is_sil
        if not ms.any():
            continue
        h = float(np.median(z[ms]))
        mc = m & contact
        if mc.any():
            upc = mc & (nz > 0.15)
            dnc = mc & (nz < -0.15)
            if upc.any():
                h = min(h, float(z[upc].min()))
            if dnc.any():
                h = max(h, float(z[dnc].max()))
        xs.append(0.5 * (edges[k] + edges[k + 1]))
        hs.append(h)
    if len(xs) < 2:
        raise PartingError(
            "未能在产品上找到侧影线（法向水平处），无法确定分模面；"
            "请确认模型为闭合实体且开模方向为 +Z")
    xs = np.asarray(xs)
    hs = np.asarray(hs)
    # 补齐到整段范围
    grid = np.linspace(lo, hi, 60)
    hs_g = np.interp(grid, xs, hs)
    if smooth > 1:
        k = smooth if smooth % 2 == 1 else smooth + 1
        pad = k // 2
        hs_g = np.convolve(np.pad(hs_g, pad, mode="edge"), np.ones(k) / k, mode="valid")
    # 夹紧：分模面必须**低于所有朝上的贴模接触面**，否则下模会在该面之上留一层料，
    # 产品沿 +Z 顶出时被挡住（凸台/台面场景的典型台阶）。
    # 反向（h 必须高于所有朝下面）**不参与夹紧**：管件末端斜切端面这类"朝下面在高处"
    # 属于真实倒扣，强抬高 h 反而会让下模长出钩料；这类情况只做统计与提示，
    # 最终由开模/顶出干涉检验判定。
    # 夹紧限值在邻域(±2 站位)内扩张，避免相邻站位之间残留薄台阶。
    conflicts = 0
    if have_contact:
        up_lim = np.full(len(grid), np.inf)
        dn_lim = np.full(len(grid), -np.inf)
        half = 0.5 * (grid[1] - grid[0])
        for i, gx in enumerate(grid):
            m = (a >= gx - half) & (a <= gx + half)
            mc = m & contact
            if not mc.any():
                continue
            upc = mc & (nz > 0.15)
            dnc = mc & (nz < -0.15)
            if upc.any():
                up_lim[i] = float(z[upc].min())
            if dnc.any():
                dn_lim[i] = float(z[dnc].max())
        w = 2
        up_lim = np.array([up_lim[max(0, i - w):i + w + 1].min() for i in range(len(grid))])
        dn_lim = np.array([dn_lim[max(0, i - w):i + w + 1].max() for i in range(len(grid))])
        conflicts = int((dn_lim > up_lim + 0.05).sum())
        hs_g = np.minimum(hs_g, up_lim)
    # 侧影法向自检：朝上/朝下的面是否被分模面正确分开（只看贴模接触点）
    nz_all = nrms[:, 2]
    h_at = np.interp(a, grid, hs_g)
    ca = contact
    up = (nz_all > 0.2) & ca
    dn = (nz_all < -0.2) & ca
    tol = 0.8
    info = {
        "sil_points": int(is_sil.sum()),
        "sample_points": int(len(pts)),
        "contact_points": int(contact.sum()),
        "clamp_conflicts": int(conflicts),
        "up_below_ratio": float(((up) & (z < h_at - tol)).sum() / max(up.sum(), 1)),
        "down_above_ratio": float(((dn) & (z > h_at + tol)).sum() / max(dn.sum(), 1)),
        "h_min": float(hs_g.min()), "h_max": float(hs_g.max()),
    }
    info["undercut_stations"] = int(conflicts)
    return grid, hs_g, pts, nrms, info


def choose_plane_height(pts, nrms, fidx, contact, hs_g, grid, margin: float = 0.2,
                        verbose: bool = True):
    """为**水平分模面**挑选一个安全高度 z_p（mm）。

    约束（都用"贴模接触点"统计，内孔面不算）:
      * 上界 `z_hi` = 所有**朝上**接触点的最低 z
        —— 分模面必须低于任何朝上的成形面，否则下模会在该面之上多留一层料，
           产品沿 +Z 顶出时被挡住；
      * 下界 `z_lo` = 所有铅垂列中"最低材料"的最大值（近似用整体最低点）
        —— 分模面必须高于产品任何"底面包围"，否则上模会在产品下方多留一层料。
    取 z_p = min(侧影曲线最低点 - margin, z_hi)，并保证 ≥ z_lo。

    返回 (z_p, info dict)。
    """
    z = pts[:, 2]
    nz = nrms[:, 2]
    up = contact & (nz > 0.15)
    dn = contact & (nz < -0.15)
    z_hi = float(z[up].min()) if up.any() else float("inf")
    z_dn_max = float(z[dn].max()) if dn.any() else -float("inf")

    # 逐列(5mm 网格)最低材料的最大值 —— 平底"台阶"约束
    cell = 5.0
    keys = np.floor(np.stack([pts[:, 0], pts[:, 1]], axis=1) / cell).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    low = np.full(len(uniq), np.inf)
    np.minimum.at(low, inv, z)
    z_lo = float(low.max())

    z_sil = float(np.min(hs_g))                 # 侧影曲线最低点 = 分型线自然高度
    z_p = min(z_sil - margin, z_hi)
    z_p = max(z_p, z_lo + margin)
    info = {
        "z_plane": z_p,
        "z_from_silhouette_min": z_sil,
        "up_facing_min": z_hi,
        "down_facing_max": z_dn_max,
        "column_low_max": z_lo,
        "flat_plane_ok": bool(z_lo <= z_hi),
    }
    if verbose:
        print("  - 水平分模面高度 z = %.2f mm（侧影最低 %.2f − 余量 %.2f；"
              "朝上接触面最低 %.2f；逐列最低材料最大 %.2f）"
              % (z_p, z_sil, margin, z_hi, z_lo))
        if not info["flat_plane_ok"]:
            print("    [提示] 该产品存在\"朝下面高于朝上面\"的区域（斜切端面/局部倒扣），"
                  "水平面无法完全避免，详见开模检验结果")
    return z_p, info


def _profile_prism(bbox, xs, hs, extra: float = 50.0, spline: bool = True):
    """构造"分模面以上"区域实体（XZ 剖面沿 Y 拉伸的棱柱；h 为常数时即水平面）。

    `spline=True` 时剖面用**插值样条**（而不是折线）—— 侧影随形分模面如果是折线拼的，
    型腔边缘就会出现"两个片面组成的一个角"，不贴合产品弧度（用户反馈的问题 3）。
    """
    x0, y0, z0, x1, y1, z1 = bbox
    ztop = z1 + extra
    hs = np.asarray(hs, dtype=float)
    use_spline = bool(spline) and len(xs) >= 3 and float(np.ptp(hs)) > 1e-6
    top_edge = None
    if use_spline:
        try:
            from OCP.GeomAPI import GeomAPI_Interpolate
            from OCP.TColgp import TColgp_HArray1OfPnt
            from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge

            # 插值点先**重采样到 ≤ 24 个**：160 个站位直接插值容易失败/抖动，
            # 落到折线兜底就会在型腔边缘留下一条条平面小面（用户反馈的"两片面成角"）。
            xs_a = np.asarray(xs, dtype=float)
            hs_a = np.asarray(hs, dtype=float)
            n_use = min(24, len(xs_a))
            if len(xs_a) > n_use:
                gx = np.linspace(float(xs_a[0]), float(xs_a[-1]), n_use)
                gh = np.interp(gx, xs_a, hs_a)
            else:
                gx, gh = xs_a, hs_a
            # 端点各外延一段，保证分模面覆盖到模芯之外
            px = [x0 - extra] + [float(v) for v in gx] + [x1 + extra]
            ph = [float(gh[0])] + [float(v) for v in gh] + [float(gh[-1])]
            arr = TColgp_HArray1OfPnt(1, len(px))
            for i, (xv, hv) in enumerate(zip(px, ph)):
                arr.SetValue(i + 1, gp_Pnt(float(xv), 0.0, float(hv)))
            interp = GeomAPI_Interpolate(arr, False, 1e-6)
            interp.Perform()
            if interp.IsDone():
                curve = interp.Curve()
                top_edge = BRepBuilderAPI_MakeEdge(curve).Edge()
        except Exception:  # noqa: BLE001 —— 样条失败就退回折线
            top_edge = None
    poly = BRepBuilderAPI_MakePolygon()
    if top_edge is not None:
        poly.Add(top_edge)
    else:
        poly.Add(gp_Pnt(x0 - extra, 0.0, float(hs[0])))
        for x, h in zip(xs, hs):
            poly.Add(gp_Pnt(float(x), 0.0, float(h)))
        poly.Add(gp_Pnt(x1 + extra, 0.0, float(hs[-1])))
    poly.Add(gp_Pnt(x1 + extra, 0.0, ztop))
    poly.Add(gp_Pnt(x0 - extra, 0.0, ztop))
    poly.Add(gp_Pnt(x0 - extra, 0.0, float(hs[0])))
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    prism = BRepPrimAPI_MakePrism(face, gp_Vec(0.0, (y1 - y0) + 2 * extra, 0.0)).Shape()
    return _translate(prism, (0.0, y0 - extra, 0.0))


def _face_normal(face, nuv: int = 5):
    """面中心附近的法向（首个有效采样）。"""
    try:
        ad = BRepAdaptor_Surface(face)
        u = 0.5 * (ad.FirstUParameter() + ad.LastUParameter())
        v = 0.5 * (ad.FirstVParameter() + ad.LastVParameter())
        pr = BRepLProp_SLProps(ad, u, v, 1, 1e-9)
        if not pr.IsNormalDefined():
            return None
        n = pr.Normal()
        v3 = np.array([n.X(), n.Y(), n.Z()])
        if face.Orientation() == TopAbs_REVERSED:
            v3 = -v3
        return v3
    except Exception:  # noqa: BLE001
        return None


AXIS_VEC = {0: (1.0, 0.0, 0.0), 1: (0.0, 1.0, 0.0)}
AXIS_NAME = {0: "x", 1: "y"}


def section_wires_plane(shape, origin, normal, tol: float = 1e-4):
    """求 shape 与**任意平面**（过 origin、法向 normal）的交线，组装成闭合 wire。

    返回 [(wire, 面积)]（按面积降序）。
    """
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
    from OCP.gp import gp_Dir as _D, gp_Pln as _P
    from parting_splitter import _assemble_wires
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace as _MF

    pln = _P(gp_Pnt(float(origin[0]), float(origin[1]), float(origin[2])),
             _D(float(normal[0]), float(normal[1]), float(normal[2])))
    sec = BRepAlgoAPI_Section(shape, pln)
    sec.Approximation(False)
    sec.Build()
    if not sec.IsDone():
        return []
    edges = []
    exp = TopExp_Explorer(sec.Shape(), TopAbs_EDGE)
    while exp.More():
        edges.append(TopoDS.Edge_s(exp.Current()))
        exp.Next()
    out = []
    for w in _assemble_wires(edges, tol=tol):
        try:
            f = _MF(w).Face()
            g = GProp_GProps()
            BRepGProp.SurfaceProperties_s(f, g)
            out.append((w, float(g.Mass())))
        except Exception:  # noqa: BLE001
            continue
    out.sort(key=lambda t: t[1], reverse=True)
    return out


def section_wires(shape, axis: int, pos: float, tol: float = 1e-4):
    """求 shape 与"垂直于 axis、位置 pos"平面的交线，组装成闭合 wire。

    返回 [(wire, 面积)]（按面积降序的外/内轮廓候选）。
    """
    pnt = [0.0, 0.0, 0.0]
    pnt[axis] = pos
    return section_wires_plane(shape, pnt, AXIS_VEC[axis], tol=tol)


def _wire_area(wire) -> float:
    try:
        f = BRepBuilderAPI_MakeFace(wire).Face()
        g = GProp_GProps()
        BRepGProp.SurfaceProperties_s(f, g)
        return float(g.Mass())
    except Exception:  # noqa: BLE001
        return -1.0


def offset_wire_outward(wire, delta: float):
    """把平面闭合 wire 向外偏置 delta（取面积更大的一侧）；失败返回 None。"""
    try:
        from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffset
        best = None
        best_area = _wire_area(wire)
        for d in (delta, -delta):
            try:
                mk = BRepOffsetAPI_MakeOffset()
                mk.AddWire(wire)
                mk.Perform(d)
                if not mk.IsDone():
                    continue
                shp = mk.Shape()
                wexp = TopExp_Explorer(shp, TopAbs_WIRE)
                while wexp.More():
                    w = TopoDS.Wire_s(wexp.Current())
                    wexp.Next()
                    if not w.Closed:
                        continue
                    a = _wire_area(w)
                    if a > best_area:
                        best, best_area = w, a
            except Exception:  # noqa: BLE001
                continue
        return best
    except Exception:  # noqa: BLE001
        return None


def _wire_center(wire):
    """wire 顶点平均位置（近似孔口中心）。"""
    pts = []
    exp = TopExp_Explorer(wire, TopAbs_VERTEX)
    while exp.More():
        try:
            pts.append(BRep_Tool.Pnt_s(TopoDS.Vertex_s(exp.Current())))
        except Exception:  # noqa: BLE001
            pass
        exp.Next()
    if not pts:
        return None
    return (sum(p.X() for p in pts) / len(pts),
            sum(p.Y() for p in pts) / len(pts),
            sum(p.Z() for p in pts) / len(pts))


def _is_open_hole(shape, wire, face_normal, probe: float = 0.6):
    """判断 wire 是不是**真实孔口**（而不是凸台/圆柱与面的接缝内环）。

    做法：孔口中心沿"进入产品"方向偏移 probe 毫米，若该点在产品实体**外面**，
    说明这里是通的（真孔）；若在实体里面，说明只是两块料的分界缝，不能封盖。
    """
    if face_normal is None:
        return False
    c = _wire_center(wire)
    if c is None:
        return False
    p = gp_Pnt(c[0] - face_normal[0] * probe,
               c[1] - face_normal[1] * probe,
               c[2] - face_normal[2] * probe)
    try:
        cls = BRepClass3d_SolidClassifier(shape)
        cls.Perform(p, 1e-6)
        return cls.State() == TopAbs_OUT
    except Exception:  # noqa: BLE001
        return False


def hole_sealing_plates(shape, axis: int = 0, bbox=None, depth: float = 1.0,
                        insets=(0.2, 0.5, 0.9, 1.4, 2.0, 2.8, 3.8, 5.0, 6.5, 8.5, 11.0, 15.0),
                        grow: float = 0.4, max_plates: int = 32, pts=None):
    """生成"切断内芯条"用的封盖薄片。

    两条途径（互补）:
      1. **长轴两端截面上的孔轮廓**（非最大环）—— 管件内孔开口在两端，最可靠；
         站位从**真实几何端面**向内自适应搜索（不能用 BRepBndLib 的包围盒端面：
         BSpline 控制点会把包围盒撑大几毫米到几十毫米，实测某件虚高 36.6 / 14.5 mm），
         并把孔轮廓向外偏置 grow 毫米，保证完整覆盖孔口、切断内芯条；
      2. 面上内环（孔口）—— 覆盖侧面开孔、端面被切成多块的情况。

    `pts` 可传入已有的表面采样点（用于取"真实几何"的轴向极值，省一次采样）。

    返回 (compound 或 None, 孔口数, 失败数, 来源统计)。
    """
    if bbox is None:
        bbox = shape_bbox(shape)
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    n_holes = 0
    n_fail = 0
    n_section = 0
    n_skipped = 0
    used_stations: List[float] = []      # 途径 1 已经封过的轴向站位（避免途径 2 重复封）

    # ---- 途径 1：两端截面上的孔 ----
    # 搜索起点必须用**真实几何**的长轴极值，不能用 BRepBndLib 的包围盒端面：
    # BSpline 控制点会把包围盒撑大（实测某件虚高 36.6 / 14.5 mm），原来那些 0.5~6 mm 的
    # inset 全落在"产品外面的空气"里 → 一个端口截面都取不到 → 封盖全靠途径 2，
    # 一旦途径 2 的封盖偏小，内孔腔就切不断（模具直接做错）。
    if pts is None:
        try:
            pts, _n, _f = sample_surface(shape, nuv=16)
        except Exception:  # noqa: BLE001
            pts = None
    if pts is not None and len(pts):
        a0 = float(pts[:, axis].min())
        a1 = float(pts[:, axis].max())
    else:
        a0, a1 = bbox[axis], bbox[axis + 3]
    span = a1 - a0
    av = AXIS_VEC[axis]
    for end, a_end in ((-1, a0), (+1, a1)):
        if span <= 0:
            continue
        found = None
        for inset in insets:
            if 2.0 * inset >= span:
                break
            a_s = a_end - end * inset          # 由端面向内搜索（向内 = -end）
            wires = section_wires(shape, axis, a_s)
            if len(wires) >= 2:
                found = (a_s, wires)
                break
        if found is None:
            continue
        a_s, wires = found
        for wire, _area in wires[1:max_plates + 1]:
            n_holes += 1
            try:
                w_use = offset_wire_outward(wire, grow) or wire
                cap = BRepBuilderAPI_MakeFace(w_use).Face()
                if cap.IsNull():
                    n_fail += 1
                    continue
                # 从该截面沿轴向一直延伸到端面之外，切断内芯条与主体的连接
                a_target = a_end + end * depth
                length = abs(a_target - a_s)
                vec = gp_Vec(end * av[0] * length, end * av[1] * length, 0.0)
                plate = BRepPrimAPI_MakePrism(cap, vec).Shape()
                if plate.IsNull():
                    n_fail += 1
                    continue
                builder.Add(comp, plate)
                n_section += 1
                used_stations.append(float(a_s))
            except Exception:  # noqa: BLE001
                n_fail += 1
                continue

    # ---- 途径 2：面上内环（孔口）----
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        exp.Next()
        if n_holes >= max_plates:
            break
        try:
            outer = BRepTools.OuterWire_s(face)
        except Exception:  # noqa: BLE001
            continue
        n = _face_normal(face)
        wexp = TopExp_Explorer(face, TopAbs_WIRE)
        while wexp.More():
            wire = TopoDS.Wire_s(wexp.Current())
            wexp.Next()
            if wire.IsSame(outer):
                continue
            n_holes += 1
            if n_holes > max_plates:
                break
            if not _is_open_hole(shape, wire, n):
                n_skipped += 1
                n_holes -= 1
                continue
            # 同一个端口如果已经被"途径 1（端面截面）"封过，这里就跳过：
            # 两片封盖叠在一起会让布尔运算又慢又危险（实测一件卡了 17 分钟没出结果）。
            c_w = _wire_center(wire)
            if c_w is not None and any(abs(c_w[axis] - s) < 3.0 for s in used_stations):
                n_skipped += 1
                n_holes -= 1
                continue
            try:
                w_use = offset_wire_outward(wire, grow) or wire
                plate = _planar_hole_plate(w_use, n, depth)
                if plate is None:
                    n_fail += 1
                    continue
                builder.Add(comp, plate)
            except Exception:  # noqa: BLE001 —— 个别孔口失败不阻塞
                n_fail += 1
                continue

    if n_holes == 0:
        return None, 0, 0, {"section": 0, "inner_wire": 0, "skipped": n_skipped}
    return comp, n_holes, n_fail, {"section": n_section,
                                   "inner_wire": n_holes - n_section,
                                   "skipped": n_skipped}


def _planar_hole_plate(wire, normal, depth: float, grow: float = 0.0):
    """把"面上的孔口内环"做成一块**平面矩形**薄片，沿 -法向 拉伸 depth。

    为什么不直接用 `BRepBuilderAPI_MakeFace(wire)`：曲面上的内环（例如 BSpline 面上的孔口）
    建出来的面可能不是平面/自身有毛病，做成棱柱去切布尔会留下病态几何 —— 实测某管件
    因此导致**下模没法作为实体写进 STEP**（STEP 里被降级成 shell，实体丢失）。
    这里改用"孔口包围盒 → 平面矩形 → 拉成棱柱"，几何简单、一定是平面，布尔结果干净。
    """
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon
    from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism

    if normal is None:
        normal = (0.0, 0.0, -1.0)
    nvec = np.asarray([float(v) for v in normal], dtype=float)
    ln = float(np.linalg.norm(nvec))
    if ln < 1e-12:
        nvec = np.asarray([0.0, 0.0, -1.0])
    else:
        nvec = nvec / ln
    pts = []
    # 沿 wire 的**每条边离散采样**，不能只取顶点：曲边环（例如 2~4 条大圆弧组成的孔口）
    # 的顶点严重低估实际范围 —— 实测封盖做小了 40%，切不断内孔腔、模具直接做错。
    exp = TopExp_Explorer(wire, TopAbs_EDGE)
    while exp.More():
        try:
            c = BRepAdaptor_Curve(TopoDS.Edge_s(exp.Current()))
            t0, t1 = c.FirstParameter(), c.LastParameter()
            if np.isfinite(t0) and np.isfinite(t1) and t1 > t0:
                for t in np.linspace(t0, t1, 13):
                    p = c.Value(float(t))
                    pts.append([p.X(), p.Y(), p.Z()])
        except Exception:  # noqa: BLE001
            pass
        exp.Next()
    vexp = TopExp_Explorer(wire, TopAbs_VERTEX)
    while vexp.More():
        try:
            p = BRep_Tool.Pnt_s(TopoDS.Vertex_s(vexp.Current()))
            pts.append([p.X(), p.Y(), p.Z()])
        except Exception:  # noqa: BLE001
            pass
        vexp.Next()
    if len(pts) < 3:
        return None
    P = np.asarray(pts)
    c = P.mean(axis=0)
    # 在法向的正交平面里取两个方向
    ref = np.asarray([1.0, 0.0, 0.0]) if abs(nvec[0]) < 0.9 else np.asarray([0.0, 1.0, 0.0])
    u = np.cross(nvec, ref)
    u = u / max(float(np.linalg.norm(u)), 1e-12)
    v = np.cross(nvec, u)
    du = float(np.abs((P - c) @ u).max()) + grow
    dv = float(np.abs((P - c) @ v).max()) + grow
    corners = [c + du * u + dv * v, c - du * u + dv * v,
               c - du * u - dv * v, c + du * u - dv * v]
    poly = BRepBuilderAPI_MakePolygon()
    for q in corners:
        poly.Add(gp_Pnt(float(q[0]), float(q[1]), float(q[2])))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    if face.IsNull():
        return None
    vec = gp_Vec(-nvec[0] * depth, -nvec[1] * depth, -nvec[2] * depth)
    plate = BRepPrimAPI_MakePrism(face, vec).Shape()
    return None if plate.IsNull() else plate


PORT_INSETS = (0.2, 0.5, 0.9, 1.4, 2.0, 2.8, 3.8, 5.0, 8.0, 12.0, 18.0, 26.0, 36.0)


def find_port_section(shape, axis: int, end: int, a_end: float, insets=PORT_INSETS):
    """从长轴端面向内搜一个"带孔的截面"站位（端口附近一定有），返回 (站位, 环表)。

    `a_end` 必须是**产品真实端面**位置，不能用 BRepBndLib 包围盒端面：BSpline 控制点
    会把包围盒撑大几毫米到几十毫米，那些 inset 全落在空气里，一个截面都取不到。
    """
    for inset in insets:
        a_s = a_end - end * inset
        try:
            ws = section_wires(shape, axis, float(a_s))
        except Exception:  # noqa: BLE001
            continue
        if len(ws) >= 2:
            return float(a_s), ws
    return None


def _ray_hits(shape, origin, direction, tmax: float = 1e4, tol: float = 1e-6):
    """射线 × 形状求交，返回按参数升序的 [(t, gp_Pnt, TopoDS_Face)]。"""
    from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector

    o = np.asarray(origin, dtype=float)
    d = np.asarray(direction, dtype=float)
    ln = float(np.linalg.norm(d))
    if ln < 1e-12:
        return []
    d = d / ln
    si = IntCurvesFace_ShapeIntersector()
    si.Load(shape, tol)
    si.Perform(gp_Lin(gp_Pnt(float(o[0]), float(o[1]), float(o[2])),
                      gp_Dir(float(d[0]), float(d[1]), float(d[2]))), tol, float(tmax))
    out = []
    for i in range(1, si.NbPnt() + 1):
        p = si.Pnt(i)
        t = (np.array([p.X(), p.Y(), p.Z()]) - o) @ d
        out.append((float(t), p, si.Face(i)))
    out.sort(key=lambda r: r[0])
    return out


def port_cut_planes(shape, axis: int, pts=None, verbose: bool = False):
    """求产品两端的**端口平面**（用于截断块法），返回 [(面上一点, 朝外法向), ...] 或 None。

    对**斜切端口 / 弯管末端**同样有效（这时不能用"垂直长轴的平面"去截——那会在斜口处
    留下一小片管壁把内芯条和主体连起来）：

      1. 从长轴端面向内搜一个"带孔的截面"，取内孔环中心 `c`（在**内孔腔里**）；
      2. 从 `c` 沿垂直长轴方向打射线 → 第 1、2 个交点就是管壁的**内、外表面** → 取中点 `m`；
      3. 从 `m` 沿**局部轴向**（前后两个截面的内孔中心连线）向外打射线 → 命中的第一个面
         就是**端口端面**（内孔是开口，射线不会从孔里穿出去）；
      4. 端口平面就取该端面所在平面；端面不是平面面时退回"过交点、垂直局部轴向"的平面。
    """

    def _say(msg):
        if verbose:
            print("    · 端口平面搜索: " + msg, flush=True)

    bb = shape_bbox(shape)
    if pts is None:
        try:
            pts, _n, _f = sample_surface(shape, nuv=16)
        except Exception:  # noqa: BLE001
            pts = None
    if pts is None or not len(pts):
        _say("没有表面采样点")
        return None
    P = np.asarray(pts, dtype=float)
    s0, s1 = float(P[:, axis].min()), float(P[:, axis].max())
    if s1 - s0 < 1e-6:
        _say("长轴范围退化")
        return None
    out = []
    for end, a_end in ((-1, s0), (+1, s1)):
        got = find_port_section(shape, axis, end, a_end)
        if got is None:
            _say("端%+d 从 %.2f 向内找不到带孔截面" % (end, a_end))
            return None
        a_s, wires = got
        holes = [w for w, _ar in wires[1:]]
        if not holes:
            _say("端%+d 截面 %.2f 里没有内孔环" % (end, a_s))
            return None
        c = _wire_center(max(holes, key=_wire_area))
        if c is None:
            _say("端%+d 内孔环中心取不到" % end)
            return None
        c = np.asarray(c, dtype=float)
        # 局部轴向：再往里 5~10 mm 取一个截面，用两截面内孔中心差
        d = None
        got2 = find_port_section(shape, axis, end, a_end,
                                 (abs(a_end - a_s) + 6.0,
                                  abs(a_end - a_s) + 12.0))
        if got2 is not None:
            h2 = [w for w, _ar in got2[1][1:]]
            c2 = _wire_center(max(h2, key=_wire_area)) if h2 else None
            if c2 is not None:
                v = c - np.asarray(c2, dtype=float)
                if float(np.linalg.norm(v)) > 1e-6:
                    d = v / float(np.linalg.norm(v))
        if d is None:
            d = np.zeros(3)
            d[axis] = float(end)
        # 管壁内的点 m：从内孔中心垂直长轴打射线，取第 1、2 个交点（内/外表面）的中点
        u = np.zeros(3)
        u[(axis + 1) % 3] = 1.0
        hits = _ray_hits(shape, c, u)
        if len(hits) < 2:
            u = np.zeros(3)
            u[(axis + 2) % 3] = 1.0
            hits = _ray_hits(shape, c, u)
        if len(hits) < 2:
            _say("端%+d 从内孔中心 %.2f 垂直打射线只命中 %d 次" % (end, a_s, len(hits)))
            return None
        m = 0.5 * (np.array([hits[0][1].X(), hits[0][1].Y(), hits[0][1].Z()]) +
                   np.array([hits[1][1].X(), hits[1][1].Y(), hits[1][1].Z()]))
        # 从管壁沿局部轴向外打，命中的第一个面就是端口端面
        axial = _ray_hits(shape, m, d, tmax=2.0 * (s1 - s0) + 10.0)
        if not axial:
            _say("端%+d 从管壁 %s 沿轴向 %s 打射线无命中" %
                 (end, np.round(m, 2), np.round(d, 3)))
            return None
        _t_hit, p_hit, face_hit = axial[0]
        p = np.array([p_hit.X(), p_hit.Y(), p_hit.Z()])
        n = None
        try:
            n = _face_normal(TopoDS.Face_s(face_hit))
        except Exception:  # noqa: BLE001
            n = None
        if n is None:
            n = d
        n = np.asarray(n, dtype=float)
        n = n / max(float(np.linalg.norm(n)), 1e-12)
        if float(n @ d) < 0.0:               # 与朝外方向相反就翻过来
            n = -n
        # 必须是"支持平面"：材料基本都在法向内侧（否则射线打到的不是端口面）。
        # 容差给 1 mm：面的 (u,v) 参数矩形包含被裁掉的区域，采样点可能落到真实几何之外
        # （本件实测 0.30 mm）—— 那只是采样虚高，端口平面本身仍是精确的；
        # 而打错面（例如打到外圆柱面）时外伸会是几十毫米，照样会被否掉。
        over = float(((P - p) @ n).max())
        if over > 1.0:
            over2 = float(((P - p) @ d).max())
            _say("端%+d 命中面 %.2f 不是支持平面（外伸 %.2f，按轴向 %.2f）"
                 % (end, np.array([p_hit.X(), p_hit.Y(), p_hit.Z()])[axis], over, over2))
            if over2 > 1.0:
                return None
            n = d
            p = p.copy()
            over = over2
        if verbose:
            print("    · 端%+d: 截面 %.2f 内孔中心 %s → 端口面点 %s 法向 %s（外伸 %.3f）"
                  % (end, a_s, np.round(c, 1), np.round(p, 2), np.round(n, 3), over),
                  flush=True)
        out.append((p, n))
    return out if len(out) == 2 else None


def port_corks(shape, axis: int, pts, grow: float = 0.0, beyond: float = 0.5,
               verbose: bool = False):
    """用"端口截面内孔轮廓"沿**局部轴向**做两个短塞子（棱柱），把内孔腔在端口处切断。

    为什么还要这一手：端口端面是**曲面/斜切面**（弯管上翘端常见）时，平面切不准 ——
    切浅了管壁还连着（内芯条不断开），切深了模具端面会多出一圈台阶。
    这里直接用截面上的内孔轮廓（平面环，`MakeFace` 一定干净），沿局部轴向（前后两个
    截面的内孔中心连线）拉伸到端口之外 `beyond` 毫米：既切得断，又只吃掉 beyond 毫米，
    芯棒端面只缩进 0.2 毫米（模具端面留一个很浅的坑，可忽略）。

    返回 (compound, info) 或 (None, info)。
    """
    bb = shape_bbox(shape)
    if pts is None:
        try:
            pts, _n, _f = sample_surface(shape, nuv=16)
        except Exception:  # noqa: BLE001
            pts = None
    if pts is None or not len(pts):
        return None, {}
    P = np.asarray(pts, dtype=float)
    s0, s1 = float(P[:, axis].min()), float(P[:, axis].max())
    if s1 - s0 < 1e-6:
        return None, {}
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    made = 0
    info: Dict[str, object] = {}
    for end, a_end in ((-1, s0), (+1, s1)):
        got = find_port_section(shape, axis, end, a_end)
        if got is None:
            return None, {"reason": "端%+d 找不到带孔截面" % end}
        a_s, wires = got
        holes = [w for w, _ar in wires[1:]]
        if not holes:
            return None, {"reason": "端%+d 截面没有内孔环" % end}
        rim = max(holes, key=_wire_area)
        c = _wire_center(rim)
        if c is None:
            return None, {"reason": "端%+d 内孔环中心取不到" % end}
        c = np.asarray(c, dtype=float)
        d = None
        got2 = find_port_section(shape, axis, end, a_end,
                                 (abs(a_end - a_s) + 6.0, abs(a_end - a_s) + 12.0))
        if got2 is not None:
            h2 = [w for w, _ar in got2[1][1:]]
            c2 = _wire_center(max(h2, key=_wire_area)) if h2 else None
            if c2 is not None:
                v = c - np.asarray(c2, dtype=float)
                if float(np.linalg.norm(v)) > 1e-6:
                    d = v / float(np.linalg.norm(v))
        if d is None:
            d = np.zeros(3)
            d[axis] = float(end)
        try:
            w_use = offset_wire_outward(rim, grow) or rim if grow else rim
            face = BRepBuilderAPI_MakeFace(w_use).Face()
            if face.IsNull():
                return None, {"reason": "端%+d 内孔轮廓建面失败" % end}
        except Exception as exc:  # noqa: BLE001
            return None, {"reason": "端%+d 内孔轮廓异常(%s)" % (end, type(exc).__name__)}
        length = float(((P - c) @ d).max()) + float(beyond)
        if length <= 0.0:
            return None, {"reason": "端%+d 拉伸长度异常" % end}
        try:
            prism = BRepPrimAPI_MakePrism(
                face, gp_Vec(float(d[0] * length), float(d[1] * length),
                             float(d[2] * length))).Shape()
        except Exception as exc:  # noqa: BLE001
            return None, {"reason": "端%+d 塞子拉伸失败(%s)" % (end, type(exc).__name__)}
        if prism.IsNull():
            return None, {"reason": "端%+d 塞子为空" % end}
        builder.Add(comp, prism)
        made += 1
        info["end%+d" % end] = (round(a_s, 2), round(length, 2))
        if verbose:
            print("    · 塞子 端%+d: 截面 %.2f 内孔中心 %s 轴向 %s 长 %.2f mm"
                  % (end, a_s, np.round(c, 1), np.round(d, 3), length), flush=True)
    return (comp if made == 2 else None), info


def _robust_plane_fit(P):
    """对一组点做稳健平面拟合（拟合 → 剔除 >3σ → 再拟合），返回 (重心, 单位法向)。"""
    Q = np.asarray(P, dtype=float)
    if len(Q) < 3:
        return None
    c = Q.mean(axis=0)
    n = None
    for _ in range(3):
        A = Q - c
        try:
            _u, _s, vt = np.linalg.svd(A, full_matrices=False)
        except Exception:  # noqa: BLE001
            return None
        n = np.asarray(vt[-1], dtype=float)
        n = n / max(float(np.linalg.norm(n)), 1e-12)
        res = np.abs(A @ n)
        keep = res <= max(3.0 * float(res.std()), 1e-6) + 1e-9
        if keep.sum() < 3 or keep.all():
            break
        Q = Q[keep]
        c = Q.mean(axis=0)
    return c, n


def _port_rim_from_face(face, axis_pt, axis_dir):
    """从**端口端面**上取出"内孔开口"那圈边，组装成 wire。

    端口开口的边界本来就存在于 BRep 里（端面与管内壁共享的边），直接取它最准 ——
    用平面去切、或用内侧截面去近似，遇到**斜切端口/曲面端面**都会偏（实测偏 2.2 mm，
    盖片就切在端口外面了，内芯条还是连着）。

    做法：把端面的边**整体组装成环**（一般是外轮廓 + 内孔口两圈），取"围成面积小"的那圈
    = 内孔开口。不能按"中点到轴线的距离"挑边：斜切口的半径沿环变化，挑出来的边不闭合。
    """
    from parting_splitter import _assemble_wires

    edges = []
    try:
        exp = TopExp_Explorer(face, TopAbs_EDGE)
        while exp.More():
            edges.append(TopoDS.Edge_s(exp.Current()))
            exp.Next()
    except Exception:  # noqa: BLE001
        return None
    if len(edges) < 2:
        return None
    best = None
    for tol in (1e-4, 1e-3, 1e-2):
        try:
            ws = _assemble_wires(edges, tol=tol)
        except Exception:  # noqa: BLE001
            continue
        for w in ws:
            try:
                a = _wire_area(w)
            except Exception:  # noqa: BLE001
                continue
            if a > 1e-6 and (best is None or a < best[0]):
                best = (a, w)
        if best is not None:
            break
    return None if best is None else best[1]


def _face_index_of(shape, face) -> Optional[int]:
    """面在 shape 中的序号（用来从采样点里挑出这个面的点）。"""
    try:
        target = TopoDS.Face_s(face)
    except Exception:  # noqa: BLE001
        return None
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    i = -1
    while exp.More():
        i += 1
        cur = TopoDS.Face_s(exp.Current())
        exp.Next()
        if cur.IsSame(target):
            return i
    return None


def _probe_port(shape, axis: int, end: int, a_end: float, pts, fidx=None,
                verbose: bool = False):
    """探测一个端口：返回 dict（截面站位、内孔中心、局部轴向、端口平面、内孔口环）。

    端口平面 = 端口端面所在平面（对**斜切面/曲面端面**用该面采样点做稳健平面拟合），
    内孔口环 = 用该平面切产品得到的内轮廓（= 端口开口的边界）。
    """
    got = find_port_section(shape, axis, end, a_end)
    if got is None:
        return None
    a_s, wires = got
    holes = [w for w, _ar in wires[1:]]
    if not holes:
        return None
    rim_s = max(holes, key=_wire_area)
    c = _wire_center(rim_s)
    if c is None:
        return None
    c = np.asarray(c, dtype=float)
    # 局部轴向：再往里 6/12 mm 取一个截面，用两个截面内孔中心差
    d = None
    got2 = find_port_section(shape, axis, end, a_end,
                             (abs(a_end - a_s) + 6.0, abs(a_end - a_s) + 12.0))
    if got2 is not None:
        h2 = [w for w, _ar in got2[1][1:]]
        c2 = _wire_center(max(h2, key=_wire_area)) if h2 else None
        if c2 is not None:
            v = c - np.asarray(c2, dtype=float)
            if float(np.linalg.norm(v)) > 1e-6:
                d = v / float(np.linalg.norm(v))
    if d is None:
        d = np.zeros(3)
        d[axis] = float(end)
    info = {"station": float(a_s), "center": c, "axis_dir": d, "rim_station": rim_s}
    # 管壁内的点 m：从内孔中心垂直长轴打射线，取第 1、2 个交点（内/外表面）的中点
    u = np.zeros(3)
    u[(axis + 1) % 3] = 1.0
    hits = _ray_hits(shape, c, u)
    if len(hits) < 2:
        u = np.zeros(3)
        u[(axis + 2) % 3] = 1.0
        hits = _ray_hits(shape, c, u)
    if len(hits) < 2:
        return None
    p1 = np.array([hits[0][1].X(), hits[0][1].Y(), hits[0][1].Z()])
    p2 = np.array([hits[1][1].X(), hits[1][1].Y(), hits[1][1].Z()])
    m = 0.5 * (p1 + p2)
    # 探测点要**贴着内表面**（靠内孔那一侧）：管件端口常有倒角/倒棱面，
    # 从管壁中间打射线会先打到倒棱面（很小的面），取到的"口环"就小得离谱
    m_in = p1 + 0.2 * (p2 - p1)
    # 沿局部轴向外打，第一个面 = 端口端面
    P = np.asarray(pts, dtype=float)
    axial = _ray_hits(shape, m_in, d, tmax=2.0 * (float(P[:, axis].max()) -
                                                 float(P[:, axis].min())) + 10.0)
    if not axial:
        return None
    _t, p_hit, face_hit = axial[0]
    p_hit = np.array([p_hit.X(), p_hit.Y(), p_hit.Z()])
    # 端口平面：优先用命中面的采样点做稳健拟合（斜切/曲面端面也准）
    plane = None
    if fidx is not None and len(fidx) == len(P):
        ff = None
        try:
            fexp = TopExp_Explorer(shape, TopAbs_FACE)
            i = -1
            while fexp.More():
                i += 1
                if TopoDS.Face_s(fexp.Current()).IsSame(TopoDS.Face_s(face_hit)):
                    ff = i
                    break
                fexp.Next()
        except Exception:  # noqa: BLE001
            ff = None
        if ff is not None:
            sel = P[fidx == ff]
            if len(sel) >= 6:
                plane = _robust_plane_fit(sel)
    if plane is None:
        plane = (p_hit, d)
    c_p, n_p = plane
    n_p = np.asarray(n_p, dtype=float)
    n_p = n_p / max(float(np.linalg.norm(n_p)), 1e-12)
    if float(n_p @ d) < 0.0:
        n_p = -n_p
    info["plane"] = (c_p, n_p)
    info["hit"] = p_hit
    # 内孔口环：**首选直接从端口端面上取**（端面与管内壁共享的那圈边，斜切口也精确）
    rim = None
    try:
        rim = _port_rim_from_face(TopoDS.Face_s(face_hit), c, d)
    except Exception:  # noqa: BLE001
        rim = None
    if rim is not None:
        info["rim_from"] = "port_face_edges"
    # 退一步：用候选平面切产品，取内轮廓
    if rim is None:
        cands = []
        if c_p is not None and n_p is not None:
            cands.append((c_p, n_p))
        cands.append((p_hit, d))
        axn = np.zeros(3)
        axn[axis] = float(end)
        cands.append((p_hit, axn))
        for o, nv in cands:
            try:
                ws = section_wires_plane(shape, o, nv)
            except Exception:  # noqa: BLE001
                continue
            if len(ws) >= 2:
                rim = max([w for w, _ar in ws[1:]], key=_wire_area)
                info["rim_from"] = "plane_section"
                break
    if rim is None:
        # 再退一步：把"截面内孔环"沿端口法向平移到端口平面上（形状只差 0.5 mm 的锥度）
        try:
            delta = float((np.asarray(c_p, float) - c) @ n_p)
            rim = TopoDS.Wire_s(_translate(
                rim_s, tuple(float(n_p[i] * delta) for i in range(3))))
            info["rim_from"] = "station_shifted"
        except Exception:  # noqa: BLE001
            rim = rim_s
            info["rim_from"] = "station"
    info["rim"] = rim
    if verbose:
        print("    · 端口探测 端%+d: 截面 %.2f 内孔中心 %s 轴向 %s 端口面点 %s 法向 %s 口环来源 %s"
              % (end, a_s, np.round(c, 1), np.round(d, 3), np.round(p_hit, 2),
                 np.round(n_p, 3), info["rim_from"]), flush=True)
    return info


def port_cap_slabs(shape, axis: int, pts, fidx=None, thickness: float = 0.5,
                   verbose: bool = False):
    """沿**端口平面**贴着端口切一层 `thickness` 毫米厚的"盖片"，把内孔腔与主体切断。

    为什么这是最稳的一手：盖片的形状 = 端口开口（用端口平面切产品得到的内轮廓），
    位置 = 端口端面所在平面（斜切面/曲面端面用采样点稳健拟合），沿该平面法向外拉
    `thickness` 毫米。于是：

    * **芯棒端面 = 端口开口本身**（不再被"垂直于长轴的平面"斜切一刀，也不会少一块楔形）；
    * 模具端面只在开口正对的位置被啃掉 `thickness` 毫米（很浅，且是平面，不会留下尖角/薄刃）；
    * 内芯条与主体之间那条"连通缝"正好落在这层里，被整层切掉 → 一次布尔就断开。

    返回 (compound, info) 或 (None, info)。
    """
    if pts is None:
        return None, {"reason": "没有表面采样点"}
    P = np.asarray(pts, dtype=float)
    s0, s1 = float(P[:, axis].min()), float(P[:, axis].max())
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    made = 0
    info: Dict[str, object] = {}
    for end, a_end in ((-1, s0), (+1, s1)):
        pr = _probe_port(shape, axis, end, a_end, pts, fidx=fidx, verbose=verbose)
        if pr is None:
            return None, {"reason": "端%+d 端口探测失败" % end}
        rim = pr["rim"]
        c_p, n_p = pr["plane"]
        # 口环校验：面积必须和"截面内孔环"同量级。管件端口常有**倒棱/倒圆小面**，
        # 射线打到那里取到的"口环"只有几十 mm²（真实内孔 2000~3400 mm²），
        # 盖片就小得切不断内芯条 —— 这种直接退回截面内孔环。
        try:
            a_rim = _wire_area(rim)
            a_ref = _wire_area(pr["rim_station"])
        except Exception:  # noqa: BLE001
            a_rim, a_ref = 0.0, 0.0
        if not (a_ref > 1e-6 and 0.5 * a_ref <= a_rim <= 2.0 * a_ref):
            rim = pr["rim_station"]
            if verbose:
                print("    · 端%+d 口环面积 %.1f 与截面 %.1f 不匹配 → 用截面内孔环"
                      % (end, a_rim, a_ref), flush=True)
        # 口环**不做外扩**：外扩会让盖片侧面与管内壁几乎平行（相切），OCCT 布尔会慢几十倍
        # （实测外扩 0.3 mm：切一次 139 s；用精确口环：0.1 s）。精确口环的侧面与内壁重合，
        # 但那是"面-面重合"，BOPAlgo 走 same-domain 通道，反而又快又干净。
        rim_use = rim
        try:
            face = BRepBuilderAPI_MakeFace(rim_use).Face()
        except Exception as exc:  # noqa: BLE001
            return None, {"reason": "端%+d 口环建面失败(%s)" % (end, type(exc).__name__)}
        if face.IsNull():
            return None, {"reason": "端%+d 口环建面为空" % end}
        # 只切"截面站位 → 产品真实端面之外 0.5 mm"这一小段：够切断内芯条，
        # 又不会在模具端面啃出深坑（斜口件也不会吃掉一大块）
        length = abs(float(a_end) - float(pr["station"])) + 0.5
        vec = np.zeros(3)
        vec[axis] = float(end) * length
        try:
            slab = BRepPrimAPI_MakePrism(
                face, gp_Vec(float(vec[0]), float(vec[1]), float(vec[2]))).Shape()
        except Exception as exc:  # noqa: BLE001
            return None, {"reason": "端%+d 盖片拉伸失败(%s)" % (end, type(exc).__name__)}
        if slab.IsNull():
            return None, {"reason": "端%+d 盖片为空" % end}
        builder.Add(comp, slab)
        made += 1
        info["end%+d" % end] = (round(pr["station"], 2), round(length, 2))
    return (comp if made == 2 else None), info


def wall_end_station(shape, axis: int, end: int, a_end: float,
                     tol: float = 0.05, verbose: bool = False):
    """二分求"管壁**真正结束**的长轴位置" = 端口开口所在平面（垂直于长轴）。

    为什么要量这个：截断平面必须落在**管壁还存在**的位置上。
      * 切在管壁端面之外（哪怕是采样极值那种虚高 0.1~0.5 mm）→ 端口那一圈没有管壁隔着，
        内芯条与主体仍然连通 → 内孔腔切不出来（实测就是这个问题）；
      * 切得太后（深入管件）→ 芯棒少一截、模具端面多一圈台阶。
    端口附近常有倒棱/倒圆小面（本件 +X 端最后 0.9 mm 是收口倒圆），所以"采样极值"不是
    管壁端面。这里用**截面环数**二分：向外找第一个"不再有整环"的位置就是管壁端面。

    返回 (位置, 环路数) 或 None。
    """
    inner = None
    for ins in (0.2, 0.5, 0.9, 1.4, 2.0, 3.0, 5.0):
        a = a_end - end * ins
        try:
            if len(section_wires(shape, axis, float(a))) >= 2:
                inner = float(a)
                break
        except Exception:  # noqa: BLE001
            continue
    if inner is None:
        return None
    lo = float(a_end)                     # 外侧：已知没有整环（或本来就是端面外）
    try:
        if len(section_wires(shape, axis, lo)) >= 2:
            return lo, 2                  # 端面处就有整环 → 采样极值就是管壁端面
    except Exception:  # noqa: BLE001
        pass
    for _ in range(14):
        if abs(lo - inner) <= tol:
            break
        mid = 0.5 * (lo + inner)
        n = 0
        try:
            n = len(section_wires(shape, axis, float(mid)))
        except Exception:  # noqa: BLE001
            n = 0
        if n >= 2:
            inner = mid
        else:
            lo = mid
    if verbose:
        print("    · 端%+d 管壁端面(截面环二分) = %.3f（采样极值 %.3f，差 %.3f mm）"
              % (end, inner, a_end, abs(a_end - inner)), flush=True)
    return inner, 2


def detect_wall_openings(shape, axis: int, pts=None, n_station: int = 18):
    """扫一遍管壁有没有**侧向开口**（内孔腔与外界连通的洞）。

    做法：沿长轴取若干站位，从**内孔中心**朝垂直长轴的 4 个方向打射线，数穿透次数（应为 2：
    内表面 + 外表面）。出现 <2 次说明那个方向管壁是断的 —— 内芯条会从洞里与主体相连，
    **光截两端切不断**（实测 24TK_2052-2 在 x≈-117~-98 有侧向开口，就是它导致芯棒不对）。

    返回 [站位列]（空列表 = 管壁完整）。
    """
    if pts is None:
        return []
    P = np.asarray(pts, dtype=float)
    s0, s1 = float(P[:, axis].min()), float(P[:, axis].max())
    dirs = []
    for k in ((axis + 1) % 3, (axis + 2) % 3):
        for sgn in (1.0, -1.0):
            v = np.zeros(3)
            v[k] = sgn
            dirs.append(v)
    opened = []
    for i in range(1, n_station):
        a = s0 + (s1 - s0) * i / n_station
        try:
            ws = section_wires(shape, axis, float(a))
        except Exception:  # noqa: BLE001
            continue
        if len(ws) < 2:
            continue
        rim = max([w for w, _ar in ws[1:]], key=_wire_area)
        c = _wire_center(rim)
        if c is None:
            continue
        c = np.asarray(c, dtype=float)
        try:
            hits = [_ray_hits(shape, c, u) for u in dirs]
        except Exception:  # noqa: BLE001
            continue
        if any(len(h) < 2 for h in hits):
            opened.append(round(float(a), 2))
    return opened


def _half_space_prism(p, n, size: float):
    """造一个覆盖"过点 p、朝外法向 n 的那一侧"的大棱柱（用来把超出端面的料削掉）。"""
    p = np.asarray(p, dtype=float)
    n = np.asarray(n, dtype=float)
    n = n / max(float(np.linalg.norm(n)), 1e-12)
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, ref)
    u = u / max(float(np.linalg.norm(u)), 1e-12)
    v = np.cross(n, u)
    poly = BRepBuilderAPI_MakePolygon()
    for a, b in ((1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)):
        q = p + size * (a * u + b * v)
        poly.Add(gp_Pnt(float(q[0]), float(q[1]), float(q[2])))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    if face.IsNull():
        return None
    return BRepPrimAPI_MakePrism(
        face, gp_Vec(float(n[0] * size), float(n[1] * size), float(n[2] * size))).Shape()


def _extend_core_to_wall_ends(core, product, axis: int, wall_ends, true_ends=None,
                              port_planes=None, grow: float = 0.3, beyond: float = 0.2,
                              verbose: bool = False):
    """把芯棒两端**补齐到真实端口端面**（端口内孔里绝不能留模料）。

    为什么必须补：截断块法为了避开"相切"配置，把截断面从管壁端面往里让了 0.2 mm，
    于是芯棒比真实内孔腔短一截。而端口端面几乎总是**斜切**的（本件 -X 端斜 0.87 mm、
    +X 端斜 2.85 mm），端面最"深"的那一侧能差出好几毫米 —— 只补 0.3 mm 根本盖不住。

    后果（用户反馈的现象）：端口那一圈留下来的是**模料**，它把管件内孔堵住：
      · 侧视/端视看，上模与下模两片模料在端口内孔里对咬成一个**尖角/三角**；
      · 把管件与上下模装在一起看，就像模具"插进/重叠"进管件；
      · 这就是"型腔端口的小台阶凸起"。

    做法：在管壁端面往里 0.6 mm 处取内孔环 → 沿轴向**往外**拉过真实端面
    (`true_ends`，来自表面采样极值) → 再用**端口平面** `port_planes`（`port_cut_planes`
    给出，斜切端面也准）把超出端面 `beyond` 毫米以外的部分削掉 → 融合进芯棒。
    留 `beyond`（0.2 mm）而不是正好切在端面上：既躲开与产品端面的共面布尔，
    又只留下一个 0.2 mm 的让位坑（在管件端面之外，不影响贴合）。
    """
    if core is None:
        return core
    try:
        cb = shape_bbox(core)
        size = max(2.0 * float(np.linalg.norm([cb[3] - cb[0], cb[4] - cb[1],
                                               cb[5] - cb[2]])), 100.0)
    except Exception:  # noqa: BLE001
        size = 1000.0
    discs = []
    for k, end in enumerate((-1, +1)):
        a_wall = wall_ends[k] if k < len(wall_ends) else None
        if a_wall is None:                # 盲端（没有开口）：不用补
            continue
        a_wall = float(a_wall)
        a_s = a_wall - end * 0.6                       # 从端面里侧 0.6 mm 起（保证与芯棒有重叠）
        if true_ends is not None and k < len(true_ends) and true_ends[k] is not None:
            a_stop = float(true_ends[k]) + end * (beyond + 0.3)
        else:                                          # 不知道真实端面：退回原来的小余量
            a_stop = a_wall + end * (0.25 + float(grow))
        length = (a_stop - a_s) * end
        if length <= 1e-6:
            continue
        try:
            ws = section_wires(product, axis, float(a_s))
        except Exception:  # noqa: BLE001
            continue
        if len(ws) < 2:
            continue
        # 面积最大的那个环是外轮廓，其余是内孔环（可能有多个孔）
        items = sorted(ws, key=lambda wa: _wire_area(wa[0]))
        plane = None
        if port_planes is not None and k < len(port_planes) and port_planes[k] is not None:
            plane = port_planes[k]
        for rim, _ar in items[:-1]:
            try:
                face = BRepBuilderAPI_MakeFace(rim).Face()
                if face.IsNull():
                    continue
                vec = np.zeros(3)
                vec[axis] = float(end) * length
                disc = BRepPrimAPI_MakePrism(
                    face, gp_Vec(float(vec[0]), float(vec[1]), float(vec[2]))).Shape()
                if disc.IsNull():
                    continue
            except Exception:  # noqa: BLE001
                continue
            if plane is not None:
                try:
                    p = np.asarray(plane[0], dtype=float)
                    n = np.asarray(plane[1], dtype=float)
                    n = n / max(float(np.linalg.norm(n)), 1e-12)
                    if float(n[axis]) * float(end) < 0.0:
                        n = -n
                    cutter = _half_space_prism(p + n * float(beyond), n, size)
                    if cutter is not None:
                        v0 = shape_volume(disc)
                        _t = time.perf_counter()
                        disc = bool_cut(disc, cutter, DEFAULT_FUZZY_VALUES,
                                        label="芯棒端面削平")
                        if verbose:
                            print("      · 端%+d 补片削平 %.1f → %.1f mm³（%.1fs）"
                                  % (end, v0, shape_volume(disc),
                                     time.perf_counter() - _t))
                except Exception as exc:  # noqa: BLE001
                    if verbose:
                        print("      · 端%+d 补片削平失败(%s)" % (end, type(exc).__name__))
            discs.append(disc)
    if not discs:
        if verbose:
            print("    · [提示] 芯棒端面没有可补的补片（截面取不到内孔环）")
        return core
    try:
        # 注意: bool_fuse 只接两个 TopoDS_Shape（传 list 会直接 TypeError）。
        # 早期这里传的是 discs 列表，异常被吞掉 → 这一步其实**从来没生效过**，
        # 端口那一圈内孔一直被模料堵着（用户反复反馈的"端口小台阶/尖角"）。
        if len(discs) == 1:
            patch = discs[0]
        else:
            builder = BRep_Builder()
            patch = TopoDS_Compound()
            builder.MakeCompound(patch)
            for d in discs:
                builder.Add(patch, d)
        _t = time.perf_counter()
        out = bool_fuse(core, patch, DEFAULT_FUZZY_VALUES, label="芯棒端面补齐")
        if verbose:
            print("      · 补齐融合耗时 %.1fs" % (time.perf_counter() - _t))
        sols = _solids(out)
        v_out = sum(shape_volume(s) for s in sols) if sols else shape_volume(out)
        if v_out + 1e-6 >= shape_volume(core):
            if verbose:
                print("  - 芯棒端面已补齐到管件端口端面（补 %.1f mm³，%d 块，"
                      "端口内孔不再被模料塞住）"
                      % (v_out - shape_volume(core), len(sols) or 1))
            return out
        if verbose:
            print("    · [提示] 补片融合后体积反而变小（%.1f → %.1f），放弃补齐"
                  % (shape_volume(core), v_out))
    except Exception as exc:  # noqa: BLE001
        if verbose:
            print("    · [提示] 芯棒端面补齐融合失败(%s: %s)，放弃补齐"
                  % (type(exc).__name__, exc))
    return core


def port_cutters(planes, block, size: Optional[float] = None):
    """由端口平面生成"切掉平面外侧模料"的两个大棱柱（合成一个 compound）。

    这样"截断块"= 模芯 − 端口外侧，而不是"用垂直长轴的平面去截"——斜切端口也切得准。
    """
    bb = shape_bbox(block)
    diag = float(np.linalg.norm([bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]]))
    size = float(size) if size else max(2.0 * diag, 100.0)
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    made = 0
    for p, n in planes:
        p = np.asarray(p, dtype=float)
        n = np.asarray(n, dtype=float)
        n = n / max(float(np.linalg.norm(n)), 1e-12)
        ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        u = np.cross(n, ref)
        u = u / max(float(np.linalg.norm(u)), 1e-12)
        v = np.cross(n, u)
        try:
            poly = BRepBuilderAPI_MakePolygon()
            for a, b in ((1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)):
                q = p + size * (a * u + b * v)
                poly.Add(gp_Pnt(float(q[0]), float(q[1]), float(q[2])))
            poly.Close()
            face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
            if face.IsNull():
                continue
            prism = BRepPrimAPI_MakePrism(
                face, gp_Vec(float(n[0] * size), float(n[1] * size), float(n[2] * size))).Shape()
            if prism.IsNull():
                continue
            builder.Add(comp, prism)
            made += 1
        except Exception:  # noqa: BLE001
            continue
    return comp if made == 2 else None


def trim_core_and_hull(block, product, axis: Optional[int] = None, pts=None,
                       fidx=None, eps: float = 0.0, fuzzy=DEFAULT_FUZZY_VALUES,
                       verbose: bool = True, run_parallel: bool = False):
    """**截断块法**：一次普通布尔就分离出内孔腔（芯棒），再融合成"外形包络"。

    原理：把模芯块沿长轴**截到产品两端端口平面**后再减产品 —— 此时内孔里的
    "内芯条"径向被管壁隔开、轴向被截断平面切断，与主体不再相连，于是自动分离；
    再把 产品 + 芯棒 融合 = 外形包络（内孔填实），整块模芯减包络就得到**带两端
    封料壁**的正确型腔，且不需要任何封盖薄片。

    为什么比封盖法好：封盖是"孔口向外偏置 0.4 mm"的棱柱，侧面与管内壁近似平行
    （相切），OCC 布尔会卡到几十分钟甚至出不来；本方法只有普通横切布尔，实测
    秒级（截断 0.19 s）。

    返回 (core, hull, info)；拿不到芯棒时 core=None。
    """
    info: Dict[str, object] = {"ok": False}
    if axis is None:
        axis = pick_parting_axis(shape_bbox(product))
    bb = shape_bbox(block)
    cuts = []                     # [(标签, 截断块 shape)]
    pa = np.asarray(pts, dtype=float) if pts is not None else None
    if pa is None or not len(pa):
        pa = None
    we: List[Optional[float]] = [None, None]
    if pa is not None:
        for k, (end, a_end) in enumerate(((-1, float(pa[:, axis].min())),
                                          (+1, float(pa[:, axis].max())))):
            try:
                got = wall_end_station(product, axis, end, a_end, verbose=verbose)
            except Exception:  # noqa: BLE001
                got = None
            if got is not None:
                we[k] = float(got[0])
    # ---- 首选：**端口平面截断**（沿产品端口端面所在平面裁模芯块）----
    # 为什么把它放第一位：这一刀正好落在**管件端口端面上**，芯棒长度就等于内孔全长，
    # 端口内孔不会被模料堵住。而"垂直长轴、切在管壁端面里侧 0.2 mm"那一刀（见下面
    # 的备选）在**斜切端口**上会短一截（本件 -X 端斜 0.87 mm、+X 端斜 2.67 mm），
    # 端口那一圈内孔就留下一个楔形"塞子"—— 外形上就是"型腔端口的台阶/尖角"，
    # 装上管件看就像模具插进了管件内孔。
    # 速度：截断本身是"箱体 − 大棱柱"的平面布尔，实测 0.01 s；减产品 1.4 s（与备选同级）。
    planes = None
    try:
        planes = port_cut_planes(product, axis, pts=pts, verbose=verbose)
    except Exception:  # noqa: BLE001
        planes = None
    if planes is not None:
        try:
            cutters = port_cutters(planes, block)
        except Exception:  # noqa: BLE001
            cutters = None
        if cutters is not None:
            try:
                trimmed = bool_cut(block, cutters, fuzzy, run_parallel=run_parallel,
                                   label="模芯-端口外侧")
                cuts.append(("端口平面", trimmed))
            except Exception as exc:  # noqa: BLE001
                info["reason"] = "端口平面截断失败(%s)" % type(exc).__name__
    # ---- 备选 1：垂直长轴截到管壁真正的两端面（截面环二分位置）----
    # 注意：**允许只有一端有开口**（有的管件一端是收口/锥封的盲端，如 24TK_2052-2），
    # 盲端那侧不截断：内芯条在那一头本来就被产品材料封住，只截开口端就足以断开。
    if any(v is not None for v in we):
        lo3 = [float(bb[0]), float(bb[1]), float(bb[2])]
        hi3 = [float(bb[3]), float(bb[4]), float(bb[5])]
        # 截面平面往里让 0.2 mm：正好切在管壁端面上是"相切"配置，OCC 布尔会非常慢；
        # 让进去一点就变成普通横切。0.2 mm 还有个好处：截断面与产品端面之间的过渡面
        # 宽度就是这 0.2 mm，太薄（0.02~0.05）会在模具端面留下又细又长的"薄刃面"。
        if we[0] is not None:
            lo3[axis] = we[0] + 0.2
        if we[1] is not None:
            hi3[axis] = we[1] - 0.2
        if bb[axis] < lo3[axis] < hi3[axis] < bb[axis + 3]:
            try:
                box = BRepPrimAPI_MakeBox(gp_Pnt(*lo3), gp_Pnt(*hi3)).Shape()
                if not box.IsNull():
                    cuts.append(("管壁端面截断", box))
            except Exception as exc:  # noqa: BLE001
                info["reason"] = "管壁端面截断失败(%s)" % type(exc).__name__
    if not cuts:
        # 备选 2：沿端口平面贴面切薄片（斜切端口/曲面端面）
        slabs, _sinfo = port_cap_slabs(product, axis, pts, fidx=fidx, verbose=verbose)
        if slabs is not None:
            try:
                cuts.append(("端口盖片", bool_cut(block, slabs, fuzzy,
                                                 run_parallel=run_parallel,
                                                 label="模芯-端口盖片")))
            except Exception as exc:  # noqa: BLE001
                info["reason"] = "端口盖片截断失败(%s)" % type(exc).__name__
    if not cuts:
        planes = planes or port_cut_planes(product, axis, pts=pts, verbose=verbose)
        cutters = port_cutters(planes, block) if planes else None
        if cutters is not None:
            try:
                trimmed = bool_cut(block, cutters, fuzzy, run_parallel=run_parallel,
                                   label="模芯-端口外侧")
                cuts.append(("端口平面", trimmed))
            except Exception as exc:  # noqa: BLE001
                info["reason"] = "端口平面截断失败(%s)" % type(exc).__name__
    # 端口端面是曲面/斜切面（弯管上翘端）时的备用一手：内孔轮廓塞子沿局部轴切断。
    # 只有前面几种截断都拿不到芯棒时才真正去做 —— 塞子侧面与管内壁几乎重合，
    # 无谓地跑一次布尔很危险（实测能把 OCC 卡住），而且探测截面本身也要花时间。
    corks = None
    if not cuts:
        # 端口面找不到就用"垂直长轴的平面"截断（往里让 0.05 mm 保证断开管壁）
        if pts is None:
            try:
                pts, _n, _f = sample_surface(product, nuv=16)
            except Exception:  # noqa: BLE001
                pts = None
        if pts is None or not len(pts):
            return None, None, info
        pa = np.asarray(pts, dtype=float)
        lo = float(pa[:, axis].min()) + 0.05
        hi = float(pa[:, axis].max()) - 0.05
        if not (bb[axis] < lo < hi < bb[axis + 3]):
            info["reason"] = "端口平面落在模芯之外"
            return None, None, info
        for eps_try in (0.0, 0.05, 0.2):
            lo3 = [float(bb[0]), float(bb[1]), float(bb[2])]
            hi3 = [float(bb[3]), float(bb[4]), float(bb[5])]
            lo3[axis] = lo - eps_try
            hi3[axis] = hi + eps_try
            box_trim = BRepPrimAPI_MakeBox(gp_Pnt(*lo3), gp_Pnt(*hi3)).Shape()
            if box_trim.IsNull():
                continue
            cuts.append(("轴向截断", box_trim))
    core = None

    def _attempt(shape_trim, tag):
        cut1 = bool_cut(shape_trim, product, fuzzy, run_parallel=run_parallel,
                        label="截断块-产品")
        sols = _solids(cut1)
        if len(sols) < 2:
            info["reason"] = "内孔腔未断开(%s：孔可能开在侧面)" % tag
            return None
        main, others = _pick_main_solid(sols, block)
        c = max(others, key=shape_volume)
        if shape_volume(c) <= 1e-6:
            info["reason"] = "芯棒体积为 0"
            return None
        info["trim"] = tag
        info["planes"] = {"管壁端面截断": "wall_end", "端口盖片": "port_slab",
                          "端口平面": "port_face", "端口塞子": "port_cork"}.get(
            tag, "axis_slab")
        return c

    for tag, box_trim in cuts:
        core = _attempt(box_trim, tag)
        if core is not None:
            break
    # 注：早期用过的"端口塞子(port_corks)"已从兜底链里拿掉 —— 它用截面内孔轮廓沿局部轴
    # 拉一根长塞子，侧面与管内壁近乎重合，布尔会慢到 10 min 以上（实测 2052-2）。
    # 现在兜底直接交给主流程的"孔口封盖薄片"法（那一套有专门加厚重试，实测几秒~几分钟）。
    if core is None:
        # 截断法没切断时，先看一眼是不是**管壁上有侧向开口**（那种情况两端截断原理上切不断，
        # 需要侧向抽芯）。把位置写进 info，主流程会转成面向用户的警告。
        try:
            opened = detect_wall_openings(product, axis, pts=pts)
        except Exception:  # noqa: BLE001
            opened = []
        if opened:
            info["wall_openings"] = opened
        return None, None, info
    # 芯棒端面补齐到**真实端口端面**。
    # 走"端口平面截断"时芯棒本来就一直到端口端面（`_attempt` 出来的就是全长），不需要补；
    # 只有退回"垂直长轴截断"（端口面拟合不出来、斜切端面切不准）时才补一下。
    # 而且只在**拿得到端口平面**时才补：不然补片会伸到端面之外，反而在端口留个缺口。
    # 注：这一步较慢（补片侧面与内壁几乎重合，OCC 要跑几十秒~几分钟），属罕见兜底。
    if (any(v is not None for v in we) and info.get("trim") != "端口平面"
            and planes is not None and info.get("trim") == "管壁端面截断"):
        true_ends = None
        if pa is not None and len(pa):
            true_ends = (float(pa[:, axis].min()), float(pa[:, axis].max()))
        if verbose:
            print("    · 端口平面: 端-1 点 %s 法向 %s / 端+1 点 %s 法向 %s"
                  % (np.round(planes[0][0], 2), np.round(planes[0][1], 3),
                     np.round(planes[1][0], 2), np.round(planes[1][1], 3)))
        core2 = _extend_core_to_wall_ends(core, product, axis, we,
                                          true_ends=true_ends, port_planes=planes,
                                          verbose=verbose)
        if core2 is not None and shape_volume(core2) >= shape_volume(core):
            core = core2
    core_vol = shape_volume(core)
    info["core_volume"] = core_vol
    hull = bool_fuse(product, core, fuzzy, run_parallel=run_parallel, label="外形包络")
    hs = _solids(hull)
    if len(hs) > 1:
        hull = max(hs, key=shape_volume)
    v_hull = shape_volume(hull)
    v_exp = shape_volume(product) + core_vol
    info["hull_volume"] = v_hull
    if abs(v_hull - v_exp) > 0.005 * max(v_exp, 1.0):
        info["reason"] = "包络融合体积异常(%.1f vs %.1f)" % (v_hull, v_exp)
        return None, None, info
    info["ok"] = True
    if verbose:
        cb = shape_bbox(core)
        print("  - 截断块法(%s): 芯棒 %.1f mm³（%.1f × %.1f × %.1f），外形包络 %.1f mm³"
              % (info.get("trim", "?"), core_vol, cb[3] - cb[0], cb[4] - cb[1],
                 cb[5] - cb[2], v_hull))
    return core, hull, info


def _pick_main_solid(sols, block):
    """从多块结果里挑出"主体"（含模芯角落参考点的那块），返回 (主体, 其余块)。"""
    bb = shape_bbox(block)
    ref = gp_Pnt(bb[0] + 0.05 * (bb[3] - bb[0]),
                 bb[1] + 0.05 * (bb[4] - bb[1]),
                 bb[2] + 0.05 * (bb[5] - bb[2]))
    main = None
    for s in sols:
        try:
            cls = BRepClass3d_SolidClassifier(s)
            cls.Perform(ref, 1e-6)
            if cls.State() in (TopAbs_IN, TopAbs_ON):
                main = s
                break
        except Exception:  # noqa: BLE001
            continue
    if main is None:                     # 兜底：取体积最大者
        main = max(sols, key=shape_volume)
    return main, [s for s in sols if not s.IsSame(main)]


def build_envelope_die(block, product, plates=None, fuzzy=DEFAULT_FUZZY_VALUES,
                       verbose: bool = True, run_parallel: bool = False,
                       keep_pieces: bool = True, cavity=None,
                       hull=None, core=None):
    """模芯挖出"产品外形包络"型腔（内孔被填实）。

    先 Cut(模芯, 产品)，再用封盖薄片把贯通孔内的"内芯条"与主体切断，
    最后只保留主体 = 外形包络型腔（用模芯角落参考点判定，避免逐块算体积）。

    `keep_pieces=True` 时把被剔除的"内芯条"（= 产品**内孔腔** = **芯棒**）一并返回在
    info["core_pieces"] 里 —— 它是白捡的：挖包络本来就要切出来，不需要额外布尔。
    `cavity` 可直接传入上一次"模芯 − 产品"的结果（换封盖重试时省掉这次大布尔）。
    `hull`+`core` 则由 `trim_core_and_hull`（截断块法）算好：这里只做"整块模芯 − 包络"，
    完全不需要封盖。

    返回 (die, info dict)。
    """
    if hull is not None:
        cavity = bool_cut(block, hull, fuzzy, run_parallel=run_parallel,
                          label="型腔(外形包络)")
        sols = _solids(cavity)
        main, others = _pick_main_solid(sols, block)
        dropped = len(others)
        dropped_vol = sum(shape_volume(s) for s in others)
        info = {"solids_before": len(sols), "dropped_cores": 1 if core is not None else 0,
                "dropped_volume": shape_volume(core) if core is not None else 0.0,
                "core_pieces": [core] if (keep_pieces and core is not None) else [],
                "cavity": cavity, "via": "trim"}
        if verbose and core is not None:
            print("  - 型腔 = 整块模芯 − 外形包络（截断块法，无需封盖）")
        if verbose and dropped:
            print(f"  - [提示] 型腔由 {dropped + 1} 块组成（共剔除 {dropped_vol:.1f} mm³）")
        return main, info
    if cavity is None:
        cavity = bool_cut(block, product, fuzzy, run_parallel=run_parallel,
                          label="型腔(外形包络)")
    cavity_prod = cavity                     # 保留"模芯−产品"，供换封盖重试复用
    n_before = len(_solids(cavity))
    dropped = 0
    dropped_vol = 0.0
    core_pieces: List[object] = []
    if plates is not None:
        try:
            cavity2 = bool_cut(cavity, plates, fuzzy, run_parallel=run_parallel,
                               label="切断内芯条")
        except Exception as exc:  # noqa: BLE001
            if verbose:
                print(f"  - [警告] 封盖切断失败({type(exc).__name__})，内芯条可能未去除")
            cavity2 = cavity
        sols = _solids(cavity2)
        if len(sols) > 1:
            main, others = _pick_main_solid(sols, block)
            dropped = len(others)
            dropped_vol = sum(shape_volume(s) for s in others)
            if keep_pieces:
                core_pieces = others
            cavity = main
        elif len(sols) == 1:
            cavity = sols[0]
    info = {"solids_before": n_before, "dropped_cores": dropped,
            "dropped_volume": dropped_vol, "core_pieces": core_pieces,
            "cavity": cavity_prod, "via": "plates"}
    if verbose and dropped:
        print(f"  - 切断并剔除 {dropped} 个内芯条（共 {dropped_vol:.1f} mm³）"
              f"→ 型腔为产品外形包络、内芯条即芯棒")
    return cavity, info


# --------------------------------------------------------------------- 校验
def _mesh_points(shape, deflection: float = 0.25, angular: float = 0.5):
    """三角化后返回三角形重心向内偏移点（用于点采样干涉检验）。"""
    from geometry_utils import triangulate_shape
    tris = triangulate_shape(shape, deflection, angular)
    if len(tris) == 0:
        return np.zeros((0, 3))
    cen = tris.mean(axis=1)
    u = tris[:, 1] - tris[:, 0]
    v = tris[:, 2] - tris[:, 0]
    n = np.cross(u, v)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln < 1e-12] = 1.0
    return cen - 0.08 * (n / ln)


def interference_volume(moving, vec, obstacle, deflection: float = 0.25,
                        max_pts: int = 25000, fuzzy=DEFAULT_FUZZY_VALUES,
                        run_parallel: bool = False) -> Tuple[float, int, int]:
    """干涉检验：moving 平移 vec 后与 obstacle 的重叠体积。

    优先用精确布尔求交（把 moving 平移后其贴合面不再重合，布尔稳健）；
    布尔失败时退回点采样法（此时返回的第三个值 > 0 表示采样点数）。
    """
    moved = _translate(moving, vec)
    try:
        inter = bool_common(moved, obstacle, fuzzy, run_parallel=run_parallel,
                            label="干涉检验")
        v = shape_volume(inter)
        if v < 0.0:                      # 布尔结果朝向异常，按点采样复核
            raise RuntimeError("negative volume")
        return v, 0, 0
    except Exception:  # noqa: BLE001 —— 退回点采样
        pass
    pts = _mesh_points(moving, deflection)
    if len(pts) == 0:
        return 0.0, 0, 0
    pts = pts + np.asarray(vec)
    if len(pts) > max_pts:
        idx = np.random.RandomState(0).choice(len(pts), max_pts, replace=False)
        pts = pts[idx]
    cls = BRepClass3d_SolidClassifier(obstacle)
    n_in = 0
    for p in pts:
        try:
            cls.Perform(gp_Pnt(float(p[0]), float(p[1]), float(p[2])), 1e-6)
            if cls.State() == TopAbs_IN:
                n_in += 1
        except Exception:  # noqa: BLE001
            continue
    return shape_volume(moving) * n_in / max(len(pts), 1), n_in, len(pts)


def verify_demold(upper, lower, product, distances=(1.0, 10.0),
                  deflection: float = 0.25, verbose: bool = True,
                  run_parallel: bool = False) -> Dict[str, float]:
    """开模 / 顶出干涉检验（+Z 方向，精确布尔求交）。

    返回 {"upper_vs_product": 最大干涉, "product_vs_lower": 最大干涉,
          "upper_vs_lower": 最大干涉, "at": 距离}
    """
    worst = {"upper_vs_product": 0.0, "product_vs_lower": 0.0,
             "upper_vs_lower": 0.0, "at": 0.0}
    for d in distances:
        v1, _, _ = interference_volume(upper, (0.0, 0.0, d), product, deflection,
                                       run_parallel=run_parallel)
        v2, _, _ = interference_volume(product, (0.0, 0.0, d), lower, deflection,
                                       run_parallel=run_parallel)
        v3, _, _ = interference_volume(upper, (0.0, 0.0, d), lower, deflection,
                                       run_parallel=run_parallel)
        if verbose:
            print("    d=%5.1f mm | 上模抬起×产品 %10.3f | 产品顶出×下模 %10.3f | 上模×下模 %8.3f"
                  % (d, v1, v2, v3))
        for key, v in (("upper_vs_product", v1), ("product_vs_lower", v2),
                       ("upper_vs_lower", v3)):
            if v > worst[key]:
                worst[key] = v
                worst["at"] = d
    return worst


# --------------------------------------------------------------------- 主流程
def build_pipe_mold(
    block,
    product,
    core_size: Tuple[float, float, float],
    axis: Optional[int] = None,
    plate_depth: float = 0.6,
    verify: bool = True,
    fuzzy=DEFAULT_FUZZY_VALUES,
    verbose: bool = True,
    parting: str = "plane",
    plane_z: Optional[float] = None,
    run_parallel: bool = False,
    keep_core: bool = True,
) -> PipeMoldResult:
    """生成管件硬模成型的上下模。

    参数:
        parting: 分模面形式
            "plane"      —— **水平面**（默认，最简单、最好加工）：高度由
                            `choose_plane_height` 按"朝上成形面最低点 / 逐列最低材料 /
                            侧影曲线最低点"三者算出安全值；
            "silhouette" —— 沿产品侧影线随形（管件上翘/弯曲时更贴合，但面是曲面）。
        plane_z: **用户指定的水平分模面高度**（mm，仅 parting="plane" 生效）。
            None = 用上面的自动推荐值。模芯不变、只移动分模面位置，所以
            "上模高 + 下模高"始终等于模芯高；程序会回报两者分别是多少，
            并在超出自动推荐的安全区间时给出提示（可能需要开模/顶出干涉校验确认）。
        run_parallel: 布尔运算是否开 OCC 并行（默认关；开可提速，但本工程实测偶有不稳）。
        keep_core:   是否把被剔除的"内芯条"（= 产品内孔腔 = **芯棒**）保留在结果里。
                      它是挖外形包络时**顺带**切出来的，保留它不花额外布尔时间。
    """
    if parting not in ("plane", "silhouette"):
        raise PartingError(f"未知分模面形式 parting={parting!r}（plane / silhouette）")
    warnings: List[str] = []
    bbox = shape_bbox(block)
    if axis is None:
        axis = pick_parting_axis(shape_bbox(product))
    if verbose:
        print(f"  - 分模轴 = {'XY'[axis]}（管件长轴方向），分模面形式 = "
              f"{'水平面' if parting == 'plane' else '侧影随形面'}")
    t_all = time.perf_counter()
    timings: Dict[str, float] = {}

    # 0) 方向合理性提醒
    pb = shape_bbox(product)
    z_span = pb[5] - pb[2]
    if z_span > (pb[axis + 3] - pb[axis]) * 0.9:
        warnings.append(
            "产品 Z 向尺寸接近/超过水平长轴尺寸：本模式假定开模方向 +Z、"
            "分模面沿水平长轴随形，请确认摆放方向正确")

    # 1) 产品表面采样（后面侧影/夹紧都要用）
    t0 = time.perf_counter()
    pts, nrms, fidx = sample_surface(product)
    timings["sample"] = time.perf_counter() - t0

    # 2) 外形包络型腔（先建好模具，才能判断哪些产品表面真的被模具成形）
    #    优先用**截断块法**：把模芯块沿长轴截到产品两端端口平面后减产品，内孔腔自动
    #    分离（秒级，不需要封盖）；只有它拿不到内孔腔（孔开在侧面、端口不是平面……）
    #    时才退回"孔口封盖薄片"法。
    t0 = time.perf_counter()
    die_full = None
    dinfo: Dict[str, object] = {}
    plates = None
    n_holes = 0
    n_fail = 0
    plate_src: Dict[str, int] = {"section": 0, "inner_wire": 0, "skipped": 0}
    core_trim = None
    tinfo: Dict[str, object] = {}
    try:
        core_trim, hull, tinfo = trim_core_and_hull(
            block, product, axis=axis, pts=pts, fidx=fidx, verbose=verbose,
            run_parallel=run_parallel)
    except Exception as exc:  # noqa: BLE001 —— 截断块法只是加速路径，失败就退回封盖法
        tinfo = {"reason": f"{type(exc).__name__}: {exc}"}
        core_trim = None
    if core_trim is not None:
        die_full, dinfo = build_envelope_die(
            block, product, None, fuzzy, verbose, run_parallel=run_parallel,
            keep_pieces=keep_core, hull=hull, core=core_trim)
        if not dinfo.get("core_pieces") and keep_core:
            core_trim = None                 # 型腔没保住芯棒 → 退回封盖法
    if core_trim is None:
        # 管壁有侧向开口时两端截断/封盖**原理上**切不断（要侧向抽芯）——明确告诉用户，
        # 不要让它悄悄产出一个"看似有芯棒"的错模具
        if tinfo.get("wall_openings"):
            note = ("产品管壁上存在**侧向开口**（x ≈ %s）：内孔腔从洞里与主体相连，"
                    "两端截断/封盖无法分离，芯棒与型腔都不完整；"
                    "该处需要侧向抽芯（斜顶/滑块），本程序目前不生成侧抽芯"
                    % ", ".join(str(v) for v in tinfo["wall_openings"][:6]))
            warnings.append(note)
            if verbose:
                print("  - [警告] " + note)
        if verbose and tinfo.get("reason"):
            print(f"  - [提示] 截断块法未成功（{tinfo['reason']}），改用孔口封盖法 ...")
        plates, n_holes, n_fail, plate_src = hole_sealing_plates(
            product, axis=axis, bbox=shape_bbox(product), depth=plate_depth, pts=pts)
        if verbose:
            extra = (f"，跳过接缝内环 {plate_src['skipped']} 处"
                     if plate_src.get("skipped") else "")
            print(f"  - 孔口封盖: 真孔 {n_holes} 处"
                  f"（端面截面 {plate_src['section']} / 面内环 {plate_src['inner_wire']}）"
                  + (f"，失败 {n_fail} 处" if n_fail else "") + extra)
        die_full, dinfo = build_envelope_die(block, product, plates, fuzzy, verbose,
                                             run_parallel=run_parallel,
                                             keep_pieces=keep_core)
        # 封盖没切断"内芯条"时自动加重封盖重试一次（实测有的件端口是斜切/锥形收口，
        # 0.4 mm 外扩 + 0.6 mm 厚的封盖封不住 → 内孔腔仍与主体相连，芯棒/四件套就没了）
        if keep_core and not dinfo.get("core_pieces") and n_holes:
            if verbose:
                print("  - [重试] 封盖未切断内孔腔 → 改用更厚/更外扩的封盖再切一次 ...")
            plates2, n2, f2, src2 = hole_sealing_plates(
                product, axis=axis, bbox=shape_bbox(product),
                depth=max(2.0, plate_depth * 4.0), grow=1.5, pts=pts)
            if plates2 is not None:
                die2, dinfo2 = build_envelope_die(
                    block, product, plates2, fuzzy, verbose, run_parallel=run_parallel,
                    keep_pieces=True, cavity=dinfo.get("cavity"))
                if dinfo2.get("core_pieces"):
                    if verbose:
                        print("  - [重试成功] 已切出内孔腔（%d 处孔口封盖，厚封盖）" % n2)
                    die_full, dinfo = die2, dinfo2
                    plates, n_holes, n_fail, plate_src = plates2, n2, f2, src2
                elif verbose:
                    print("  - [重试未成功] 内孔腔仍未与主体断开")
    timings["envelope"] = time.perf_counter() - t0
    core_pin = None
    core_volume = 0.0
    pieces = dinfo.get("core_pieces") or []
    if pieces:
        core_pin = max(pieces, key=shape_volume)
        core_volume = shape_volume(core_pin)
        if verbose:
            cbb = shape_bbox(core_pin)
            print("  - 芯棒(内孔腔): 体积 %.1f mm³，尺寸 %.1f × %.1f × %.1f mm"
                  % (core_volume, cbb[3] - cbb[0], cbb[4] - cbb[1], cbb[5] - cbb[2]))
    elif keep_core:
        warnings.append("未能在挖包络时切出内孔腔（芯棒）：产品内孔可能不在长轴两端开口，"
                        "或封盖未切断内芯条")
    if dinfo["dropped_cores"] == 0 and n_holes > 0:
        warnings.append(
            "已找到产品孔口但未能切断/剔除内芯条：模具可能仍被内芯卡住"
            "（见下面的开模/顶出干涉检验结果）")

    # 3) 贴模成形面判定（用**射线法**，秒级；不再对上千面的型腔模做逐点分类）
    t0 = time.perf_counter()
    flags = outer_contact_flags(product, pts, nrms, fidx)
    contact = flags[np.clip(fidx, 0, len(flags) - 1)] if len(flags) else \
        np.ones(len(pts), dtype=bool)
    if contact.sum() < 0.05 * len(contact):
        # 射线法给出退化结果时，才退回"用型腔模逐点分类"（很慢但更稳）
        if verbose:
            print("  - [提示] 射线法判定的成形面过少，退回用型腔模判定（较慢）...")
        flags = contact_face_flags(product, die_full, pts=pts, nrms=nrms, fidx=fidx)
        contact = flags[np.clip(fidx, 0, len(flags) - 1)] if len(flags) else \
            np.ones(len(pts), dtype=bool)
        timings["contact_die"] = time.perf_counter() - t0

    # 侧影分模高度曲线（用"贴模接触点"夹在材料厚度区间内）
    xs, hs, pts, nrms, info = silhouette_profile(
        product, axis=axis, pts=pts, nrms=nrms, fidx=fidx, contact=contact)
    timings["silhouette"] = time.perf_counter() - t0
    if verbose:
        print("  - 侧影分模高度 h = %.2f .. %.2f mm（随长轴变化 %.2f mm）"
              % (info["h_min"], info["h_max"], info["h_max"] - info["h_min"]))
        print("  - 贴模接触点: %d / %d 个采样点被模具成形"
              % (info["contact_points"], info["sample_points"]))
        if info.get("clamp_conflicts"):
            print("  - [提示] %d 个站位存在\"朝下面高于朝上面\"（端面斜切或局部倒扣）："
                  "分模面保持侧影高度，由开模干涉检验判定" % info["clamp_conflicts"])
    if info["up_below_ratio"] > 0.10 or info["down_above_ratio"] > 0.10:
        note = ("产品存在局部倒扣征兆（斜切端面/上翘段），分模面按上面策略取值；"
                "是否真的卡模以开模/顶出干涉检验为准")
        warnings.append(note)

    # 4) 分模（contact 已在第 3 步用射线法算好，这里不再重复"用型腔模逐面分类"）
    t0 = time.perf_counter()
    plane_info: Dict[str, object] = {}
    if parting == "plane":
        z_auto, plane_info = choose_plane_height(pts, nrms, fidx, contact, hs, xs,
                                                 verbose=verbose)
        z_p = z_auto
        z_src = "auto"
        lo_ok = float(plane_info["column_low_max"] + 0.2)
        hi_ok = float(plane_info["up_facing_min"])
        # 安全区间上下限颠倒 = 这件**没有能完全避免台阶/钩料的水平面**（斜切端面 / 局部倒扣 /
        # 弯管上翘段）。**仍然按水平面分模**（用户明确要求：水平面最好加工、高度自己填），
        # 只把情况说清楚：区间颠倒时无论取哪个高度，都会有部分站位出现台阶或钩料，
        # 以开模/顶出干涉检验为准；也可以改用「侧影随形」分模面。
        infeasible = hi_ok < lo_ok - 1e-9
        if infeasible:
            note = ("水平分模面安全区间 [%.2f, %.2f] 上下限颠倒（斜切端面/局部倒扣）："
                    "任何高度都会在部分站位留下台阶或钩料；"
                    "本件侧影高度 %.2f ~ %.2f，可据此填一个高度，"
                    "或改用「侧影随形」分模面" % (lo_ok, hi_ok,
                                          float(np.min(hs)), float(np.max(hs))))
            warnings.append(note)
            if verbose:
                print("  - [提示] " + note)
        if parting == "plane":
            if plane_z is not None:
                z_p = float(plane_z)
                z_src = "user"
                z0, z1 = bbox[2], bbox[5]
                if not (z0 + 0.2 <= z_p <= z1 - 0.2):
                    raise PartingError(
                        "指定的分模面高度 z=%.2f mm 超出模芯 Z 范围 [%.2f, %.2f]"
                        "（需要留 0.2 mm 余量）" % (z_p, z0, z1))
                if z_p < lo_ok or z_p > hi_ok:
                    note = ("指定的分模面 z=%.2f mm 超出自动推荐的安全区间 [%.2f, %.2f]："
                            "可能在上/下模留下台阶或钩料，以开模/顶出干涉检验为准"
                            % (z_p, lo_ok, hi_ok))
                    warnings.append(note)
                    if verbose:
                        print("  - [提示] " + note)
            plane_info = dict(plane_info)
            plane_info.update({
                "z_plane": float(z_p),
                "z_auto": float(z_auto),
                "source": z_src,
                "safe_lo": lo_ok,
                "safe_hi": hi_ok,
                "infeasible": bool(infeasible),
            })
            xs_split = np.array([bbox[0] - 1.0, bbox[3] + 1.0])
            hs_split = np.array([z_p, z_p])
            if verbose:
                print("  - 沿水平分模面 z=%.2f 切分上/下模（%s；自动推荐 %.2f，"
                      "安全区间 %.2f ~ %.2f）..."
                      % (z_p, "用户指定" if z_src == "user" else "自动推荐",
                         z_auto, plane_info["safe_lo"], plane_info["safe_hi"]))
    if parting != "plane":
        xs_split, hs_split = xs, hs
        plane_info = dict(plane_info)
        plane_info.update({
            "z_plane": float(np.mean(hs)),
            "z_auto": float(np.mean(hs)),
            "source": "silhouette",
            "safe_lo": float(plane_info.get("column_low_max", np.min(hs)) + 0.2),
            "safe_hi": float(plane_info.get("up_facing_min", np.max(hs))),
        })
        if verbose:
            print("  - 沿侧影分模面切分上/下模（h = %.2f ~ %.2f，随长轴变化 %.2f mm）..."
                  % (float(np.min(hs)), float(np.max(hs)),
                     float(np.max(hs) - np.min(hs))))
    above = _profile_prism(bbox, xs_split, hs_split)
    upper = bool_common(die_full, above, fuzzy, label="上模")
    lower = bool_cut(die_full, above, fuzzy, label="下模")
    timings["split"] = time.perf_counter() - t0

    vu, vl, vd = shape_volume(upper), shape_volume(lower), shape_volume(die_full)
    vp = shape_volume(product)
    if vu <= 0.0 or vl <= 0.0:
        raise PartingError(
            f"分模后上/下模体积异常（上 {vu:.1f}, 下 {vl:.1f}）")

    # 上/下模的 Z 向高度（模芯不变、只移动分模面 → 两者之和 = 模芯高）
    z0b, z1b = float(bbox[2]), float(bbox[5])
    if parting == "plane":
        lower_h = float(z_p) - z0b
        upper_h = z1b - float(z_p)
    else:
        lower_h = float(np.min(hs_split)) - z0b
        upper_h = z1b - float(np.min(hs_split))
    height_info = {"block_z": [z0b, z1b], "block_height_mm": z1b - z0b,
                   "lower_height_mm": lower_h, "upper_height_mm": upper_h,
                   "sum_height_mm": lower_h + upper_h}
    if verbose:
        print("  - 上/下模高度: 下模 %.2f mm（z %.2f ~ %.2f）+ 上模 %.2f mm"
              "（z %.2f ~ %.2f）= 模芯高 %.2f mm"
              % (lower_h, z0b, z0b + lower_h, upper_h, z1b - upper_h, z1b,
                 height_info["block_height_mm"]))
        if abs(height_info["sum_height_mm"] - height_info["block_height_mm"]) > 1e-6:
            print("    [警告] 上模高 + 下模高 与模芯高不一致 %.3f mm"
                  % (height_info["sum_height_mm"] - height_info["block_height_mm"]))

    # 4) 校验（干涉量阈值取"型腔模体积的 0.02%"，最小 2 mm³：
    #     零间隙贴合时布尔求交会残留几十~几百 mm³ 的数值碎屑，属正常噪声）
    demold: Dict[str, float] = {}
    thr = max(2.0, 2e-4 * vd)
    if verify:
        t0 = time.perf_counter()
        if verbose:
            print("  - 开模/顶出干涉检验（精确布尔求交，0 表示可分离；"
                  "判定阈值 %.1f mm³）:" % thr)
        demold = verify_demold(upper, lower, product, verbose=verbose,
                               run_parallel=run_parallel)
        timings["verify"] = time.perf_counter() - t0
        if demold["upper_vs_product"] > thr:
            warnings.append(
                f"上模沿 +Z 抬起会与产品干涉 {demold['upper_vs_product']:.1f} mm³"
                f"（分离距离 {demold['at']} mm）")
        if demold["product_vs_lower"] > thr:
            warnings.append(
                f"产品沿 +Z 顶出会与下模干涉 {demold['product_vs_lower']:.1f} mm³"
                f"（分离距离 {demold['at']} mm）")
        if demold["upper_vs_lower"] > thr:
            warnings.append(
                f"上模与下模分离时相互干涉 {demold['upper_vs_lower']:.1f} mm³")
    nu, nl = len(_solids(upper)), len(_solids(lower))
    if nu > 1:
        warnings.append(f"上模由 {nu} 个分离实体组成（可能产生掉块/浮块）")
    if nl > 1:
        warnings.append(f"下模由 {nl} 个分离实体组成（可能产生掉块/浮块）")
    timings["total"] = time.perf_counter() - t_all
    if verbose:
        print("  - 耗时分解: " + "，".join(
            f"{k} {v:.1f}s" for k, v in timings.items()))
        print(f"  - 型腔模实体数: 上模 {nu} 个 / 下模 {nl} 个")
    # 开模检验通过时，把"局部倒扣征兆"降级为提示（不阻塞、不需处理）
    if demold and max(demold.get("upper_vs_product", 0.0),
                      demold.get("product_vs_lower", 0.0),
                      demold.get("upper_vs_lower", 0.0)) <= max(2.0, 2e-4 * vd):
        warnings = [w for w in warnings if "倒扣征兆" not in w]
        if verbose:
            print("  - 校验通过: 上模可抬起、产品可顶出，上下模互不干涉 ✅"
                  + ("（产品有斜切端面等局部倒扣征兆，但实测不卡模）"
                     if info.get("undercut_stations") else ""))
    elif not verify:
        # 本次没做闭环校验，必须明确告知（不能让人以为验过了）
        warnings = [w for w in warnings if "倒扣征兆" not in w]
        warnings.append("已跳过开模/顶出干涉校验（本次未做闭环验证）："
                        "如需确认可抬起/可顶出，请去掉 --no-verify 重跑")
        if verbose:
            print("  - [注意] 本次跳过了开模/顶出干涉校验（未闭环验证）")
    elif not warnings:
        print("  - 校验通过 ✅")

    return PipeMoldResult(
        upper_mold=upper,
        lower_mold=lower,
        die_full=die_full,
        core_size=tuple(float(v) for v in core_size),
        axis=axis,
        profile=[(float(x), float(h)) for x, h in zip(xs_split, hs_split)],
        parting_z_mean=float(np.mean(hs_split)),
        parting_kind=parting,
        upper_volume=vu,
        lower_volume=vl,
        die_volume=vd,
        product_volume=vp,
        envelope_extra_volume=(core_volume if core_pin is not None else
                               max(0.0, vd - (shape_volume(block) - vp))),
        core_pin=core_pin,
        core_volume=core_volume,
        demold=demold,
        warnings=warnings,
        stats={
            "silhouette": info,
            "plane": plane_info,
            "heights": height_info,
            "plates": {"holes": n_holes, "failed": n_fail, **plate_src},
            # 注意：core_pieces / cavity 是 TopoDS_Shape，**不能**进统计信息（会被序列化成 JSON 上报）
            "envelope": {k: v for k, v in dinfo.items()
                         if k not in ("core_pieces", "cavity")},
            "solids": {"upper": nu, "lower": nl},
            "timings_s": {k: round(v, 2) for k, v in timings.items()},
        },
    )
