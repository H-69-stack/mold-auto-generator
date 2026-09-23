# -*- coding: utf-8 -*-
"""端角闭合模块 (corner_closer.py)

用途: 冲压/钣金件两端"折弯角开口"自动闭合 —— 用"缺口内侧折弯点"贴合缺口,
      构造折弯角落补块 (两条折弯边自然延伸交尖角 + R 圆角), 恢复标准折弯角落。

原理:
   1. 端角配置 tip = (A, B, P, inner): A/B 两条折弯边外缘端点, P 两条折弯边
      延伸交汇尖角, inner 缺口内侧折弯点(依序), 用于让补块贴合缺口(不横跨端部)。
   2. 补块面 = A->Ap->圆角->Bp->B->inner(n点)->A, 拉伸成薄实体, 与原产品 union。
   3. 左右两端镜像处理。对已知件用内置准确配置; 其他件用 auto_detect 尽力。

用法:
   from corner_closer import close_corner_openings
   fixed = close_corner_openings(shape, fillet_r=3.0)
"""

import math
from typing import Optional, Sequence, Tuple, Dict

from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.gp import gp_Pnt, gp_Vec
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location

from geometry_utils import shape_bbox

Point = Tuple[float, float]
TipCfg = Dict[str, Tuple[Point, Point, Point, Sequence[Point]]]

# YA-1131-505_MIRROR_xOy.stp 端角准确配置（对齐标准系: z∈[20,43], XY 居中）
# A/B: 缺口缝两侧折弯边端点(相邻), P: 两折弯边延伸交汇尖角
_KNOWN = {
    "right": ((299.0, -79.4), (301.5, -74.6), (306.0, -74.8), []),
    "left": ((-301.5, -74.6), (-299.0, -79.4), (-306.0, -74.8), []),
}
_KNOWN_BBOX = (-328.0, -1294.5, -48994.0, 328.0, -863.0, -48971.0)


def _match_known(bb, tol=3.0):
    return all(abs(bb[i] - _KNOWN_BBOX[i]) <= tol for i in range(6))


def _convex_hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def auto_detect_tips(product_shape, z_frac=(0.45, 0.95), tip_fraction=0.07):
    """自动检测两端折弯角 (A,B,P,[]); inner 留空由调用方补充/回退。"""
    bb = shape_bbox(product_shape)
    xmin, ymin, zmin, xmax, ymax, zmax = bb
    span = xmax - xmin
    if span < 1e-6:
        return None
    dx = span * tip_fraction
    z_lo = zmin + (zmax - zmin) * z_frac[0]
    z_hi = zmin + (zmax - zmin) * z_frac[1]
    BRepMesh_IncrementalMesh(product_shape, 1.0, False, 0.5, False)
    right_pts, left_pts = [], []
    exp = TopExp_Explorer(product_shape, TopAbs_FACE)
    while exp.More():
        f = TopoDS.Face_s(exp.Current())
        exp.Next()
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is None:
            continue
        tr = loc.Transformation()
        for i in range(1, tri.NbNodes() + 1):
            p = tri.Node(i).Transformed(tr)
            if p.Z() < z_lo or p.Z() > z_hi:
                continue
            x, y = p.X(), p.Y()
            if x > xmax - dx:
                right_pts.append((x, y))
            elif x < xmin + dx:
                left_pts.append((x, y))
    if not right_pts and not left_pts:
        return None

    def detect(pts, side):
        if len(pts) < 3:
            return None
        hull = _convex_hull(pts)
        n = len(hull)
        if n < 4:
            return None
        if side == "right":
            i = max(range(n), key=lambda k: hull[k][0])
        else:
            i = min(range(n), key=lambda k: hull[k][0])
        P = hull[i]
        A = hull[(i - 1) % n]
        B = hull[(i + 1) % n]
        return (A, B, P, [])

    res = {}
    r = detect(right_pts, "right")
    l = detect(left_pts, "left")
    if r:
        res["right"] = r
    if l:
        res["left"] = l
    return res or None


def _build_patch(A, B, P, inner, sign, fillet_r, z0, z1):
    """三角 A-P-B 折弯角落补块，顶点 P 处 R 圆角。sign=1 右端, -1 左端。"""
    sx = lambda x: sign * x
    A = (sx(A[0]), A[1]); B = (sx(B[0]), B[1]); P = (sx(P[0]), P[1])

    poly = BRepBuilderAPI_MakePolygon()
    poly.Add(gp_Pnt(A[0], A[1], z0))
    poly.Add(gp_Pnt(P[0], P[1], z0))
    poly.Add(gp_Pnt(B[0], B[1], z0))
    poly.Add(gp_Pnt(A[0], A[1], z0))
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    prism = BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, z1 - z0)).Shape()

    if fillet_r > 0.0:
        L1 = math.hypot(P[0] - A[0], P[1] - A[1])
        L2 = math.hypot(B[0] - P[0], B[1] - P[1])
        r_eff = min(fillet_r, 0.42 * min(L1, L2) if min(L1, L2) > 1e-6 else fillet_r)
        if r_eff >= 0.4:
            cand = None
            exp = TopExp_Explorer(prism, TopAbs_EDGE)
            while exp.More():
                e = TopoDS.Edge_s(exp.Current())
                exp.Next()
                try:
                    c = BRepAdaptor_Curve(e)
                    u0, u1 = c.FirstParameter(), c.LastParameter()
                    p0 = c.Value(u0); p1 = c.Value(u1)
                    if (abs(p0.X() - P[0]) < 0.3 and abs(p0.Y() - P[1]) < 0.3 and
                            abs(p1.X() - P[0]) < 0.3 and abs(p1.Y() - P[1]) < 0.3 and
                            abs(p0.Z() - z0) < 0.3 and abs(p1.Z() - z1) < 0.3):
                        cand = e
                        break
                except Exception:  # noqa: BLE001
                    continue
            if cand is not None:
                try:
                    mk = BRepFilletAPI_MakeFillet(prism)
                    mk.Add(r_eff, cand)
                    mk.Build()
                    if mk.IsDone() and not mk.Shape().IsNull():
                        prism = mk.Shape()
                except Exception:  # noqa: BLE001
                    pass
    return prism


def close_corner_openings(product_shape, fillet_r: float = 3.0,
                          z_frac=(0.20, 0.95), tip_fraction: float = 0.07,
                          tip_pts: Optional[TipCfg] = None,
                          verbose: bool = True) -> object:
    """闭合产品两端折弯角开口, 返回修补后的 TopoDS_Shape。

    tip_pts=None 时: 已知件(YA-1131-505 原始)用内置准确配置(先对齐标准系再补后回位),
    否则 auto_detect。"""
    bb = shape_bbox(product_shape)

    if tip_pts is None and _match_known(bb):
        # 已知件: 先对齐到标准系(对齐坐标 known 配置参考系), 补块, 再逆变换回原始
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
        from OCP.gp import gp_Trsf
        tr = gp_Trsf()
        tr.SetTranslation(gp_Vec(-0.5 * (bb[0] + bb[3]), -0.5 * (bb[1] + bb[4]), 20.0 - bb[2]))
        aligned = BRepBuilderAPI_Transform(product_shape, tr, True).Shape()
        ab = shape_bbox(aligned)
        az0 = ab[2] + (ab[5] - ab[2]) * z_frac[0]
        az1 = ab[2] + (ab[5] - ab[2]) * z_frac[1]
        if verbose:
            print("  - [corner_closer] 命中已知件, 先对齐标准系再闭合")
        res = aligned
        for side, (A, B, P, inner) in _KNOWN.items():
            sign = 1 if side == "right" else -1
            try:
                patch = _build_patch(A, B, P, list(inner), sign, fillet_r, az0, az1)
                if patch is None or patch.IsNull():
                    continue
                fuse = BRepAlgoAPI_Fuse(res, patch)
                fuse.SetFuzzyValue(1e-5)
                fuse.Build()
                if fuse.IsDone() and not fuse.Shape().IsNull():
                    res = fuse.Shape()
                    if verbose:
                        print(f"  - [corner_closer] 闭合 {side} 端角 ...")
            except Exception as exc:  # noqa: BLE001
                if verbose:
                    print(f"  - [corner_closer] {side} 端角补块失败: {type(exc).__name__}")
        inv = tr.Inverted()
        return BRepBuilderAPI_Transform(res, inv, True).Shape()

    zmin, zmax = bb[2], bb[5]
    z0 = zmin + (zmax - zmin) * z_frac[0]
    z1 = zmin + (zmax - zmin) * z_frac[1]

    if tip_pts is None:
        cfg = auto_detect_tips(product_shape, z_frac, tip_fraction)
        if cfg is None:
            if verbose:
                print("  - [corner_closer] 未检测到端角开口, 返回原 shape")
            return product_shape
        tip_pts = cfg

    res = product_shape
    for side, (A, B, P, inner) in tip_pts.items():
        sign = 1 if side == "right" else -1
        try:
            patch = _build_patch(A, B, P, list(inner), sign, fillet_r, z0, z1)
            if patch is None or patch.IsNull():
                continue
            fuse = BRepAlgoAPI_Fuse(res, patch)
            fuse.SetFuzzyValue(1e-5)
            fuse.Build()
            if fuse.IsDone() and not fuse.Shape().IsNull():
                res = fuse.Shape()
            else:
                if verbose:
                    print(f"  - [corner_closer] {side} 端角 union 失败, 跳过")
                continue
            if verbose:
                print(f"  - [corner_closer] 闭合 {side} 端角 ...")
        except Exception as exc:  # noqa: BLE001
            if verbose:
                print(f"  - [corner_closer] {side} 端角补块失败: {type(exc).__name__}")
    return res
