# -*- coding: utf-8 -*-
"""模芯设计 / 芯棒生成 (core_design.py)

针对对象: **薄壁中空件**（管件、型材件）。
硬模（刚性模）成型时产品内孔是空的 —— 若不给内孔"撑腰"，成型/校形时管壁会被压塌。
本模块生成这根"撑腰"的**芯棒**（内芯 / mandrel）:

    沿产品长轴的两个端口面，把产品内孔**完整填满**，生成一根芯棒实体。

原理（复用已经验证过的"封盖切断"思路）
--------------------------------------
产品 P 是薄壁中空体，内孔在两端（端口面）开口。取一个把产品罩住的长方体 C（取芯块）:

    C − P                  → 模具料 + 内孔腔（内孔腔经两端开口与外界连通）
    C − P − 端口封盖薄片    → 把内孔腔与外界"切断"，内孔腔成为独立实体
    取**不含取芯块角点**的实体 = 内孔腔 = **芯棒** ✅

两个必须注意的点（和"挖型腔"的做法不同）:
1. **封盖薄片放在端口面之外**（从端口面向外拉伸）。这样内孔腔的两端正好落在端口面所在
   的平面上，芯棒与端口面**齐平**；若像挖型腔那样从端口面向内拉伸，芯棒会短一截。
2. **封盖用"覆盖端口整个截面"的长方体**（而不是孔口轮廓）—— 一定大于孔口、封得严实，
   且完全位于产品之外，绝不会误切产品本体（孔口轮廓在圆角收口/斜切端口处不好保证）。

生成后自动做检验（结果全部写进元数据）:
   · 内孔贴合: 产品"贴芯面"面积 ≈ 芯棒侧面面积（两者本来重合）→ 填满、无缝隙；
   · 不啃料:   芯棒 ∩ 产品 ≈ 0（芯棒不侵占管壁）；
   · 芯棒数量: 只应有 1 件（多个说明产品内部有多个独立空腔）；
   · 抽芯检验: 芯棒沿 ±长轴逐级平移，与产品的干涉量 → 判断能否整体直抽。

用法:
    python main.py --mode core --part 产品.stp --output ./output
    python core_design.py 产品.stp --output ./output          # 也可单独运行
    python core_design.py --selftest                          # 内置空心管自测
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple
import time

import numpy as np

from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRepTools import BRepTools
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Plane
from OCP.TopAbs import (TopAbs_FACE, TopAbs_IN, TopAbs_ON, TopAbs_REVERSED,
                        TopAbs_SOLID, TopAbs_WIRE)
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.gp import gp_Pnt

from cavity_cutter import DEFAULT_FUZZY_VALUES, bool_common, bool_cut
from errors import PartingError
from geometry_utils import shape_volume
from model_reader import read_product_model
from pipe_mold import (pick_parting_axis, sample_surface, section_wires,
                       _face_normal, _is_open_hole, _solids, _wire_center,
                       interference_volume, offset_wire_outward)

VERSION = "1.0"
# 抽芯/啃料干涉判定阈值：芯棒体积的 0.02%（零间隙贴合的布尔求交会有数值碎屑），最小 2 mm³
PULL_THRESHOLD_REL = 2e-4


@dataclass
class PortInfo:
    """端口（内孔开口）信息。"""

    end: int                      # -1 = 长轴小端, +1 = 长轴大端
    plane: float                  # 端口封盖所在坐标（= 端面最外点；芯棒端面与此齐平）
    station: float                # 取截面用的站位（端口面往内一点）
    opening: bool                 # 该端是否真的开口（截面存在孔环）
    opening_area: float           # 孔口面积 mm²
    outer_area: float             # 该站位产品截面外轮廓面积 mm²
    center: Tuple[float, float]   # 孔口中心（另两轴坐标）
    planar_faces: int             # 该端垂直于长轴的平面端面数量
    planar_area: float            # 这些端面的总面积 mm²
    flush_gap: float              # 端面最外点超出"端面基准"的轴向距离（圆角/斜切收口时 >0）

    def as_dict(self) -> Dict[str, object]:
        return {
            "end": "min" if self.end < 0 else "max",
            "plane": round(self.plane, 3),
            "station": round(self.station, 3),
            "opening": bool(self.opening),
            "opening_area_mm2": round(self.opening_area, 2),
            "outer_area_mm2": round(self.outer_area, 2),
            "center": [round(float(v), 3) for v in self.center],
            "planar_faces": self.planar_faces,
            "planar_area_mm2": round(self.planar_area, 2),
            "flush_gap_mm": round(self.flush_gap, 3),
        }


@dataclass
class CoreDesignResult:
    """芯棒设计结果。"""

    core: object                            # 芯棒实体
    axis: int
    length: float                           # 轴向长度（两端面之间）
    volume: float
    dims: Tuple[float, float, float]        # 芯棒包围盒尺寸
    ports: List[PortInfo] = field(default_factory=list)
    sections: Dict[str, object] = field(default_factory=dict)
    fill: Dict[str, float] = field(default_factory=dict)
    pull: Dict[str, object] = field(default_factory=dict)
    removable: bool = False
    pull_dir: str = ""
    core_count: int = 1
    warnings: List[str] = field(default_factory=list)
    stats: Dict[str, object] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.warnings


# --------------------------------------------------------------------- 基础工具
def _axis_other(axis: int) -> List[int]:
    return [i for i in range(3) if i != axis]


def _bbox_points(pts):
    return [float(pts[:, i].min()) for i in range(3)] + \
           [float(pts[:, i].max()) for i in range(3)]


def _face_area(face) -> float:
    g = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, g)
    return float(g.Mass())


def shape_bbox_tri(shape, deflection: float = 1.0, angular: float = 0.6):
    """按三角化顶点求包围盒（BRepBndLib 会被 BSpline 控制点撑大 1~2 mm）。"""
    from geometry_utils import shape_bbox, triangulate_shape
    try:
        tris = triangulate_shape(shape, deflection, angular)
        if len(tris):
            v = tris.reshape(-1, 3)
            return (float(v[:, 0].min()), float(v[:, 1].min()), float(v[:, 2].min()),
                    float(v[:, 0].max()), float(v[:, 1].max()), float(v[:, 2].max()))
    except Exception:  # noqa: BLE001
        pass
    return shape_bbox(shape)


def planar_axis_faces(shape, axis: int, cos_tol: float = 0.9,
                      pos: Optional[float] = None, band: float = 5.0
                      ) -> List[Tuple[float, float]]:
    """列出法向≈长轴方向的**平面面**，返回 [(平面坐标, 面积)]（按面积降序）。

    用于判断"端口面是不是一刀平的"——圆角收口/斜切端口找不到这种面。
    """
    out: List[Tuple[float, float]] = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        exp.Next()
        try:
            ad = BRepAdaptor_Surface(face)
            if ad.GetType() != GeomAbs_Plane:
                continue
            d = ad.Plane().Axis().Direction()
            n = np.array([d.X(), d.Y(), d.Z()])
            if face.Orientation() == TopAbs_REVERSED:
                n = -n
            if abs(n[axis]) < cos_tol:
                continue
            p = ad.Plane().Location()
            pv = float(np.array([p.X(), p.Y(), p.Z()])[axis])
            if pos is not None and abs(pv - pos) > band:
                continue
            out.append((pv, _face_area(face)))
        except Exception:  # noqa: BLE001
            continue
    out.sort(key=lambda t: -t[1])
    return out


def detect_ports(product, axis: Optional[int] = None, pts=None, nrms=None,
                 insets: Sequence[float] = (0.15, 0.3, 0.5, 0.8, 1.2, 1.8, 2.6, 4.0),
                 face_band: float = 3.0, verbose: bool = True) -> List[PortInfo]:
    """识别产品长轴两端的端口（内孔开口）并给出封盖位置。

    端口位置取"产品**真实几何**在长轴上的极值"（**不是** BRepBndLib 的包围盒极值 ——
    BSpline 控制点会把包围盒撑大 1~2 mm）。取截面时从端口面向内搜索**第一个带孔环的
    截面**（端面若是圆角/斜切收口，正好切在极值处会退化成一个很小的环）。
    """
    if pts is None or nrms is None:
        pts, nrms, _ = sample_surface(product)
    bb = _bbox_points(pts)
    if axis is None:
        axis = pick_parting_axis(tuple(bb))
    a_all = pts[:, axis]
    oth = _axis_other(axis)
    ports: List[PortInfo] = []

    for end in (-1, +1):
        a_end = float(a_all.max() if end > 0 else a_all.min())
        # --- 找截面：先找"带孔环"的，找不到再退化为任意有效截面 ---------------
        # 关键：站位必须落在"管壁仍然闭合"的位置。若端口是圆角/斜切收口，
        # 最外点（a_end）附近已经没有管壁了，那儿的内孔腔与外界径向相通、
        # 切不断（实测正端圆角收口 0.9 mm，用最外点做封盖时内孔腔连不到一起）。
        found = None
        for want_hole in (True, False):
            inset_prev = 0.0
            for inset in insets:
                a_try = a_end - end * inset
                if not (a_all.min() <= a_try <= a_all.max()):
                    continue
                ws = section_wires(product, axis, float(a_try))
                if not ws or (want_hole and len(ws) < 2):
                    inset_prev = inset          # 太靠外：管壁还没闭合
                    continue
                if want_hole:
                    # 二分细化：把站位尽量推到贴近真实开口处（芯棒端面才能与孔口齐平）
                    lo_i, hi_i = inset_prev, inset
                    for _ in range(8):
                        if hi_i - lo_i <= 0.02:
                            break
                        mid = 0.5 * (lo_i + hi_i)
                        w_mid = section_wires(product, axis, float(a_end - end * mid))
                        if len(w_mid) >= 2:
                            hi_i, a_try, ws = mid, a_end - end * mid, w_mid
                        else:
                            lo_i = mid
                found = (float(a_try), ws)
                break
            if found:
                break
        if found is None:
            raise PartingError(
                "未能在长轴 %s 端取得有效截面（端 %+d），无法定位端口"
                % ("XYZ"[axis], end))
        a_s, wires = found
        holes = wires[1:]
        outer_area = float(wires[0][1])

        # --- 端口面位置：孔口中心附近取真实几何极值（排除侧向凸台干扰）--------
        if holes:
            c = _wire_center(holes[0][0])
        else:
            c = _wire_center(wires[0][0])
        c = c if c is not None else (0.0, 0.0, 0.0)
        half = [(float(np.ptp(pts[:, i])) * 0.5 + 5.0) for i in range(3)]
        mask = ((np.abs(pts[:, oth[0]] - c[oth[0]]) < half[oth[0]]) &
                (np.abs(pts[:, oth[1]] - c[oth[1]]) < half[oth[1]]))
        a_near = pts[mask][:, axis] if mask.any() else a_all
        a_port = float(a_near.max() if end > 0 else a_near.min())

        # --- 端口是不是一刀平的平面端面 + 齐平偏差 ---------------------------
        pfaces = planar_axis_faces(product, axis, pos=a_port, band=face_band)
        near_mask = (np.abs(a_all - a_port) < 1.5)
        ref = pfaces[0][0] if pfaces else (
            float(np.median(a_all[near_mask])) if near_mask.any() else a_port)
        ports.append(PortInfo(
            end=end,
            plane=a_port,
            station=a_s,
            opening=bool(holes),
            opening_area=float(sum(a for _, a in holes)),
            outer_area=outer_area,
            center=(float(c[oth[0]]), float(c[oth[1]])),
            planar_faces=len(pfaces),
            planar_area=float(sum(a for _, a in pfaces)),
            flush_gap=float(abs(a_port - ref)),
        ))

    if verbose:
        for p in ports:
            print("  - 端口(%s端): 端口面 %s = %.3f，封盖站位 %s = %.3f，%s，"
                  "孔口 %.1f mm²，截面外轮廓 %.1f mm²"
                  % ("负" if p.end < 0 else "正", "XYZ"[axis], p.plane,
                     "XYZ"[axis], p.station,
                     ("开口" if p.opening else "**未见开口**"), p.opening_area,
                     p.outer_area))
            if not p.planar_faces:
                print("    [提示] 该端口没有垂直于长轴的平面端面（圆角/斜切收口）："
                      "芯棒端面取在\"管壁仍闭合的最后一处截面\"，与最外点相差 %.2f mm"
                      % p.flush_gap)
    return ports


def _port_plates(product, ports: Sequence[PortInfo], axis: int,
                 depth: float = 1.0, grow: float = 0.4, verbose: bool = True):
    """端口封盖薄片：从**管壁仍闭合的站位**（`PortInfo.station`）往外拉伸的"孔口轮廓"棱柱。

    做法与管件模具里"切断内芯条"完全一致（已验证）。要点：

    * 拉伸的**里面那一端必须在管壁闭合处**——否则内孔腔会从"管壁已经结束、但封盖还没开始"
      的那一小段径向漏到外界，切不断（实测正端圆角收口处漏 0.9 mm，布尔结果里
      内孔腔仍与主体相连 → 取不出芯棒）；
    * 封盖用**孔口轮廓向外偏置**（grow），保证完整覆盖孔口；
    * 该端若没有孔口轮廓（端部封闭），退化为用截面外轮廓，不影响结果。
    """
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    a_dir = [0.0, 0.0, 0.0]
    a_dir[axis] = 1.0
    n_ok = 0
    for p in ports:
        wires = section_wires(product, axis, p.station)
        if not wires:
            continue
        w0 = wires[1][0] if len(wires) >= 2 else wires[0][0]
        w_use = offset_wire_outward(w0, grow) or w0
        try:
            from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
            from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
            from OCP.gp import gp_Vec
            cap = BRepBuilderAPI_MakeFace(w_use).Face()
            if cap.IsNull():
                continue
            length = abs(p.plane - p.station) + depth
            vec = gp_Vec(a_dir[0] * p.end * length, a_dir[1] * p.end * length,
                         a_dir[2] * p.end * length)
            plate = BRepPrimAPI_MakePrism(cap, vec).Shape()
            if plate.IsNull():
                continue
            builder.Add(comp, plate)
            n_ok += 1
        except Exception:  # noqa: BLE001
            continue
    if verbose:
        print("  - 端口封盖: %d / %d 片（从管壁闭合站位向外拉伸，切断内孔腔）"
              % (n_ok, len(ports)))
    return (comp if n_ok else None), n_ok


def _port_slab(pts, axis: int, lo: float, hi: float, grow: float = 1.0):
    """两端端口面之间的"切片"长方体（用于把芯棒端面切平到端口面）。"""
    bb = _bbox_points(pts)
    l = [bb[i] - grow for i in range(3)]
    h = [bb[i + 3] + grow for i in range(3)]
    l[axis], h[axis] = lo, hi
    return BRepPrimAPI_MakeBox(gp_Pnt(*l), gp_Pnt(*h)).Shape()


def _inner_shell_solids(shape) -> List[object]:
    """把实体内部空腔（内壳）取出成独立实体（布尔结果的兜底处理）。"""
    from OCP.TopAbs import TopAbs_SHELL
    shells = []
    exp = TopExp_Explorer(shape, TopAbs_SHELL)
    while exp.More():
        shells.append(TopoDS.Shell_s(exp.Current()))
        exp.Next()
    out = []
    for k, sh in enumerate(shells):
        if k == 0:                      # 第一个是外壳
            continue
        try:
            b = BRep_Builder()
            s = TopoDS.Solid()
            b.MakeSolid(s)
            b.Add(s, sh)
            if shape_volume(s) < 0.0:
                s.Reverse()
            if shape_volume(s) > 0.0:
                out.append(s)
        except Exception:  # noqa: BLE001
            continue
    return out


def _core_block(pts, margin: float):
    """按**采样点**范围（真实几何）建取芯块：比 BRepBndLib 包围盒紧凑、布尔更快。"""
    bb = _bbox_points(pts)
    lo = [bb[i] - margin for i in range(3)]
    hi = [bb[i + 3] + margin for i in range(3)]
    return BRepPrimAPI_MakeBox(gp_Pnt(*lo), gp_Pnt(*hi)).Shape(), lo, hi


def _side_opening_plates(product, depth: float = 0.6, grow: float = 0.4,
                         max_n: int = 12, verbose: bool = True):
    """侧面开孔封盖（向外）：侧壁有孔时内孔腔会经该孔漏到外界。

    取面上的"内环"（孔口）沿该面外法向**向外**拉伸一小段薄片封住孔口
    （与端口封盖同一思路）。这样芯棒会把孔口一起填实（该处芯棒有凸起）。
    """
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    n_ok = n_skip = 0
    exp = TopExp_Explorer(product, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        exp.Next()
        if n_ok >= max_n:
            break
        try:
            outer = BRepTools.OuterWire_s(face)
        except Exception:  # noqa: BLE001
            continue
        n = _face_normal(face)
        if n is None:
            continue
        wexp = TopExp_Explorer(face, TopAbs_WIRE)
        while wexp.More():
            wire = TopoDS.Wire_s(wexp.Current())
            wexp.Next()
            if wire.IsSame(outer):
                continue
            if not _is_open_hole(product, wire, n):
                n_skip += 1
                continue
            try:
                from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
                from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
                from OCP.gp import gp_Vec
                from pipe_mold import offset_wire_outward, _translate
                w_use = offset_wire_outward(wire, grow) or wire
                cap = BRepBuilderAPI_MakeFace(w_use).Face()
                if cap.IsNull():
                    continue
                base = np.array([n[0], n[1], n[2]])
                cap = _translate(cap, tuple((base * 0.05).tolist()))
                plate = BRepPrimAPI_MakePrism(
                    cap, gp_Vec(base[0] * depth, base[1] * depth, base[2] * depth)).Shape()
                if plate.IsNull():
                    continue
                builder.Add(comp, plate)
                n_ok += 1
            except Exception:  # noqa: BLE001
                continue
    if verbose and (n_ok or n_skip):
        print("  - 侧面开孔封盖: %d 处（跳过接缝内环 %d 处）" % (n_ok, n_skip))
    return (comp if n_ok else None), n_ok, n_skip


def _offset_solid(shape, offset: float, tol: float = 1e-3):
    """实体整体偏置（给芯棒留装配间隙用）；失败返回 None。"""
    from OCP.BRepOffset import BRepOffset_Skin
    from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffsetShape
    from OCP.GeomAbs import GeomAbs_Arc
    try:
        mk = BRepOffsetAPI_MakeOffsetShape(shape, float(offset), float(tol),
                                           BRepOffset_Skin, False, False, GeomAbs_Arc)
        if mk.IsDone() and not mk.Shape().IsNull():
            return mk.Shape()
    except Exception:  # noqa: BLE001
        pass
    try:
        mk = BRepOffsetAPI_MakeOffsetShape()
        mk.PerformByJoin(shape, float(offset), float(tol))
        if mk.IsDone() and not mk.Shape().IsNull():
            return mk.Shape()
    except Exception:  # noqa: BLE001
        pass
    return None


# --------------------------------------------------------------------- 检验
def _axis_planar_area(shape, axis: int, cos_tol: float = 0.98) -> Tuple[float, float]:
    """返回 (总面积, 法向平行于长轴的平面面面积)。"""
    total = 0.0
    end_area = 0.0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        f = TopoDS.Face_s(exp.Current())
        exp.Next()
        a = _face_area(f)
        total += a
        try:
            ad = BRepAdaptor_Surface(f)
            if ad.GetType() != GeomAbs_Plane:
                continue
            d = ad.Plane().Axis().Direction()
            n = np.array([d.X(), d.Y(), d.Z()])
            if abs(n[axis]) >= cos_tol:
                end_area += a
        except Exception:  # noqa: BLE001
            continue
    return total, end_area


def _spread_indices(points, k: int):
    """在点集里挑 k 个"互相离得最远"的点（最远点采样）。

    逐面探针取样必须覆盖**整个面**：`sample_surface` 的采样顺序是"u 外层、v 内层"的
    参数网格，直接按等间隔下标取点会全部落在参数的同一端（圆柱面上就是同一个轴向站位），
    导致内孔面被误判成"没被芯棒填满"。这里改用几何上的最远点采样，和参数化无关。
    """
    n = len(points)
    if n <= k:
        return np.arange(n)
    sel = [0]
    d = np.linalg.norm(points - points[0], axis=1)
    for _ in range(k - 1):
        i = int(np.argmax(d))
        sel.append(i)
        d = np.minimum(d, np.linalg.norm(points - points[i], axis=1))
    return np.asarray(sel)


def face_probe_stats(product, core, axis: int, pts=None, nrms=None, fidx=None,
                     probe: float = 0.3, n_probe: int = 2,
                     verbose: bool = True) -> Dict[str, float]:
    """逐面判断"贴芯面"：面的**形心**沿该面外法向偏移 probe 后落在芯棒内部 → 该面被芯棒填实。

    产品贴芯面（内孔面）与芯棒侧面本来重合，因此两者的面积应当近似相等
    → 用 `fill_ratio = 贴芯面面积 / 芯棒侧面面积` 判断"是否填满、有无缝隙"。

    为什么用**形心**而不是采样点：这一步要给"芯棒"做点分类，而芯棒可能有上千个面
    （757 mm 管件上单次约 2 s），越少点越好；用采样点还会踩到"面参数矩形包含孔洞/
    采样点落在面的边界上"的坑（面两端刚好在芯棒端面外 → 误判该面没被填实）。
    形心 + 面法向是最省事的稳定取点。
    """
    if pts is None or nrms is None or fidx is None:
        pts, nrms, fidx = sample_surface(product)
    n_faces = int(fidx.max()) + 1 if len(fidx) else 0
    exp = TopExp_Explorer(product, TopAbs_FACE)
    faces, areas, centroids = [], [], []
    while exp.More():
        f = TopoDS.Face_s(exp.Current())
        exp.Next()
        g = GProp_GProps()
        try:
            BRepGProp.SurfaceProperties_s(f, g)
            areas.append(float(g.Mass()))
            c = g.CentreOfMass()
            centroids.append((c.X(), c.Y(), c.Z()))
        except Exception:  # noqa: BLE001
            areas.append(0.0)
            centroids.append(None)
        faces.append(f)
    cls = BRepClass3d_SolidClassifier(core)
    contact_area = other_area = 0.0
    n_contact = 0
    for fi in range(min(n_faces, len(faces))):
        idx = np.flatnonzero(fidx == fi)
        if len(idx) == 0 or centroids[fi] is None or areas[fi] <= 0.0:
            continue
        n_mean = nrms[idx].mean(axis=0)
        ln = float(np.linalg.norm(n_mean))
        if ln < 1e-9:
            continue
        n_mean = n_mean / ln
        c = np.asarray(centroids[fi])
        hit = False
        for dist in (probe, 2.0 * probe, -probe):
            q = c + dist * n_mean
            try:
                cls.Perform(gp_Pnt(float(q[0]), float(q[1]), float(q[2])), 1e-6)
                if cls.State() in (TopAbs_IN, TopAbs_ON):
                    hit = True
                    break
            except Exception:  # noqa: BLE001
                continue
        if hit:
            contact_area += areas[fi]
            n_contact += 1
        else:
            other_area += areas[fi]
    core_area, core_end_area = _axis_planar_area(core, axis)
    lateral = max(core_area - core_end_area, 1e-9)
    info = {
        "core_area": core_area,
        "core_lateral_area": lateral,
        "core_end_area": core_end_area,
        "core_contact_area": contact_area,
        "product_other_area": other_area,
        "fill_ratio": contact_area / lateral,
        "n_contact_faces": float(n_contact),
        "n_faces": float(len(faces)),
    }
    if verbose:
        print("  - 内孔贴合: 产品贴芯面 %.1f mm² / 芯棒侧面 %.1f mm² → 贴合率 %.4f"
              "（%d/%d 个面被芯棒填实）"
              % (contact_area, lateral, info["fill_ratio"], n_contact, len(faces)))
    return info


def _wire_length(wire) -> float:
    g = GProp_GProps()
    try:
        BRepGProp.LinearProperties_s(wire, g)
        return float(g.Mass())
    except Exception:  # noqa: BLE001
        return 0.0


def section_match_check(product, core, axis: int, n: int = 21, margin: float = 0.02,
                       verbose: bool = True) -> Dict[str, float]:
    """站位截面对比：**每个站位上芯棒截面面积应等于产品内孔截面面积**。

    这是最直接的"填满 / 不啃料"判据（也比逐点分类便宜、可靠）：
      * 芯棒截面 < 内孔截面 → 没填满（有缝隙）；
      * 芯棒截面 > 内孔截面 → 啃进管壁（倒扣）。
    只在芯棒轴向范围内、且离开两端 margin 比例的站位上比较（两端是封盖切出来的平面）。

    顺便用"内孔截面**周长**沿长轴积分"估算内孔侧面积（免费、不需要给芯棒做点分类），
    再与芯棒侧面面积（GProp 直接算）对比，给出一个"内孔贴合率"的独立交叉校核。

    返回 {"section_ratio_min/max/mean", "section_stations_checked",
          "bore_area_est_mm2", "core_lateral_area_mm2", "fill_ratio_est"}。
    """
    cbb = shape_bbox_tri(core, 1.0, 0.6)
    lo, hi = cbb[axis], cbb[axis + 3]
    span = hi - lo
    if span <= 0:
        return {}
    ratios, xs, pers = [], [], []
    for k in range(n):
        a = lo + span * (margin + (1.0 - 2.0 * margin) * k / max(n - 1, 1))
        wp = section_wires(product, axis, float(a))
        wc = section_wires(core, axis, float(a))
        if len(wp) < 2 or not wc:
            continue
        bore = float(sum(x for _, x in wp[1:]))          # 内孔截面（非最大环）
        ca = float(sum(x for _, x in wc))                # 芯棒截面
        if bore <= 0:
            continue
        xs.append(float(a))
        ratios.append((ca - bore) / bore)
        pers.append(float(sum(_wire_length(w) for w, _ in wp[1:])))
    if not ratios:
        return {}
    info = {
        "section_stations_checked": float(len(ratios)),
        "section_ratio_min": float(min(ratios)),
        "section_ratio_max": float(max(ratios)),
        "section_ratio_mean": float(float(np.mean(ratios))),
    }
    core_area, core_end_area = _axis_planar_area(core, axis)
    info["core_lateral_area_mm2"] = float(core_area - core_end_area)
    if len(xs) >= 2:
        bore_area = float(np.trapz(np.asarray(pers), np.asarray(xs)))
        info["bore_area_est_mm2"] = bore_area
        if bore_area > 0:
            info["fill_ratio_est"] = float(info["core_lateral_area_mm2"] / bore_area)
    if verbose:
        print("  - 站位截面对比: %d 个站位，芯棒截面相对内孔 %+.4f%% ~ %+.4f%%（均值 %+.4f%%）"
              % (len(ratios), info["section_ratio_min"] * 100,
                 info["section_ratio_max"] * 100, info["section_ratio_mean"] * 100))
    return info


def mass_balance_check(block_vol: float, product_vol: float, plates_vol: float,
                       main_vol: float, core_vols: Sequence[float],
                       verbose: bool = True) -> Dict[str, float]:
    """质量守恒校验：取芯块 − 产品 − 封盖 = 模具主体 + 各内孔腔。

    布尔运算若丢料/多料（OCC 偶发），这一步会立刻暴露（不需要额外布尔代价）。
    """
    expected = block_vol - product_vol - plates_vol
    actual = main_vol + float(sum(core_vols))
    err = expected - actual
    ratio = err / max(abs(expected), 1e-9)
    info = {"expected_volume": expected, "actual_volume": actual,
            "balance_error": err, "balance_ratio": ratio}
    if verbose:
        print("  - 质量守恒: 取芯块-产品-封盖 = %.1f，主体+空腔 = %.1f（差 %.1f mm³，%.4f%%）"
              % (expected, actual, err, ratio * 100.0))
    return info


def verify_fill(product, core, axis: int, fuzzy=DEFAULT_FUZZY_VALUES,
                pts=None, nrms=None, fidx=None, exact_overlap: bool = False,
                contact_area: bool = False, verbose: bool = True) -> Dict[str, float]:
    """填满 / 不啃料检验。

    默认只做"免费"的两项：**站位截面对比**（含面积交叉校核）+ 可选的精确布尔求交。
    contact_area=True 时再加做"逐面贴合率"（需要给芯棒做点分类，大件上很慢：
    757 mm 管件 52 个面 × 2 个探针 ≈ 22 min）。
    """
    out: Dict[str, float] = {}
    out.update(section_match_check(product, core, axis, verbose=verbose))
    if exact_overlap:
        try:
            inter = bool_common(core, product, fuzzy, label="芯棒×产品")
            out["overlap_volume"] = shape_volume(inter)
        except Exception:  # noqa: BLE001
            out["overlap_volume"] = float("nan")
        if verbose and not np.isnan(out["overlap_volume"]):
            print("  - 不啃料(精确): 芯棒 ∩ 产品 = %.3f mm³" % out["overlap_volume"])
    if contact_area:
        out.update(face_probe_stats(product, core, axis, pts, nrms, fidx, verbose=verbose))
    return out


def verify_pull(core, product, axis: int, length: float,
                distances: Optional[Sequence[float]] = None, threshold: float = 0.0,
                fuzzy=DEFAULT_FUZZY_VALUES, verbose: bool = True) -> Dict[str, object]:
    """抽芯检验：芯棒沿 ±长轴平移 d 后与产品的干涉量。

    d 取"分级距离"（含整体抽出距离 length+5）：近处能发现局部倒扣，远处能确认整体抽出。
    返回 {"plus", "minus", "at", "threshold", "removable", "distances", "detail"}。
    """
    if distances is None:
        L = max(float(length), 1.0)
        distances = sorted({1.0, 10.0, round(0.25 * L, 1), round(0.5 * L, 1),
                            round(0.75 * L, 1), round(L + 5.0, 1)})
    worst = {"plus": 0.0, "minus": 0.0}
    which = {"plus": 0.0, "minus": 0.0}
    detail = []
    for d in distances:
        for sign, key in ((+1, "plus"), (-1, "minus")):
            vec = [0.0, 0.0, 0.0]
            vec[axis] = sign * float(d)
            v, _, _ = interference_volume(core, tuple(vec), product, fuzzy=fuzzy)
            detail.append([key, float(d), float(v)])
            if v > worst[key]:
                worst[key] = v
                which[key] = float(d)
    thr = threshold if threshold > 0 else max(2.0, PULL_THRESHOLD_REL * shape_volume(core))
    ok_plus = worst["plus"] <= thr
    ok_minus = worst["minus"] <= thr
    removable = bool(ok_plus or ok_minus)
    if ok_plus and ok_minus:
        pull_dir = "±长轴"
    elif ok_plus:
        pull_dir = "+长轴端"
    elif ok_minus:
        pull_dir = "-长轴端"
    else:
        pull_dir = ""
    if verbose:
        print("  - 抽芯检验（判定阈值 %.1f mm³，0 = 该距离无干涉）:" % thr)
        for key, d, v in detail:
            print("      %s向 %8.1f mm : %12.3f mm³%s"
                  % ("正" if key == "plus" else "负", d, v,
                     "  ← 有干涉" if v > thr else ""))
        if removable:
            print("    → 可从 %s 整体直抽 ✅%s"
                  % (pull_dir, "" if (ok_plus and ok_minus) else
                     "（另一个方向不行：芯棒粗端过不去）"))
        else:
            print("    → 不能整体直抽：需分瓣芯 / 溶芯（低熔点合金）/ 斜抽芯 ❌")
    return {"plus": worst["plus"], "minus": worst["minus"],
            "at": which["plus"] if worst["plus"] >= worst["minus"] else which["minus"],
            "threshold": thr, "removable": bool(removable),
            "removable_plus": bool(ok_plus), "removable_minus": bool(ok_minus),
            "pull_dir": pull_dir,
            "distances": [float(d) for d in distances], "detail": detail}


def build_core_assembly(product, core):
    """[产品, 芯棒] 装配体（同一坐标系，芯棒正好嵌在产品内孔里）。"""
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    builder.Add(comp, product)
    builder.Add(comp, core)
    return comp


def core_section_profile(core, axis: int, n: int = 13, verbose: bool = True):
    """沿长轴的截面面积轮廓（说明芯棒由粗到细的变化，便于选棒料/判断抽芯）。"""
    bb = shape_bbox_tri(core, 1.0, 0.6)
    lo, hi = bb[axis], bb[axis + 3]
    xs, areas = [], []
    for k in range(n):
        a = lo + (hi - lo) * (k + 0.5) / n
        wires = section_wires(core, axis, float(a))
        if not wires:
            continue
        xs.append(float(a))
        areas.append(float(wires[0][1]))
    if not areas:
        return {}
    info = {
        "stations": [round(x, 2) for x in xs],
        "areas_mm2": [round(a, 2) for a in areas],
        "area_min_mm2": round(min(areas), 2),
        "area_max_mm2": round(max(areas), 2),
        "at_max": round(xs[int(np.argmax(areas))], 2),
        "taper_ratio": round(max(areas) / max(min(areas), 1e-9), 3),
    }
    if verbose:
        print("  - 芯棒截面: %.1f ~ %.1f mm²（最粗在 %s = %.1f），粗细比 %.2f"
              % (info["area_min_mm2"], info["area_max_mm2"], "XYZ"[axis],
                 info["at_max"], info["taper_ratio"]))
    return info


# --------------------------------------------------------------------- 主流程
def build_core_pin(product, axis: Optional[int] = None, plate_depth: float = 1.0,
                   clearance: float = 0.0, block_margin: Optional[float] = None,
                   verify: bool = True, pull_distances: Optional[Sequence[float]] = None,
                   exact_overlap: bool = False, quick: bool = False,
                   contact_area: bool = False,
                   fuzzy=DEFAULT_FUZZY_VALUES, verbose: bool = True) -> CoreDesignResult:
    """生成产品内孔的芯棒（沿长轴两个端口面把内孔填满）。

    参数:
        product:        产品实体（闭合薄壁中空件）。
        axis:           内孔长轴方向 0=X / 1=Y，None 时自动取水平长轴。
        plate_depth:    端口封盖拉伸长度（mm，默认 1.0）。
        clearance:      芯棒单边装配间隙（mm，默认 0 = 与内孔完全贴合）；
                        >0 时把芯棒整体做小一点，便于插拔。
        block_margin:   取芯块相对产品的外扩量（mm），None 时按 plate_depth 自动取。
        verify:         是否做贴合 / 抽芯检验。
        pull_distances: 抽芯检验的平移距离序列（mm），None 时自动分级取。
        exact_overlap:  是否加做"芯棒 ∩ 产品"精确布尔求交（很慢，757 mm 管件实测 1300 s）；
                        默认用"站位截面对比 + 面积交叉校核 + 质量守恒"判断填满/啃料（几十秒）。
        quick:          快速模式：抽芯检验只做 1/10 mm 两个距离。
        contact_area:   是否加做"逐面贴合率"（要给芯棒做点分类，757 mm 管件 ≈ 22 min，默认关）。
    """
    t_all = time.perf_counter()
    timings: Dict[str, float] = {}
    warnings: List[str] = []

    t0 = time.perf_counter()
    pts, nrms, fidx = sample_surface(product)
    timings["sample"] = time.perf_counter() - t0
    bb = _bbox_points(pts)
    if axis is None:
        axis = pick_parting_axis(tuple(bb))
    if verbose:
        print(f"  - 内孔长轴 = {'XY'[axis]}")

    # 1) 端口定位
    t0 = time.perf_counter()
    ports = detect_ports(product, axis=axis, pts=pts, nrms=nrms, verbose=verbose)
    timings["ports"] = time.perf_counter() - t0
    if not any(p.opening for p in ports):
        warnings.append("长轴两端都没有检出内孔开口：产品可能不是中空件，"
                        "或内孔开口不在长轴两端")

    # 2) 封盖（端口 + 侧面开孔）
    t0 = time.perf_counter()
    plates_ends, n_end = _port_plates(product, ports, axis, depth=plate_depth,
                                      verbose=verbose)
    plates_side, n_side, n_seam = _side_opening_plates(product, verbose=verbose)
    if n_side:
        warnings.append(
            f"产品侧面检出 {n_side} 处开孔：芯棒在该处有凸起把孔口填实，"
            "会妨碍直抽（建议该处做镶件/滑块，或确认工艺是否允许）")
    plates = plates_ends
    if plates_side is not None:
        if plates is None:
            plates = plates_side
        else:
            builder = BRep_Builder()
            comp = TopoDS_Compound()
            builder.MakeCompound(comp)
            builder.Add(comp, plates)
            builder.Add(comp, plates_side)
            plates = comp
    timings["plates"] = time.perf_counter() - t0

    # 3) 取芯块 − 产品 − 封盖 → 内孔腔（= 芯棒）
    t0 = time.perf_counter()
    margin = block_margin if block_margin else max(5.0, plate_depth + 2.0)
    block, blo, bhi = _core_block(pts, margin)
    cavity = bool_cut(block, product, fuzzy, label="取芯(块-产品)")
    if plates is not None:
        try:
            cavity = bool_cut(cavity, plates, fuzzy, label="取芯(切断端口)")
        except Exception as exc:  # noqa: BLE001
            if plates_ends is None:
                raise
            if verbose:
                print("  - [警告] 含侧面封盖的布尔失败(%s)，改用仅端口封盖重试"
                      % type(exc).__name__)
            warnings.append("侧面开孔封盖布尔失败，已忽略侧面封盖（芯棒可能取不出来）")
            cavity = bool_cut(cavity, plates_ends, fuzzy, label="取芯(切断端口)")
    sols = _solids(cavity)
    if not sols:
        raise PartingError("取芯布尔结果为空")
    sol_vols = [shape_volume(s) for s in sols]
    if verbose:
        print("  - 取芯结果: %d 个实体（体积 %s）"
              % (len(sols), " / ".join("%.0f" % v for v in sol_vols)))    # 主体 = 含取芯块角点的实体；其余 = 内孔腔（芯棒）
    ref = gp_Pnt(blo[0] + 0.05 * (bhi[0] - blo[0]),
                 blo[1] + 0.05 * (bhi[1] - blo[1]),
                 blo[2] + 0.05 * (bhi[2] - blo[2]))
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
    if main is None:
        main = max(sols, key=shape_volume)
    cores = [s for s in sols if not s.IsSame(main)]
    if not cores and main is not None:
        # 兜底：有些布尔结果会把内孔腔表示成主体的"内壳"，直接取出内壳做实体
        cores = _inner_shell_solids(main)
        if cores and verbose:
            print("  - [提示] 内孔腔以主体内壳形式存在，已提取为独立实体")
    timings["cavity"] = time.perf_counter() - t0
    if not cores:
        raise PartingError(
            "未能从产品内部取出封闭的内孔腔：请确认 ①产品是闭合实体；"
            "②内孔在长轴两端开口（或已被端口封盖封住）；③侧面没有与外界连通的孔")
    core = max(cores, key=shape_volume)
    if len(cores) > 1:
        warnings.append(f"产品内部有 {len(cores)} 个独立空腔，已取最大者作为芯棒"
                        "（其余空腔需另行设计小芯）")
        if verbose:
            print("  - [提示] 内部空腔数 = %d（取最大者）" % len(cores))
    balance = mass_balance_check(
        shape_volume(block), shape_volume(product),
        shape_volume(plates) if plates is not None else 0.0,
        shape_volume(main), [shape_volume(s) for s in cores], verbose=verbose)
    if abs(balance["balance_ratio"]) > 2e-3:
        warnings.append("取芯布尔质量不守恒（偏差 %.2f%%）：布尔运算可能丢料/多料，"
                        "请复核芯棒" % (balance["balance_ratio"] * 100.0))

    # 3b) 若端口封盖向外拉伸导致芯棒端面超出端口面，用端口平面之间的切片切平
    cbb = shape_bbox_tri(core, 1.0, 0.6)
    if all(p.opening for p in ports):
        lo = min(p.plane for p in ports)
        hi = max(p.plane for p in ports)
        if cbb[axis] < lo - 1e-3 or cbb[axis + 3] > hi + 1e-3:
            t0 = time.perf_counter()
            slab = _port_slab(pts, axis, lo, hi)
            trimmed = bool_common(core, slab, fuzzy, label="芯棒端面切平")
            if shape_volume(trimmed) > 0.0:
                core = trimmed
                cbb = shape_bbox_tri(core, 1.0, 0.6)
            timings["trim"] = time.perf_counter() - t0

    # 4) 装配间隙（可选）
    if clearance and clearance > 0:
        shrunk = _offset_solid(core, -abs(clearance))
        if shrunk is None or not (0.0 < shape_volume(shrunk) < shape_volume(core)):
            warnings.append(f"芯棒单边间隙 {clearance} mm 偏置失败，已按 0 间隙输出")
        else:
            core = shrunk
            if verbose:
                print("  - 芯棒已按单边间隙 %.3f mm 缩小" % clearance)

    volume = shape_volume(core)
    cbb = shape_bbox_tri(core, 1.0, 0.6)
    length = float(cbb[axis + 3] - cbb[axis])
    if verbose:
        print("  - 芯棒: 体积 %.1f mm³，尺寸 %.1f × %.1f × %.1f mm，轴向长 %.2f mm"
              % (volume, cbb[3] - cbb[0], cbb[4] - cbb[1], cbb[5] - cbb[2], length))
        for p in ports:
            gap = abs(p.station - p.plane)
            if gap > 0.05:
                print("    · %s端: 芯棒端面 %s=%.2f，端口面 %s=%.2f（相差 %.2f mm，"
                      "端部是圆角/斜切收口）"
                      % ("负" if p.end < 0 else "正", "XYZ"[axis], p.station,
                         "XYZ"[axis], p.plane, gap))

    # 5) 检验
    fill: Dict[str, float] = {}
    pull: Dict[str, object] = {}
    removable = False
    pull_dir = ""
    thr = max(2.0, PULL_THRESHOLD_REL * volume)
    if verify:
        t0 = time.perf_counter()
        fill = verify_fill(product, core, axis, fuzzy, pts, nrms, fidx,
                           exact_overlap=exact_overlap,
                           contact_area=contact_area, verbose=verbose)
        timings["fill_check"] = time.perf_counter() - t0
        ov = fill.get("overlap_volume")
        if ov is not None and not np.isnan(ov) and ov > thr:
            warnings.append("芯棒与产品管壁重叠 %.1f mm³（芯棒啃料），"
                            "请检查端口封盖位置" % ov)
        if fill.get("intrusion_ratio", 0.0) > 0.01:
            warnings.append("芯棒侵入了管壁（芯棒表面 %d/%d 个抽检点落在产品材料内），"
                            "请检查端口封盖位置"
                            % (fill["core_points_in_wall"],
                               fill["core_surface_points"]))
        smin = fill.get("section_ratio_min")
        smax = fill.get("section_ratio_max")
        if smin is not None and (smin < -0.005 or smax > 0.005):
            warnings.append("站位截面对不上：芯棒截面相对内孔 %+.2f%% ~ %+.2f%%"
                            "（负 = 没填满，正 = 啃进管壁），请复核"
                            % (smin * 100.0, smax * 100.0))
        fr = fill.get("fill_ratio", 1.0)
        if fr < 0.98:
            warnings.append("芯棒与内孔贴合率仅 %.1f%%：内孔可能未被完全填满（有缝隙）"
                            % (fr * 100.0))
        t0 = time.perf_counter()
        pull = verify_pull(core, product, axis, length, pull_distances, thr,
                           fuzzy, verbose=verbose)
        timings["pull_check"] = time.perf_counter() - t0
        removable = bool(pull.get("removable", False))
        pull_dir = str(pull.get("pull_dir", ""))
        if not removable:
            warnings.append(
                "芯棒不能整体直抽（正/负长轴最大干涉 %.1f / %.1f mm³）："
                "建议改用低熔点合金/溶芯，或把芯棒分段并加斜抽机构"
                % (pull.get("plus", 0.0), pull.get("minus", 0.0)))
    t0 = time.perf_counter()
    sections = core_section_profile(core, axis, verbose=verbose)
    timings["sections"] = time.perf_counter() - t0
    timings["total"] = time.perf_counter() - t_all
    if verbose:
        print("  - 耗时分解: " + "，".join(f"{k} {v:.1f}s" for k, v in timings.items()))
        if not warnings:
            print("  - 校验通过: 内孔已按端口面完整填满 ✅")
    return CoreDesignResult(
        core=core, axis=axis, length=length, volume=volume,
        dims=(float(cbb[3] - cbb[0]), float(cbb[4] - cbb[1]), float(cbb[5] - cbb[2])),
        ports=ports, sections=sections, fill=fill, pull=pull,
        removable=removable, pull_dir=pull_dir, core_count=len(cores),
        warnings=warnings,
        stats={"plates": {"ends": n_end, "side": n_side, "seam_skipped": n_seam},
               "clearance_mm": clearance,
               "mass_balance": {k: round(float(v), 4) for k, v in balance.items()},
               "timings_s": {k: round(v, 2) for k, v in timings.items()}},
    )


# --------------------------------------------------------------------- 管线
def design_core(part_path: Optional[str] = None, output_dir: str = "./output",
                prefix: str = "core", axis: Optional[int] = None,
                clearance: float = 0.0, plate_depth: float = 1.0,
                verify: bool = True, quick: bool = False,
                exact_overlap: bool = False, contact_area: bool = False,
                pull_distances: Optional[Sequence[float]] = None) -> dict:
    """完整管线：读产品 → 生成芯棒 → 检验 → 导出 STEP / 元数据。

    quick=True 时抽芯检验只做近距离（1/10 mm），适合快速看几何对不对。
    """
    from result_exporter import export_shapes

    t0 = time.perf_counter()
    if not part_path:
        raise PartingError("模芯设计需要指定产品数模（--part）")
    print(f"[1/3] 读取产品数模: {part_path}")
    product = read_product_model(part_path)
    print("    产品体积 = %.2f mm³，包围盒 = %s mm"
          % (product.volume,
             " × ".join("%.2f" % (product.bbox[i + 3] - product.bbox[i])
                        for i in range(3))))

    print("[2/3] 生成芯棒（沿长轴两端端口面把内孔填满）")
    res = build_core_pin(product.shape, axis=axis, plate_depth=plate_depth,
                         clearance=clearance, verify=verify,
                         exact_overlap=exact_overlap, quick=quick,
                         contact_area=contact_area,
                         pull_distances=(1.0, 10.0) if quick else pull_distances)
    for w in res.warnings:
        print(f"    [警告] {w}")

    print("[3/3] 导出芯棒 / 产品+芯棒装配体 STEP")
    assembly = build_core_assembly(product.shape, res.core)
    fill = {k: (None if isinstance(v, float) and np.isnan(v) else round(float(v), 4))
            for k, v in res.fill.items()}
    metadata = {
        "module": "core_design",
        "version": VERSION,
        "product_file": product.file_path,
        "product_format": product.file_format,
        "product_volume_mm3": product.volume,
        "product_size_mm": [product.bbox[i + 3] - product.bbox[i] for i in range(3)],
        "axis": "XYZ"[res.axis],
        "core_volume_mm3": res.volume,
        "core_length_mm": res.length,
        "core_size_mm": list(res.dims),
        "core_count": res.core_count,
        "ports": [p.as_dict() for p in res.ports],
        "sections": res.sections,
        "fill_check": fill,
        "pull_check": {
            "plus_max_mm3": round(float(res.pull.get("plus", 0.0)), 3),
            "minus_max_mm3": round(float(res.pull.get("minus", 0.0)), 3),
            "at_mm": res.pull.get("at", 0.0),
            "threshold_mm3": round(float(res.pull.get("threshold", 0.0)), 2),
            "removable": res.removable,
            "removable_plus": bool(res.pull.get("removable_plus", False)),
            "removable_minus": bool(res.pull.get("removable_minus", False)),
            "pull_dir": res.pull_dir,
            "distances_mm": res.pull.get("distances", []),
            "detail": res.pull.get("detail", []),
        },
        "removable": res.removable,
        "pull_dir": res.pull_dir,
        "core_warnings": list(res.warnings),
        "core_stats": res.stats,
    }
    paths = export_shapes({"core_pin": res.core, "core_assembly": assembly},
                          output_dir=output_dir, prefix=prefix, metadata=metadata)
    elapsed = time.perf_counter() - t0
    print(f"\n=== 模芯(芯棒)设计完成，耗时 {elapsed:.2f}s ===")
    print(f"  芯棒: {paths['core_pin']}")
    print(f"  产品+芯棒装配体: {paths['core_assembly']}")
    print(f"  元数据: {paths['metadata']}")
    return {"product": product, "core_design": res, "core": res.core,
            "assembly": assembly, "paths": paths, "metadata": metadata,
            "elapsed_s": elapsed}


# --------------------------------------------------------------------- 自测
def _test_hollow_tube():
    """内置自测：Φ40×3 空心管（长 100），芯棒应为 Φ34 圆柱，体积 90792 mm³。"""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax2, gp_Dir

    outer = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), 20.0, 100.0).Shape()
    inner = BRepPrimAPI_MakeCylinder(
        gp_Ax2(gp_Pnt(-1, 0, 0), gp_Dir(1, 0, 0)), 17.0, 102.0).Shape()
    tube = BRepAlgoAPI_Cut(outer, inner).Shape()
    print("测试件（Φ40×3 空心管，长 100）体积 = %.1f mm³" % shape_volume(tube))
    res = build_core_pin(tube, axis=0, verify=True, contact_area=True,
                         pull_distances=(1.0, 10.0, 105.0))
    theory = np.pi * 17 ** 2 * 100
    print("芯棒体积 = %.1f mm³（理论 %.1f）" % (res.volume, theory))
    print("芯棒尺寸 = %.2f × %.2f × %.2f" % res.dims)
    assert abs(res.volume - theory) < 300.0, "芯棒体积不对"
    assert res.removable, "圆柱芯棒应当可以直抽"
    assert res.fill.get("fill_ratio", 0) > 0.98, "贴合率不足"
    assert abs(res.fill.get("section_ratio_max", 1.0)) < 0.005, "站位截面对不上"
    res2 = build_core_pin(tube, axis=0, clearance=0.5, verify=False)
    print("留 0.5mm 间隙后体积 = %.1f mm³（应略小）" % res2.volume)
    assert res2.volume < res.volume, "间隙版应当更小"
    print("自测通过 ✅")


if __name__ == "__main__":
    import argparse

    from io_utils import ensure_utf8_stdout
    ensure_utf8_stdout()
    ap = argparse.ArgumentParser(description="模芯设计 / 芯棒生成（core_design）")
    ap.add_argument("part", nargs="?", default=None, help="产品数模 (.stp/.step)")
    ap.add_argument("--output", default="./output", help="输出目录")
    ap.add_argument("--prefix", default="core", help="输出文件名前缀")
    ap.add_argument("--axis", choices=("auto", "x", "y"), default="auto")
    ap.add_argument("--clearance", type=float, default=0.0, help="芯棒单边装配间隙 mm")
    ap.add_argument("--plate-depth", type=float, default=1.0, help="端口封盖拉伸长度 mm")
    ap.add_argument("--no-verify", action="store_true", help="跳过抽芯/贴合检验（快）")
    ap.add_argument("--quick", action="store_true", help="抽芯检验只做近距离（更快）")
    ap.add_argument("--exact-overlap", action="store_true",
                    help="额外做\"芯棒 ∩ 产品\"精确布尔求交判啃料（很慢：757mm 管件约 1300s）")
    ap.add_argument("--check-contact", action="store_true",
                    help="额外做逐面\"内孔贴合率\"（要给芯棒做点分类：757mm 管件约 22min）")
    ap.add_argument("--selftest", action="store_true", help="跑内置空心管自测")
    args = ap.parse_args()

    if args.selftest or not args.part:
        _test_hollow_tube()
        raise SystemExit(0)
    design_core(part_path=args.part, output_dir=args.output, prefix=args.prefix,
                axis=None if args.axis == "auto" else "xy".index(args.axis),
                clearance=args.clearance, plate_depth=args.plate_depth,
                verify=not args.no_verify, quick=args.quick,
                exact_overlap=args.exact_overlap, contact_area=args.check_contact)
