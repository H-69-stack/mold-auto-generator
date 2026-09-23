"""
Step 2: 精准识别折弯模块 -- BendDetector

基于 Step 1 的 BREP 拓扑数据，使用拓扑边重合 + 圆角特征双判定：

1. 两个平面共享公共 Edge + 中间存在连续圆角 Blend 面 -> 标记为直角折弯边
2. 圆柱柱面两侧衔接平面法兰 -> 标记为滚弯成型弧段
3. 计算折弯二面角 theta、折弯内半径 R
4. 过滤自由边（无相邻面、无圆角），排除外轮廓切割边

优势：只要 BREP 拓扑边连续，完全不受网格采样点间隙干扰。
"""

import os
import math
from collections import defaultdict


# ── 工具函数 ──

def _vec_length(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)


def _vec_normalize(v):
    l = _vec_length(v)
    if l < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0]/l, v[1]/l, v[2]/l)


def _vec_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def _vec_cross(a, b):
    return (a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])


def _vec_sub(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])


def _vec_add(a, b):
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2])


def _vec_scale(v, s):
    return (v[0]*s, v[1]*s, v[2]*s)


def _vec_dist(a, b):
    return _vec_length(_vec_sub(a, b))


def _vec_midpoint(a, b):
    return ((a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2)


def _vec_angle_between(a, b):
    """两个单位向量的夹角 (弧度, [0, pi])"""
    dot = _vec_dot(a, b)
    # clamp to [-1, 1] for numerical safety
    dot = max(-1.0, min(1.0, dot))
    return math.acos(dot)


# ── 主类 ──

class BendDetector:
    """折弯精准识别器

    用法:
        from brep_topology import BrepTopologyExtractor
        from bend_detector import BendDetector

        extractor = BrepTopologyExtractor("model.stp")
        topo = extractor.extract()

        detector = BendDetector(topo)
        result = detector.detect()

        for bend in result['bends']:
            print(f"Bend: face#{bend['plane_a']} <-> face#{bend['plane_b']}, "
                  f"angle={bend['bend_angle_deg']} deg, R={bend['bend_radius']} mm")
    """

    def __init__(self, topology: dict):
        self.topo = topology
        self.faces = topology['faces']
        self.edges = topology['edges']
        self.vertices = topology['vertices']
        self.thickness = topology['thickness'] or 2.0
        self.classification = topology['classification']
        self.planar_faces = set(self.classification.get('planar_faces', []))
        self.fillet_faces = self.classification.get('fillet_faces', [])

        # 辅助索引
        self._edge_to_faces = defaultdict(set)  # edge_idx -> {face_idx, ...}
        self._face_to_edges = defaultdict(set)  # face_idx -> {edge_idx, ...}
        self._face_adjacency = defaultdict(set) # face_idx -> {adj_face_idx, ...}
        self._build_indices()

        # 厚度对应面映射 (face -> 其厚度反向面)
        self._thickness_counterpart = {}  # face_idx -> its thickness counterpart
        self._build_thickness_map()

        # 钣金"侧"分配 (0 = 正面, 1 = 背面)
        self._face_side = {}  # face_idx -> 0 or 1
        self._assign_sheet_sides()

    def _build_indices(self):
        """从拓扑数据构建快速查找索引"""
        for f in self.faces:
            fi = f['face_idx']
            for ei in f.get('bounding_edges', []):
                self._face_to_edges[fi].add(ei)
                self._edge_to_faces[ei].add(fi)
            for aj in f.get('adjacent_faces', []):
                self._face_adjacency[fi].add(aj)

    def _build_thickness_map(self):
        """从厚度面对构建双向映射"""
        for fi, fj in self.topo.get('thickness_pairs', []):
            self._thickness_counterpart[fi] = fj
            self._thickness_counterpart[fj] = fi

    def _assign_sheet_sides(self):
        """将平面分配到钣金的"正面"(0) 或 "背面"(1)

        算法:
        1. 从基准面开始，标记为 side 0
        2. 通过圆角面邻接传播 side 0（同一侧的法兰）
        3. 通过厚度面对传播到 side 1（反面）
        4. 剩下的未标记面通过厚度图继续传播

        这样就能区分：
        - 正面（折弯内 R 所在面）的边 → 可能为折弯
        - 背面（厚度反面）的边 → 轮廓/剖面边，非折弯
        """
        if not self.planar_faces:
            return

        base_fi = self.topo.get('base_face_idx')
        all_planar = set(self.planar_faces)

        # BFS: 通过圆角面传播同一侧，通过厚度对面传播到另一侧
        from collections import deque

        # 初始化：基准面及所有"正面"法向一致的面 → side 0
        if base_fi is not None and base_fi in all_planar:
            q = deque()
            q.append((base_fi, 0))
            self._face_side[base_fi] = 0

            while q:
                fi, side = q.popleft()
                other_side = 1 - side

                # 传播 1: 厚度对应面 → 分配到反面
                counterpart = self._thickness_counterpart.get(fi)
                if counterpart is not None and counterpart in all_planar:
                    if counterpart not in self._face_side:
                        self._face_side[counterpart] = other_side
                        q.append((counterpart, other_side))

                # 传播 2: 通过圆角面连接的面 → 分配到同一侧
                for fl_info in self.fillet_faces:
                    fl_idx = fl_info['face_idx']
                    fl_planes = set(fl_info.get('adjacent_planes', []))
                    if fi in fl_planes:
                        for pj in fl_planes:
                            if pj in all_planar and pj not in self._face_side:
                                self._face_side[pj] = side
                                q.append((pj, side))

            # 处理剩余未标记面：从已标记面通过 BREP 邻接传播
            remaining = all_planar - set(self._face_side.keys())
            while remaining:
                # 找与已标记面相邻的未标记面
                assigned = False
                for fi in list(remaining):
                    for adj in self._face_adjacency.get(fi, set()):
                        if adj in self._face_side:
                            # 根据相邻面判断：如果通过圆角连到同一侧
                            side_of_adj = self._face_side[adj]
                            # 检查是否有圆角桥接
                            has_fillet_bridge = False
                            for fl_info in self.fillet_faces:
                                fl_planes = set(fl_info.get('adjacent_planes', []))
                                if fi in fl_planes and adj in fl_planes:
                                    has_fillet_bridge = True
                                    break
                            if has_fillet_bridge:
                                self._face_side[fi] = side_of_adj
                            else:
                                # 通过厚度关系或直接接触判断
                                counterpart = self._thickness_counterpart.get(fi)
                                if counterpart == adj:
                                    self._face_side[fi] = 1 - side_of_adj
                                else:
                                    # 默认同一侧
                                    self._face_side[fi] = side_of_adj
                            q.append((fi, self._face_side[fi]))
                            remaining.discard(fi)
                            assigned = True
                            break
                if not assigned:
                    # 彻底无法传播 → 全标为 side 0
                    for fi in remaining:
                        self._face_side[fi] = 0
                    break

    def _same_side(self, fa: int, fb: int) -> bool:
        """判断两个面是否在钣金的同一侧"""
        sa = self._face_side.get(fa)
        sb = self._face_side.get(fb)
        if sa is None or sb is None:
            return True  # 未知 → 保守地认为是同侧
        return sa == sb

    def _are_thickness_counterparts(self, fa: int, fb: int) -> bool:
        """判断两个面是否为厚度对应面（同一面墙的正反面）"""
        return self._thickness_counterpart.get(fa) == fb

    # ── 主入口 ──

    def detect(self) -> dict:
        """执行折弯识别，返回结构化结果"""
        bends = []

        # 方法 1: 圆角面中介折弯 (fillet-mediated bends)
        #         两个平面通过圆柱/圆环/BSpline 面连接
        fillet_bends = self._detect_fillet_bends()
        bends.extend(fillet_bends)

        # 方法 2: 直接边折弯 (direct edge bends)
        #         两个平面直接共享边，无圆角面
        #         注意：排除已被方法 1 覆盖的平面对
        covered_pairs = set()
        for b in bends:
            key = tuple(sorted([b['plane_a'], b['plane_b']]))
            covered_pairs.add(key)
        direct_bends = self._detect_direct_edge_bends(skip_pairs=covered_pairs)
        bends.extend(direct_bends)

        # 去重：每对面只保留 R 最大的那条折弯
        bends = self._deduplicate_bends(bends)

        # 分类折弯类型
        for b in bends:
            b['bend_type'] = self._classify_bend_type(b)

        # 构建连接面索引（哪些平面被折弯连接）
        bend_face_pairs = set()
        for b in bends:
            bend_face_pairs.add(tuple(sorted([b['plane_a'], b['plane_b']])))

        # 过滤自由边
        free_edges = self._find_free_edges(bends)

        # 构建折弯图
        bend_graph = self._build_bend_graph(bends)

        return {
            'bends': bends,
            'total_bends': len(bends),
            'bend_face_pairs': [list(p) for p in bend_face_pairs],
            'free_edges': free_edges,
            'bend_graph': bend_graph,
        }

    def _deduplicate_bends(self, bends: list) -> list:
        """对每对面只保留一条折弯（优先 R 更大的）"""
        best = {}  # (pa, pb) sorted -> best bend
        for b in bends:
            key = tuple(sorted([b['plane_a'], b['plane_b']]))
            r = b.get('bend_radius') or 0
            if key not in best or r > (best[key].get('bend_radius') or 0):
                best[key] = b
        return list(best.values())

    # ── 方法 1: 圆角面中介折弯 ──

    def _detect_fillet_bends(self) -> list:
        """通过圆角面 (Cylinder/Torus/BSpline) 连接的两个平面 -> 折弯

        这是最可靠的折弯识别方式：
        - 圆角面提供了折弯内半径 R
        - 圆角面的轴线方向 = 折弯轴线方向
        - 两个平面的法向量夹角 = 折弯角度 theta

        重要修复：圆角面可能连接超过2个平面（如横跨整个零件宽度的
        长圆柱面，两端各连接不同平面），需要遍历所有平面 pair，
        不能只取前两个。
        """
        bends = []

        for fl_info in self.fillet_faces:
            fl_idx = fl_info['face_idx']
            adj_planes = fl_info.get('adjacent_planes', [])

            # 过滤出平面
            plane_candidates = [p for p in adj_planes if p in self.planar_faces]
            if len(plane_candidates) < 2:
                continue

            # 遍历所有平面对（不仅是前两个）
            for i in range(len(plane_candidates)):
                for j in range(i + 1, len(plane_candidates)):
                    pa, pb = plane_candidates[i], plane_candidates[j]
                    self._add_fillet_bend(
                        bends, fl_info, fl_idx, pa, pb
                    )

        return bends

    def _add_fillet_bend(self, bends: list, fl_info: dict, fl_idx: int,
                         pa: int, pb: int):
        """为一个特定的平面对 (pa, pb) 添加折弯记录

        重要修复：不再使用"两面必须共享边"的过滤条件，因为这会把
        真正的墙-墙折弯（通过同一圆柱面相连但不直接共享边）误杀。
        取而代之的是"边缘面过滤"：如果某一面的法向与圆柱轴线平行，
        则该面是边缘面（板厚截面），应排除。
        """
        # 获取法向量
        na = self._get_face_normal(pa)
        nb = self._get_face_normal(pb)
        if na is None or nb is None:
            return

        # 检查是否不共面（折弯角度 > 0.5 deg）
        bend_angle_rad = _vec_angle_between(na, nb)
        bend_angle_deg = math.degrees(bend_angle_rad)
        if bend_angle_deg < 0.5:
            return  # 几乎共面，不是折弯

        # ── 边缘面过滤 ──
        # 圆柱轴线方向：优先从圆角面获取，其次 cross(na, nb)
        cyl_axis_dir = fl_info.get('axis_direction')
        if cyl_axis_dir is None:
            cross_dir = _vec_cross(na, nb)
            if _vec_length(cross_dir) > 1e-12:
                cyl_axis_dir = _vec_normalize(cross_dir)

        if cyl_axis_dir is not None and _vec_length(cyl_axis_dir) > 1e-12:
            # 边缘面判定：法向与圆柱轴线平行 (|dot| > 0.99)
            # 边缘面是板厚截面（如 Y=±25 端面），不应作为折弯参与面
            dot_a = abs(_vec_dot(na, cyl_axis_dir))
            dot_b = abs(_vec_dot(nb, cyl_axis_dir))
            if dot_a > 0.99 or dot_b > 0.99:
                return  # 至少有一面是边缘面 → 跳过

        # 折弯半径 from fillet face
        bend_radius = fl_info.get('bend_radius')

        # 折弯轴线方向
        axis_dir = cyl_axis_dir
        if axis_dir is None:
            cross_dir = _vec_cross(na, nb)
            if _vec_length(cross_dir) > 1e-12:
                axis_dir = _vec_normalize(cross_dir)

        # 折弯轴线原点
        # 优先使用两面与圆角面的共享边交点，其次用圆柱面轴线原点
        shared_edges_ab = sorted(self._shared_edges_between_faces(pa, pb))
        bend_origin = self._compute_bend_origin(
            pa, pb, na, nb, axis_dir, fl_info,
            shared_edges_ab=shared_edges_ab if shared_edges_ab else None
        )

        # 折弯边：圆角面与两个平面的共享边
        tangent_edges_a = sorted(self._shared_edges_between_faces(pa, fl_idx))
        tangent_edges_b = sorted(self._shared_edges_between_faces(pb, fl_idx))

        bends.append({
            'plane_a': pa,
            'plane_b': pb,
            'fillet_face': fl_idx,
            'fillet_surface_type': fl_info.get('surface_type', '?'),
            'bend_axis_origin': bend_origin,
            'bend_axis_direction': axis_dir,
            'bend_angle_deg': round(bend_angle_deg, 2),
            'bend_angle_rad': round(bend_angle_rad, 6),
            'bend_radius': round(bend_radius, 4) if bend_radius else None,
            'tangent_edges_a': tangent_edges_a,
            'tangent_edges_b': tangent_edges_b,
            'shared_edges_between_planes': shared_edges_ab,
            'detection_method': 'fillet_mediated',
        })

    # ── 方法 2: 直接边折弯（无圆角面） ──

    def _detect_direct_edge_bends(self, skip_pairs: set) -> list:
        """检测两个平面直接共享边但无圆角面的折弯

        关键过滤规则：
        1. 两条面必须在钣金的同一侧（正面/背面），否则是轮廓/剖面边
        2. 两条面不能是厚度对应面（同一面墙的正反面）
        3. 两条面之间不能已有圆角桥接（方法 1 已覆盖）

        保留场景：
        - 锐角折弯（CAD 中未建模圆角）
        - 零半径折弯
        """
        bends = []
        checked = set()

        # 构建"参与折弯的面集"：已通过圆角连接到其他面的平面
        fillet_connected_faces = set()
        for fl_info in self.fillet_faces:
            for p in fl_info.get('adjacent_planes', []):
                if p in self.planar_faces:
                    fillet_connected_faces.add(p)

        for pa in self.planar_faces:
            for pb in self.planar_faces:
                if pa >= pb:
                    continue
                key = tuple(sorted([pa, pb]))
                if key in skip_pairs or key in checked:
                    continue
                checked.add(key)

                # 必须有共享边
                shared = self._shared_edges_between_faces(pa, pb)
                if not shared:
                    continue

                # ── 过滤规则 1: 两面必须在同一侧 ──
                if not self._same_side(pa, pb):
                    continue

                # ── 过滤规则 2: 不能是同一面墙的正反面 ──
                if self._are_thickness_counterparts(pa, pb):
                    continue

                # ── 过滤规则 3: 两个面都必须参与圆角折弯 ──
                # 理由：如果有一面不参与任何圆角折弯，说明该面不在"折弯网络"中
                # 它的边很可能是外轮廓边（如 face#6 与各法兰的边），而非真实折弯
                if pa not in fillet_connected_faces or pb not in fillet_connected_faces:
                    continue

                # ── 过滤规则 4: 共享边必须有至少一条是直线且足够长 ──
                # 折弯边是直线，太短（< 5mm）的边通常是圆角端盖边而非折弯边
                has_valid_edge = False
                for ei in shared:
                    if ei < len(self.edges):
                        edge = self.edges[ei]
                        if edge['curve_type'] == 'Line':
                            length = edge['curve_params'].get('length', 0)
                            if length >= 5.0:
                                has_valid_edge = True
                                break
                if not has_valid_edge:
                    continue

                # 法向量
                na = self._get_face_normal(pa)
                nb = self._get_face_normal(pb)
                if na is None or nb is None:
                    continue

                # 角度
                bend_angle_rad = _vec_angle_between(na, nb)
                bend_angle_deg = math.degrees(bend_angle_rad)
                if bend_angle_deg < 0.5:
                    continue  # 共面

                # 检查是否有圆角面也连接这两个平面
                # （如果有，应该已被方法 1 覆盖）
                has_fillet_bridge = False
                for fl_info in self.fillet_faces:
                    fl_planes = set(fl_info.get('adjacent_planes', []))
                    if pa in fl_planes and pb in fl_planes:
                        has_fillet_bridge = True
                        break

                if has_fillet_bridge:
                    continue  # 方法 1 已覆盖

                # 轴线方向 = 共享边的方向
                axis_dir = self._infer_axis_from_edges(shared)
                if axis_dir is None:
                    axis_dir = _vec_normalize(_vec_cross(na, nb))

                # 原点：取共享边中点
                origin = self._midpoint_of_edges(shared)

                bends.append({
                    'plane_a': pa,
                    'plane_b': pb,
                    'fillet_face': None,
                    'fillet_surface_type': None,
                    'bend_axis_origin': origin,
                    'bend_axis_direction': axis_dir,
                    'bend_angle_deg': round(bend_angle_deg, 2),
                    'bend_angle_rad': round(bend_angle_rad, 6),
                    'bend_radius': None,  # 无圆角 -> 尖角折弯，R = 0 或 K*t
                    'tangent_edges_a': [],
                    'tangent_edges_b': [],
                    'shared_edges_between_planes': sorted(shared),
                    'detection_method': 'direct_edge',
                })

        return bends

    # ── 折弯类型分类 ──

    def _classify_bend_type(self, bend: dict) -> str:
        """分类折弯类型

        - 'regular': 常规折弯 (直角或接近直角，小 R)
        - 'arc_bend': 滚弯成型弧段 (大 R 圆柱面)
        - 'sharp': 尖角折弯 (无圆角面)
        """
        r = bend.get('bend_radius')
        angle = bend.get('bend_angle_deg', 0)
        fl_type = bend.get('fillet_surface_type')

        if r is None:
            return 'sharp'

        # 滚弯判定：R > 10 * 板厚 且是圆柱面
        if fl_type == 'Cylinder' and r > 10 * self.thickness:
            return 'arc_bend'

        # 常规折弯
        if angle < 5.0:
            return 'flat'  # 几乎平的
        elif angle < 135.0:
            return 'regular'
        else:
            return 'large_angle'

    # ── 自由边过滤 ──

    def _find_free_edges(self, bends: list) -> list:
        """过滤自由边 — 外轮廓切割边，不参与折弯

        自由边判定：
        1. 只属于一个面的边（真正的自由边界）
        2. 属于两个共面平面且无圆角面的边
        3. 属于平面+非折弯面的边（如平面+孔洞面）

        这些边被排除，不作为折弯边。
        """
        # 收集所有参与折弯的边
        bend_edges = set()
        bend_faces = set()
        for b in bends:
            bend_faces.add(b['plane_a'])
            bend_faces.add(b['plane_b'])
            if b.get('fillet_face'):
                bend_faces.add(b['fillet_face'])
            for ei in b.get('shared_edges_between_planes', []):
                bend_edges.add(ei)
            for ei in b.get('tangent_edges_a', []):
                bend_edges.add(ei)
            for ei in b.get('tangent_edges_b', []):
                bend_edges.add(ei)

        free_edges = []

        for edge in self.edges:
            ei = edge['edge_idx']
            adj_faces = edge.get('adjacent_faces', [])

            # 规则 1: 自由边 — 只属于 1 个面
            if len(adj_faces) < 2:
                free_edges.append({
                    'edge_idx': ei,
                    'reason': 'boundary_edge',
                    'adjacent_faces': adj_faces,
                })
                continue

            # 规则 2: 属于 2 个面但不是折弯边
            if ei not in bend_edges and len(adj_faces) == 2:
                # 检查两个面是否都是平面
                f0, f1 = adj_faces[0], adj_faces[1]
                f0_type = self.faces[f0]['surface_type']
                f1_type = self.faces[f1]['surface_type']

                if f0_type == 'Plane' and f1_type == 'Plane':
                    # 两个平面共面 -> 不是折弯，是接缝
                    n0 = self._get_face_normal(f0)
                    n1 = self._get_face_normal(f1)
                    if n0 and n1:
                        angle = math.degrees(_vec_angle_between(n0, n1))
                        if angle < 0.5:  # 共面
                            free_edges.append({
                                'edge_idx': ei,
                                'reason': 'coplanar_planes',
                                'adjacent_faces': adj_faces,
                                'angle_deg': round(angle, 2),
                            })
                            continue

                # 任何面不是平面，且不在折弯边中 -> 可能是外轮廓
                if f0 not in bend_faces or f1 not in bend_faces:
                    free_edges.append({
                        'edge_idx': ei,
                        'reason': 'non_bend_edge',
                        'adjacent_faces': adj_faces,
                    })
                    continue

        return free_edges

    # ── 折弯图 ──

    def _build_bend_graph(self, bends: list) -> dict:
        """构建折弯图：每个平面面通过哪些折弯连接到哪些面"""
        graph = defaultdict(list)
        for b in bends:
            pa, pb = b['plane_a'], b['plane_b']
            graph[pa].append({
                'to_face': pb,
                'bend_angle_deg': b['bend_angle_deg'],
                'bend_radius': b['bend_radius'],
                'bend_type': b['bend_type'],
            })
            graph[pb].append({
                'to_face': pa,
                'bend_angle_deg': b['bend_angle_deg'],
                'bend_radius': b['bend_radius'],
                'bend_type': b['bend_type'],
            })
        return {k: v for k, v in graph.items()}

    # ── 几何计算辅助 ──

    def _get_face_normal(self, fi: int):
        """获取平面的法向量"""
        if fi >= len(self.faces):
            return None
        f = self.faces[fi]
        if f['surface_type'] != 'Plane':
            return None
        return f['surface_params'].get('normal')

    def _get_face_origin(self, fi: int):
        """获取平面的原点"""
        if fi >= len(self.faces):
            return None
        f = self.faces[fi]
        return f['surface_params'].get('origin')

    def _shared_edges_between_faces(self, fi: int, fj: int) -> list:
        """返回两面之间的共享边列表"""
        edges_i = self._face_to_edges.get(fi, set())
        edges_j = self._face_to_edges.get(fj, set())
        return list(edges_i & edges_j)

    def _infer_axis_from_edges(self, edge_indices: list):
        """从共享边推断折弯轴线方向

        取所有直线边的方向；如果有多条，取最长的。
        """
        if not edge_indices:
            return None

        line_dirs = []
        for ei in edge_indices:
            if ei < len(self.edges):
                edge = self.edges[ei]
                if edge['curve_type'] == 'Line':
                    d = edge['curve_params'].get('direction')
                    length = edge['curve_params'].get('length', 0)
                    if d:
                        line_dirs.append((length, d))

        if not line_dirs:
            return None

        # 取最长直线的方向
        line_dirs.sort(key=lambda x: -x[0])
        return line_dirs[0][1]

    def _midpoint_of_edges(self, edge_indices: list):
        """计算多条边中点的平均值"""
        points = []
        for ei in edge_indices:
            if ei < len(self.edges):
                edge = self.edges[ei]
                p1 = edge.get('start_vertex')
                p2 = edge.get('end_vertex')
                if p1 and p2:
                    points.append(_vec_midpoint(p1, p2))

        if not points:
            return (0.0, 0.0, 0.0)

        n = len(points)
        return tuple(sum(p[i] for p in points) / n for i in range(3))

    def _compute_bend_origin(self, pa: int, pb: int,
                              na: tuple, nb: tuple,
                              axis_dir: tuple,
                              fl_info: dict,
                              shared_edges_ab: list = None) -> tuple:
        """计算折弯轴线原点

        优先级：
        1. 两面共享边的中点（最可靠，直接在实际折弯边上）
        2. 两面交线 + 圆角半径偏移
        3. 圆角面轴线原点回退

        返回: (x, y, z) 折弯轴线上的一点
        """
        # ── 优先级 1：共享边中点 ──
        if shared_edges_ab:
            edge_midpoints = []
            for ei in shared_edges_ab:
                edge = self.edges[ei] if ei < len(self.edges) else {}
                sv = edge.get('start_vertex')
                ev = edge.get('end_vertex')
                if sv and ev and len(sv) == 3 and len(ev) == 3:
                    edge_midpoints.append((
                        (sv[0] + ev[0]) / 2,
                        (sv[1] + ev[1]) / 2,
                        (sv[2] + ev[2]) / 2,
                    ))
            if edge_midpoints:
                # 返回所有共享边中点的平均值
                n = len(edge_midpoints)
                return (
                    sum(p[0] for p in edge_midpoints) / n,
                    sum(p[1] for p in edge_midpoints) / n,
                    sum(p[2] for p in edge_midpoints) / n,
                )

        # ── 优先级 2：平面交线计算 ──
        intersection_dir = _vec_normalize(_vec_cross(na, nb))
        if _vec_length(intersection_dir) < 1e-6:
            # 平面平行，无交线 -> 回退
            return fl_info.get('axis_origin', (0.0, 0.0, 0.0))

        oa = self._get_face_origin(pa)
        ob = self._get_face_origin(pb)

        if oa is None or ob is None:
            return fl_info.get('axis_origin', (0.0, 0.0, 0.0))

        da = _vec_dot(na, oa)
        db = _vec_dot(nb, ob)
        dot_nab = _vec_dot(na, nb)
        denom = 1.0 - dot_nab * dot_nab

        if abs(denom) < 1e-12:
            return fl_info.get('axis_origin', (0.0, 0.0, 0.0))

        alpha = (da - db * dot_nab) / denom
        beta = (db - da * dot_nab) / denom

        intersection_point = (
            alpha * na[0] + beta * nb[0],
            alpha * na[1] + beta * nb[1],
            alpha * na[2] + beta * nb[2],
        )

        # ── 优先级 3：圆角面轴线原点回退 ──
        return fl_info.get('axis_origin', intersection_point)

    # ── 信息输出 ──

    def summarize(self, result: dict = None) -> str:
        """生成折弯识别结果的文本摘要"""
        if result is None:
            result = self.detect()

        lines = []
        lines.append(f"折弯总数: {result['total_bends']}")
        lines.append(f"自由边数: {len(result['free_edges'])}")
        lines.append("")

        # 按类型统计
        type_counts = defaultdict(int)
        for b in result['bends']:
            type_counts[b['bend_type']] += 1
        lines.append("折弯类型分布:")
        for bt, cnt in sorted(type_counts.items(), key=lambda x: -x[1]):
            lines.append(f"  {bt}: {cnt} 个")

        # 详细信息
        lines.append("")
        lines.append(f"{'#':>3} {'A':>5} {'B':>5} {'angle':>8} {'R(mm)':>8} {'type':>12} {'method':>16}")
        lines.append("-" * 70)
        for i, b in enumerate(result['bends']):
            r_str = f"{b['bend_radius']:.1f}" if b['bend_radius'] else "N/A"
            lines.append(
                f"{i:>3} {b['plane_a']:>5} {b['plane_b']:>5} "
                f"{b['bend_angle_deg']:>7.1f} {r_str:>8} "
                f"{b['bend_type']:>12} {b['detection_method']:>16}"
            )

        # 自由边摘要
        if result['free_edges']:
            lines.append("")
            lines.append(f"自由边 (前 10 条):")
            reason_counts = defaultdict(int)
            for fe in result['free_edges']:
                reason_counts[fe['reason']] += 1
            for reason, cnt in sorted(reason_counts.items()):
                lines.append(f"  {reason}: {cnt} 条")

        return "\n".join(lines)


# ── 测试入口 ──

if __name__ == '__main__':
    from brep_topology import BrepTopologyExtractor

    test_dir = r'd:\Users\liyis\Desktop\pythonproject\py06再实验\三维数模'

    for model_name in ['YA-1131-505.stp', '2ROM31-6061 ---.stp']:
        filepath = os.path.join(test_dir, model_name)
        if not os.path.exists(filepath):
            print(f"跳过: {model_name} (文件不存在)")
            continue

        print(f"\n{'='*70}")
        print(f"  Step 2 折弯识别: {model_name}")
        print(f"{'='*70}")

        # Step 1
        extractor = BrepTopologyExtractor(filepath)
        topo = extractor.extract()

        # Step 2
        detector = BendDetector(topo)
        result = detector.detect()

        print(detector.summarize(result))

        # 详细输出每个折弯
        print(f"\n[详细] 折弯详情:")
        for i, b in enumerate(result['bends']):
            print(f"\n  --- Bend #{i} ---")
            print(f"  Planes:       face#{b['plane_a']} <-> face#{b['plane_b']}")
            print(f"  Fillet:       face#{b['fillet_face']} ({b['fillet_surface_type']})")
            print(f"  Type:         {b['bend_type']}")
            print(f"  Angle:        {b['bend_angle_deg']} deg")
            print(f"  Radius:       {b['bend_radius']} mm")
            print(f"  Axis dir:     {b['bend_axis_direction']}")
            print(f"  Axis origin:  {b['bend_axis_origin']}")
            print(f"  Tangent A:    {b['tangent_edges_a']}")
            print(f"  Tangent B:    {b['tangent_edges_b']}")
            print(f"  Shared(AB):   {b['shared_edges_between_planes']}")
            print(f"  Detection:    {b['detection_method']}")

        # 邻接图连通性检查
        graph = result['bend_graph']
        if graph:
            # BFS 检查所有平面是否连通
            all_bend_planes = set()
            for b in result['bends']:
                all_bend_planes.add(b['plane_a'])
                all_bend_planes.add(b['plane_b'])

            if all_bend_planes:
                start = next(iter(all_bend_planes))
                visited = set()
                stack = [start]
                while stack:
                    fi = stack.pop()
                    if fi in visited:
                        continue
                    visited.add(fi)
                    for edge in graph.get(fi, []):
                        if edge['to_face'] not in visited:
                            stack.append(edge['to_face'])

                unvisited = all_bend_planes - visited
                print(f"\n[连通性] 折弯图连通性:")
                print(f"  参与折弯的平面: {len(all_bend_planes)} 个")
                print(f"  从 face#{start} 可到达: {len(visited)} 个")
                if unvisited:
                    print(f"  未到达: {sorted(unvisited)}")

    print(f"\n{'='*70}")
    print("  [OK] Step 2 折弯识别完成")
    print(f"{'='*70}")
