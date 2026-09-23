"""
SliceAnalyzer — 网格切片法毛料计算引擎

沿指定方向生成等距平行平面，与三角网格求交得截面轮廓线，
取轮廓线弧长（拉直长度）的最大值作为展开尺寸。
"""

import math


# ── 向量工具 ──

def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_scale(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


def _vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vec_length(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _vec_normalize(v):
    l = _vec_length(v)
    if l < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / l, v[1] / l, v[2] / l)


def _vec_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _vec_dist(a, b):
    return _vec_length(_vec_sub(a, b))


# ── 主类 ──

class MeshSlicer:
    """三角网格切片器

    输入:
        direction: (dx, dy, dz)
            切片平面的法线方向（即用户选中的展开边方向）
        origin: (ox, oy, oz)
            切片范围的参考原点（选中边的中点）

    用法:
        slicer = MeshSlicer(direction=(0, 1, 0), origin=(50, 0, 30))
        result = slicer.slice(num_slices=50)
        print(f"拉直长度: {result['best_length']:.2f} mm")
    """

    def __init__(self, direction: tuple, origin: tuple = None):
        self.direction = _vec_normalize(direction)
        self.origin = origin or (0.0, 0.0, 0.0)

        # 内部数据
        self._vertices = []   # [(x,y,z), ...]
        self._triangles = []  # [(i0,i1,i2), ...]

    # ── 数据加载 ──

    def add_solid(self, vertices: list, triangles: list):
        """添加一个 solid 的网格数据

        Args:
            vertices: flat list [x0,y0,z0, x1,y1,z1, ...]
            triangles: flat list [i0,i1,i2, ...] 全局顶点索引（0-based）
        """
        base = len(self._vertices)

        # 解析顶点
        for i in range(0, len(vertices), 3):
            v = (vertices[i], vertices[i + 1], vertices[i + 2])
            self._vertices.append(v)

        # 解析三角形（索引偏移到全局）
        for i in range(0, len(triangles), 3):
            i0 = base + triangles[i]
            i1 = base + triangles[i + 1]
            i2 = base + triangles[i + 2]
            self._triangles.append((i0, i1, i2))

    def add_solids_from_raw(self, raw: dict):
        """从 StepReader.read() 的返回结果加载所有 solid

        Args:
            raw: StepReader.read() 的返回字典，包含 raw['solids']
        """
        for solid in raw.get('solids', []):
            self.add_solid(solid.get('vertices', []), solid.get('triangles', []))

    @property
    def vertex_count(self) -> int:
        return len(self._vertices)

    @property
    def triangle_count(self) -> int:
        return len(self._triangles)

    # ── 切片 ──

    def slice(self, num_slices: int = 50, margin: float = 5.0) -> dict:
        """执行切片，返回所有切片数据

        Args:
            num_slices: 切片数量
            margin: 两端扩展距离 (mm)，确保覆盖完整模型

        Returns:
            dict: {
                'slice_positions': [t0, t1, ...],
                'slice_lengths': [L0, L1, ...],
                'best_index': int,
                'best_length': float,
                'best_contour_3d': [[x,y,z], ...],
                'best_contour_2d': [[u,v], ...],
                'direction': (dx, dy, dz),
                'angle_deg': float,
            }
        """
        if not self._vertices or not self._triangles:
            return {
                'slice_positions': [], 'slice_lengths': [],
                'best_index': -1, 'best_length': 0.0,
                'best_contour_3d': [], 'best_contour_2d': [],
                'direction': self.direction, 'angle_deg': 0.0,
            }

        # 1. 计算顶点在 direction 上的投影范围
        projs = [_vec_dot(v, self.direction) for v in self._vertices]
        t_min, t_max = min(projs), max(projs)
        t_range = t_max - t_min

        if t_range < 0.01:
            # 模型沿方向无厚度 → 退化为单平面
            t_min -= 1.0
            t_max += 1.0
            t_range = t_max - t_min

        # 扩展 margin 确保覆盖
        t_min -= margin
        t_max += margin
        t_range = t_max - t_min

        # 切片步长
        if num_slices <= 1:
            step = t_range
        else:
            step = t_range / (num_slices - 1)

        # 2. 构建平面的 2D 投影基（用于生成 2D 轮廓）
        u_axis, v_axis = self._build_ortho_basis(self.direction)

        # 3. 对每个切片平面求交
        slice_positions = []
        slice_lengths = []
        all_slice_contours_3d = []
        all_slice_contours_2d = []

        best_index = -1
        best_length = -1.0
        EPS = 1e-9

        for si in range(num_slices):
            t = t_min + si * step
            plane_origin = _vec_add(self.origin, _vec_scale(self.direction, t - _vec_dot(self.origin, self.direction)))

            # 收集交线段
            segments = []
            for i0, i1, i2 in self._triangles:
                p0 = self._vertices[i0]
                p1 = self._vertices[i1]
                p2 = self._vertices[i2]
                seg = self._intersect_triangle_plane(
                    (p0, p1, p2), plane_origin, self.direction
                )
                if seg is not None:
                    segments.append(seg)

            if not segments:
                slice_positions.append(t)
                slice_lengths.append(0.0)
                all_slice_contours_3d.append([])
                all_slice_contours_2d.append([])
                continue

            # 链接成轮廓
            contours = self._chain_segments(segments, tolerance=0.05)

            if not contours:
                slice_positions.append(t)
                slice_lengths.append(0.0)
                all_slice_contours_3d.append([])
                all_slice_contours_2d.append([])
                continue

            # 取最长轮廓的弧长
            best_contour = max(contours, key=lambda c: self._arc_length(c))
            arc_len = self._arc_length(best_contour)

            slice_positions.append(t)
            slice_lengths.append(arc_len)
            all_slice_contours_3d.append(best_contour)

            # 投影到 2D
            contour_2d = [
                (_vec_dot(_vec_sub(pt, plane_origin), u_axis),
                 _vec_dot(_vec_sub(pt, plane_origin), v_axis))
                for pt in best_contour
            ]
            all_slice_contours_2d.append(contour_2d)

            if arc_len > best_length:
                best_length = arc_len
                best_index = si

        # 角度: direction 与 X 轴的夹角
        angle_rad = math.atan2(self.direction[1], self.direction[0])
        angle_deg = math.degrees(angle_rad)

        result = {
            'slice_positions': slice_positions,
            'slice_lengths': slice_lengths,
            'best_index': best_index,
            'best_length': round(best_length, 4) if best_length >= 0 else 0.0,
            'best_contour_3d': all_slice_contours_3d[best_index] if best_index >= 0 else [],
            'best_contour_2d': all_slice_contours_2d[best_index] if best_index >= 0 else [],
            'direction': self.direction,
            'angle_deg': round(angle_deg, 4),
        }
        return result

    # ── 三角形与平面求交 ──

    @staticmethod
    def _intersect_triangle_plane(tri_pts, plane_origin, plane_normal):
        """单三角形与平面求交

        Args:
            tri_pts: (p0, p1, p2)  三个顶点坐标 tuple
            plane_origin: (ox, oy, oz)
            plane_normal: (nx, ny, nz)  单位法向量

        Returns:
            None | ((x0,y0,z0), (x1,y1,z1))  两个交点组成的线段
        """
        p0, p1, p2 = tri_pts
        n = plane_normal
        o = plane_origin

        d0 = _vec_dot(_vec_sub(p0, o), n)
        d1 = _vec_dot(_vec_sub(p1, o), n)
        d2 = _vec_dot(_vec_sub(p2, o), n)

        EPS = 1e-9
        above = 0
        below = 0
        on_plane = 0

        for d in (d0, d1, d2):
            if d > EPS:
                above += 1
            elif d < -EPS:
                below += 1
            else:
                on_plane += 1

        # 全部同侧 → 无交
        if above == 3 or below == 3:
            return None

        # 全部在平面上 → 退化，不处理
        if on_plane == 3:
            return None

        # 一顶点在平面，另外两个同侧 → 仅点接触，忽略
        if on_plane == 1 and (above == 2 or below == 2):
            return None

        # 两顶点在平面 → 这两点连线就是截面轮廓的一部分
        # 为避免相邻三角形产生重复线段，只在第三顶点在上方时输出
        # （下方的情况由共享该边的相邻三角形处理，其第三顶点必在上方）
        if on_plane == 2:
            if below == 1:
                return None
            # above == 1: 第三顶点在上方，正常输出该边

        # 计算交点：找出异侧顶点对
        edges = [(p0, p1, d0, d1), (p1, p2, d1, d2), (p2, p0, d2, d0)]
        intersections = []

        for pa, pb, da, db in edges:
            # 顶点在平面上 → 交点即顶点本身
            if abs(da) <= EPS:
                pt = pa
                # 避免重复（已在 list 中则跳过）
                intersections.append(pt)
            elif abs(db) <= EPS:
                pt = pb
                intersections.append(pt)
            elif da * db < 0:
                # 异侧 → 线性插值
                t = abs(da) / (abs(da) + abs(db))
                pt = _vec_add(pa, _vec_scale(_vec_sub(pb, pa), t))
                intersections.append(pt)

        if len(intersections) == 2:
            return (intersections[0], intersections[1])
        elif len(intersections) > 2:
            # 取前两个唯一点
            return (intersections[0], intersections[1])

        return None

    # ── 线段链接 ──

    @staticmethod
    def _chain_segments(segments: list, tolerance: float = 0.05) -> list:
        """将无序线段链接成连续轮廓线

        Args:
            segments: [(start, end), ...]  start/end 为 (x,y,z)
            tolerance: 端点匹配容差 (mm)

        Returns:
            [[(x,y,z), ...], ...]  多条轮廓线
        """
        if not segments:
            return []

        # 量化函数
        inv_tol = 1.0 / tolerance

        def _key(pt):
            return (round(pt[0] * inv_tol), round(pt[1] * inv_tol), round(pt[2] * inv_tol))

        # 建端点索引: key → [(seg_idx, is_start), ...]
        endpoint_map = {}
        for idx, (s, e) in enumerate(segments):
            for pt, is_start in ((s, True), (e, False)):
                k = _key(pt)
                if k not in endpoint_map:
                    endpoint_map[k] = []
                endpoint_map[k].append((idx, is_start))

        # 贪心链接
        used = set()
        contours = []

        for start_idx, (s, e) in enumerate(segments):
            if start_idx in used:
                continue

            # 开启新轮廓
            contour = [s, e]
            used.add(start_idx)
            current_pt = e

            while True:
                # 找下一个段
                k = _key(current_pt)
                candidates = endpoint_map.get(k, [])
                found = False
                for seg_idx, is_start in candidates:
                    if seg_idx in used:
                        continue
                    used.add(seg_idx)
                    seg = segments[seg_idx]
                    if is_start:
                        next_pt = seg[1]
                        contour.append(next_pt)
                        current_pt = next_pt
                    else:
                        next_pt = seg[0]
                        contour.append(next_pt)
                        current_pt = next_pt
                    found = True
                    break

                if not found:
                    break

                # 闭环检测
                if _key(current_pt) == _key(contour[0]):
                    break

                # 安全上限
                if len(contour) > len(segments) * 2:
                    break

            contours.append(contour)

        # 过滤极短的轮廓（片厚方向的边）
        filtered = [c for c in contours if len(c) >= 3]
        return filtered

    # ── 弧长 ──

    @staticmethod
    def _arc_length(polyline: list) -> float:
        """多段线总弧长"""
        if len(polyline) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(polyline)):
            total += _vec_dist(polyline[i - 1], polyline[i])
        return total

    # ── 单平面求交 ──

    def intersect_plane(self, plane_origin: tuple, plane_normal: tuple) -> dict:
        """单个裁切平面与网格求交

        Args:
            plane_origin: (ox, oy, oz) 平面上一点
            plane_normal: (nx, ny, nz) 平面法向量（会自动归一化）

        Returns:
            dict: {
                'contours_3d': [[(x,y,z), ...], ...],
                'contours_2d': [[(u,v), ...], ...],
                'arc_lengths': [float, ...],
                'best_contour_index': int,
                'best_contour_3d': [(x,y,z), ...],
                'best_contour_2d': [(u,v), ...],
                'best_length': float,
                'direction': (dx, dy, dz),
            }
        """
        normal = _vec_normalize(plane_normal)
        if _vec_length(normal) < 1e-12:
            return {
                'contours_3d': [], 'contours_2d': [], 'arc_lengths': [],
                'best_contour_index': -1, 'best_contour_3d': [], 'best_contour_2d': [],
                'best_length': 0.0, 'direction': normal,
                'all_closed': True, 'gap_count': 0, 'contour_count': 0, 'closure_flags': [],
            }

        if not self._vertices or not self._triangles:
            return {
                'contours_3d': [], 'contours_2d': [], 'arc_lengths': [],
                'best_contour_index': -1, 'best_contour_3d': [], 'best_contour_2d': [],
                'best_length': 0.0, 'direction': normal,
                'all_closed': True, 'gap_count': 0, 'contour_count': 0, 'closure_flags': [],
            }

        # 1. 对所有三角形求交线段
        segments = []
        for i0, i1, i2 in self._triangles:
            p0 = self._vertices[i0]
            p1 = self._vertices[i1]
            p2 = self._vertices[i2]
            seg = self._intersect_triangle_plane((p0, p1, p2), plane_origin, normal)
            if seg is not None:
                segments.append(seg)

        if not segments:
            return {
                'contours_3d': [], 'contours_2d': [], 'arc_lengths': [],
                'best_contour_index': -1, 'best_contour_3d': [], 'best_contour_2d': [],
                'best_length': 0.0, 'direction': normal,
                'all_closed': True, 'gap_count': 0, 'contour_count': 0, 'closure_flags': [],
            }

        # 2. 链接成轮廓
        contours = self._chain_segments(segments, tolerance=0.05)
        if not contours:
            return {
                'contours_3d': [], 'contours_2d': [], 'arc_lengths': [],
                'best_contour_index': -1, 'best_contour_3d': [], 'best_contour_2d': [],
                'best_length': 0.0, 'direction': normal,
                'all_closed': True, 'gap_count': 0, 'contour_count': 0, 'closure_flags': [],
            }

        # 3. 构建 2D 投影基
        u_axis, v_axis = self._build_ortho_basis(normal)

        # 4. 计算每条轮廓的弧长、2D 投影、闭合状态
        contours_2d = []
        arc_lengths = []
        closure_flags = []
        closure_tolerance = 0.5  # 首尾距离 < 0.5mm 视为几何闭合
        for contour in contours:
            arc_len = self._arc_length(contour)
            arc_lengths.append(arc_len)
            c2d = [
                (_vec_dot(_vec_sub(pt, plane_origin), u_axis),
                 _vec_dot(_vec_sub(pt, plane_origin), v_axis))
                for pt in contour
            ]
            contours_2d.append(c2d)
            # 几何闭合检测
            if len(contour) >= 3:
                gap = _vec_dist(contour[0], contour[-1])
                closure_flags.append(gap < closure_tolerance)
            else:
                closure_flags.append(False)

        # 5. 取最长轮廓
        if arc_lengths:
            best_idx = max(range(len(arc_lengths)), key=lambda i: arc_lengths[i])
        else:
            best_idx = -1

        # 6. 轮廓闭合判断（基于轮廓数量）
        # 正常钣金截面 = 1个四边形；切到孔洞 = 2+个四边形
        contour_count = len(contours)
        all_geo_closed = all(closure_flags) if closure_flags else True
        # 单轮廓且几何闭合 = 完整封闭；多轮廓 = 有断口/孔
        is_single_profile = (contour_count == 1)
        has_gaps = (contour_count >= 2)
        gap_count = contour_count - 1 if has_gaps else 0

        return {
            'contours_3d': contours,
            'contours_2d': contours_2d,
            'arc_lengths': [round(l, 4) for l in arc_lengths],
            'best_contour_index': best_idx,
            'best_contour_3d': contours[best_idx] if best_idx >= 0 else [],
            'best_contour_2d': contours_2d[best_idx] if best_idx >= 0 else [],
            'best_length': round(arc_lengths[best_idx], 4) if best_idx >= 0 else 0.0,
            'direction': normal,
            'all_closed': not has_gaps,  # 单轮廓=封闭，多轮廓=有断口
            'gap_count': gap_count,
            'contour_count': contour_count,
            'closure_flags': closure_flags,
        }

    # ── 平面拟合 ──

    @staticmethod
    def fit_plane_from_points(points: list) -> tuple:
        """从一组 3D 点拟合最佳平面（PCA 法）

        Args:
            points: [(x, y, z), ...]  至少 3 个不共线点

        Returns:
            (origin, normal)
                origin: (ox, oy, oz) 点集质心
                normal: (nx, ny, nz) 单位法向量（最小特征值方向）

        若点数不足或退化，返回 (None, None)
        """
        n = len(points)
        if n < 3:
            return None, None

        # 质心
        cx = sum(p[0] for p in points) / n
        cy = sum(p[1] for p in points) / n
        cz = sum(p[2] for p in points) / n
        centroid = (cx, cy, cz)

        # 3×3 协方差矩阵
        c00 = c01 = c02 = c11 = c12 = c22 = 0.0
        for x, y, z in points:
            dx, dy, dz = x - cx, y - cy, z - cz
            c00 += dx * dx
            c01 += dx * dy
            c02 += dx * dz
            c11 += dy * dy
            c12 += dy * dz
            c22 += dz * dz

        # 幂迭代法求最小特征向量
        # 先求最大特征向量，然后在正交补中求次大，最后剩下的就是最小
        # 简化：直接用幂迭代求最大，再从协方差矩阵减去其贡献，再求最大

        def mat_vec_mul(vx, vy, vz):
            return (
                c00 * vx + c01 * vy + c02 * vz,
                c01 * vx + c11 * vy + c12 * vz,
                c02 * vx + c12 * vy + c22 * vz,
            )

        # 幂迭代求最大特征向量
        def power_iteration(init):
            vx, vy, vz = init
            for _ in range(50):
                mx, my, mz = mat_vec_mul(vx, vy, vz)
                norm = math.sqrt(mx * mx + my * my + mz * mz)
                if norm < 1e-12:
                    break
                vx, vy, vz = mx / norm, my / norm, mz / norm
            return (vx, vy, vz)

        # 第一主成分（最大特征向量）
        eig1 = power_iteration((1.0, 0.0, 0.0))

        # 从协方差矩阵减去第一主成分的贡献，求第二主成分
        # 简化：在与 eig1 正交的平面上找一个向量，用原协方差矩阵迭代并投影
        # 选一个不平行于 eig1 的向量
        if abs(eig1[0]) < 0.9:
            ref = (1.0, 0.0, 0.0)
        elif abs(eig1[1]) < 0.9:
            ref = (0.0, 1.0, 0.0)
        else:
            ref = (0.0, 0.0, 1.0)
        # 正交化
        dot_ref = eig1[0] * ref[0] + eig1[1] * ref[1] + eig1[2] * ref[2]
        init2 = (ref[0] - dot_ref * eig1[0],
                 ref[1] - dot_ref * eig1[1],
                 ref[2] - dot_ref * eig1[2])
        norm2 = math.sqrt(init2[0]**2 + init2[1]**2 + init2[2]**2)
        if norm2 < 1e-12:
            # 退化：所有点共线
            return centroid, None
        init2 = (init2[0]/norm2, init2[1]/norm2, init2[2]/norm2)

        # 幂迭代并在每次迭代后投影到正交补
        vx, vy, vz = init2
        for _ in range(50):
            mx, my, mz = mat_vec_mul(vx, vy, vz)
            # 投影掉 eig1 分量
            dot_e1 = mx * eig1[0] + my * eig1[1] + mz * eig1[2]
            mx -= dot_e1 * eig1[0]
            my -= dot_e1 * eig1[1]
            mz -= dot_e1 * eig1[2]
            norm = math.sqrt(mx * mx + my * my + mz * mz)
            if norm < 1e-12:
                break
            vx, vy, vz = mx / norm, my / norm, mz / norm
        eig2 = (vx, vy, vz)

        # 第三主成分 = eig1 × eig2（法向量 = 最小特征向量）
        normal = _vec_cross(eig1, eig2)
        normal = _vec_normalize(normal)

        return centroid, normal

    # ── 正交基 ──

    @staticmethod
    def _build_ortho_basis(normal: tuple) -> tuple:
        """从法向量构建 2D 投影基 (u_axis, v_axis)"""
        n = _vec_normalize(normal)
        # 选一个不共线的参考向量
        ref = (0.0, 1.0, 0.0) if abs(n[0]) < 0.9 else (1.0, 0.0, 0.0)
        u = _vec_normalize(_vec_cross(n, ref))
        v = _vec_cross(n, u)
        return u, v


# ==================== 命令行测试 ====================

if __name__ == '__main__':
    import sys
    import os

    print("=" * 60)
    print("  MeshSlicer 单元测试")
    print("=" * 60)

    # ── Test 1: 简单长方体 (20x10x5 mm) ──
    print("\n[Test 1] 长方体 20x10x5 mm，沿 Y 轴切片")

    # 构建长方体网格（8 顶点 + 12 三角形）
    # X: 0-20, Y: 0-10, Z: 0-5
    w, h, d = 20.0, 10.0, 5.0
    verts_flat = [
        0, 0, 0,  w, 0, 0,  w, h, 0,  0, h, 0,   # 底面 z=0
        0, 0, d,  w, 0, d,  w, h, d,  0, h, d,   # 顶面 z=d
    ]
    # 12 个三角形（6 个面 × 2 个三角形）
    tris_flat = [
        0,1,2, 0,2,3,   # 底面
        4,5,6, 4,6,7,   # 顶面
        0,1,5, 0,5,4,   # 前面
        2,3,7, 2,7,6,   # 后面
        0,3,7, 0,7,4,   # 左面
        1,2,6, 1,6,5,   # 右面
    ]

    slicer = MeshSlicer(direction=(0, 1, 0), origin=(10, 5, 2.5))
    slicer.add_solid(verts_flat, tris_flat)
    result = slicer.slice(num_slices=10)

    print(f"  顶点数: {slicer.vertex_count}")
    print(f"  三角形数: {slicer.triangle_count}")
    print(f"  最佳切片索引: {result['best_index']}")
    print(f"  拉直长度: {result['best_length']:.2f} mm")
    print(f"  切片位置数: {len(result['slice_positions'])}")
    print(f"  所有切片长度: {[f'{l:.2f}' for l in result['slice_lengths']]}")

    # 长方体沿 Y 轴的截面是 20×5 矩形，轮廓周长 = 2*(20+5) = 50mm
    assert result['best_length'] > 0, "ERROR: best_length 应为正值"
    assert abs(result['best_length'] - 50.0) < 3.0, \
        f"ERROR: 预期拉直长度 ~50mm，实际 {result['best_length']:.2f} mm"
    print("  [OK] PASS")

    # ── Test 2: L 形板 ──
    print("\n[Test 2] L 形钣金件，沿折弯轴 (Z) 切片")

    # 简化的 L 形：水平板 (0,0,0)-(20,10,2) + 竖板 (0,0,0)-(2,10,20)
    # 用两个长方体拼接
    verts_l = [
        # 水平板底面
        0, 0, 0,  20, 0, 0,  20, 10, 0,  0, 10, 0,
        # 水平板顶面
        0, 0, 2,  20, 0, 2,  20, 10, 2,  0, 10, 2,
        # 竖板左侧 (x=0)
        0, 0, 0,  2, 0, 0,  2, 10, 0,  0, 10, 0,
        # 竖板右侧
        0, 0, 20,  2, 0, 20,  2, 10, 20,  0, 10, 20,
    ]
    # 简化：只用水平板的前 8 个顶点
    # 重新构建 L 形截面网格
    # 水平部分: x=0-20, y=0-10, z=0-2
    # 竖直部分: x=0-2, y=0-10, z=0-20

    # 用 12 个顶点描述 L 形截面扫掠体
    # 底水平面 z=0
    v = [
        0,0,0,  20,0,0,  20,10,0,  0,10,0,      # 0-3: 底水平面
        0,0,2,  20,0,2,  20,10,2,  0,10,2,      # 4-7: 顶水平面 (z=2)
        0,0,2,  2,0,2,   2,10,2,   0,10,2,       # 8-11: 竖板底面(复用顶面)
        0,0,22, 2,0,22,  2,10,22,  0,10,22,     # 12-15: 竖板顶面 (z=22)
    ]

    # 简化测试：用多个沿Y的切片，取最长轮廓
    t = [
        # 水平板
        0,1,2, 0,2,3,
        4,5,6, 4,6,7,
        0,1,5, 0,5,4,
        2,3,7, 2,7,6,
        0,3,7, 0,7,4,  # 左面
        1,2,6, 1,6,5,  # 右面
        # 竖板
        8,9,10,  8,10,11,
        12,13,14, 12,14,15,
        8,9,13, 8,13,12,
        10,11,15, 10,15,14,
        8,11,15, 8,15,12,
        9,10,14, 9,14,13,
    ]

    slicer2 = MeshSlicer(direction=(0, 1, 0), origin=(1, 5, 11))
    slicer2.add_solid(v, t)
    result2 = slicer2.slice(num_slices=10)

    print(f"  顶点数: {slicer2.vertex_count}")
    print(f"  三角形数: {slicer2.triangle_count}")
    print(f"  拉直长度: {result2['best_length']:.2f} mm")
    print(f"  角度: {result2['angle_deg']:.1f}°")

    # L 形截面：水平 20 + 竖直 20 = 40mm (忽略圆角和厚度)
    assert result2['best_length'] > 0, "ERROR: best_length 应为正值"
    assert 35 < result2['best_length'] < 45, \
        f"ERROR: 预期拉直长度 ~40mm，实际 {result2['best_length']:.2f} mm"
    print("  [OK] PASS")

    # ── Test 3: 空数据 ──
    print("\n[Test 3] 空数据退化测试")
    slicer3 = MeshSlicer(direction=(0, 0, 1))
    result3 = slicer3.slice()
    assert result3['best_length'] == 0.0
    assert result3['best_index'] == -1
    print("  [OK] PASS")

    # ── Test 4: 从实际 STEP 文件测试 ──
    print("\n[Test 4] 实际 STEP 文件测试")
    legacy_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '三维数模')
    if os.path.isdir(legacy_dir):
        stp_files = [f for f in os.listdir(legacy_dir) if f.lower().endswith(('.stp', '.step'))]
        if stp_files:
            from step_reader import StepReader
            test_file = os.path.join(legacy_dir, stp_files[0])
            print(f"  文件: {stp_files[0]}")
            reader = StepReader(linear_deflection=0.5, angular_deflection=0.3)
            raw = reader.read(test_file)

            # 用 bounding box 的长轴作为切片方向
            bbox = raw['bbox']
            size = bbox['size']  # [dx, dy, dz]
            sizes = [(size[0], (1,0,0)), (size[1], (0,1,0)), (size[2], (0,0,1))]
            sizes.sort(key=lambda x: x[0], reverse=True)
            axis_name = {(1,0,0): 'X', (0,1,0): 'Y', (0,0,1): 'Z'}
            # 取最短轴作为切片方向（钣金件厚度方向通常最短）
            short_axis = sizes[-1][1]
            print(f"  切片方向: {axis_name[short_axis]} (最短轴)")

            slicer4 = MeshSlicer(direction=short_axis)
            slicer4.add_solids_from_raw(raw)
            result4 = slicer4.slice(num_slices=30)

            print(f"  总顶点: {slicer4.vertex_count:,}")
            print(f"  总三角形: {slicer4.triangle_count:,}")
            print(f"  拉直长度: {result4['best_length']:.2f} mm")
            print(f"  有效切片: {sum(1 for l in result4['slice_lengths'] if l > 0)} / {len(result4['slice_lengths'])}")
            print("  [OK] PASS (如无异常则通过)")
        else:
            print("  (无 STEP 文件，跳过)")
    else:
        print("  (三维数模目录不存在，跳过)")

    # ── Test 5: intersect_plane 单平面求交 ──
    print("\n[Test 5] intersect_plane 单平面求交 (长方体 20x10x5)")
    slicer5 = MeshSlicer(direction=(0, 0, 1))
    slicer5.add_solid(verts_flat, tris_flat)
    result5 = slicer5.intersect_plane(plane_origin=(10, 5, 2.5), plane_normal=(0, 1, 0))
    print(f"  轮廓数: {len(result5['contours_3d'])}")
    print(f"  弧长列表: {result5['arc_lengths']}")
    print(f"  最佳长度: {result5['best_length']:.2f} mm (预期 ~50mm)")
    assert result5['best_length'] > 0, "ERROR: best_length 应为正值"
    assert abs(result5['best_length'] - 50.0) < 5.0, f"ERROR: 预期 ~50mm，实际 {result5['best_length']:.2f}"
    print("  [OK] PASS")

    # ── Test 6: fit_plane_from_points 平面拟合 ──
    print("\n[Test 6] fit_plane_from_points")
    plane_points = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0), (0.0, 10.0, 0.0), (5.0, 5.0, 0.0)]
    origin, normal = MeshSlicer.fit_plane_from_points(plane_points)
    print(f"  拟合法线: ({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f})")
    assert origin is not None and normal is not None, "ERROR: 应有拟合结果"
    assert abs(normal[2]) > 0.9, f"ERROR: 法线应接近 Z 轴，实际 {normal}"
    print("  [OK] PASS")

    # ── Test 7: 组合测试（用斜切平面模拟选中边拟合） ──
    print("\n[Test 7] 拟合平面 + 斜切长方体")
    # 模拟在长方体侧面上选了 4 个点，拟合出一个斜的裁切面
    # 选点：底面左下、底面右上、顶面右下、顶面左上 → 法线应接近 X 或 Y
    diag_pts = [(0, 0, 0), (20, 0, 5), (20, 10, 0), (0, 10, 5)]
    origin7, normal7 = MeshSlicer.fit_plane_from_points(diag_pts)
    assert origin7 is not None and normal7 is not None, "ERROR: 应有拟合结果"
    print(f"  拟合法线: ({normal7[0]:.4f}, {normal7[1]:.4f}, {normal7[2]:.4f})")
    slicer7 = MeshSlicer(direction=normal7)
    slicer7.add_solid(verts_flat, tris_flat)
    result7 = slicer7.intersect_plane(origin7, normal7)
    print(f"  轮廓数: {len(result7['contours_3d'])}, 弧长: {[f'{l:.1f}' for l in result7['arc_lengths']]}")
    assert result7['best_length'] > 0, "ERROR: 应有交点"
    print("  [OK] PASS")

    print("\n" + "=" * 60)
    print("  全部测试完成")
    print("=" * 60)
