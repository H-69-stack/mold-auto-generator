"""
BREP 拓扑抽取模块 -- Step 1: 模型预处理

从 STEP 文件的原生 BREP 结构中提取：
  - 所有 Face（面片）、Edge（边界边）、Vertex（顶点）
  - 面-边拓扑邻接关系（哪些边围成哪个面、哪两个面共享边）
  - 统一板厚 t（基于邻接面的平行反向面对检测）
  - 面片分类：平面 / 圆柱面 / 圆角面
  - 基准底板（面积最大平面）

输出：结构化拓扑字典，供后续展开步骤使用。
"""

import os
import math
from collections import defaultdict

import cadquery as cq
from OCP.TopExp import TopExp_Explorer, TopExp
from OCP.TopAbs import (
    TopAbs_FACE, TopAbs_EDGE, TopAbs_WIRE, TopAbs_VERTEX,
    TopAbs_SOLID, TopAbs_SHELL,
)
from OCP.TopoDS import TopoDS
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCP.BRep import BRep_Tool
from OCP.BRepTools import BRepTools
from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCP.GeomAbs import (
    GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Cone,
    GeomAbs_Sphere, GeomAbs_Torus,
    GeomAbs_BezierSurface, GeomAbs_BSplineSurface,
    GeomAbs_SurfaceOfRevolution, GeomAbs_SurfaceOfExtrusion,
    GeomAbs_OffsetSurface,
    GeomAbs_Line, GeomAbs_Circle, GeomAbs_Ellipse,
    GeomAbs_BSplineCurve, GeomAbs_BezierCurve, GeomAbs_OtherCurve,
)
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt, gp_Vec
from OCP.TopLoc import TopLoc_Location


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


def _point_to_tuple(pnt):
    """gp_Pnt -> (x, y, z)"""
    return (round(pnt.X(), 6), round(pnt.Y(), 6), round(pnt.Z(), 6))


def _dir_to_tuple(d):
    """gp_Dir -> (x, y, z)"""
    return (round(d.X(), 6), round(d.Y(), 6), round(d.Z(), 6))


# ── 主类 ──

class BrepTopologyExtractor:
    """STEP 模型 BREP 拓扑抽取器

    用法:
        extractor = BrepTopologyExtractor("path/to/model.stp")
        topology = extractor.extract()
    """

    def __init__(self, filepath: str):
        self.filepath = filepath
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"文件不存在: {filepath}")
        self.filename = os.path.basename(filepath)

        # 内部状态
        self._shape = None           # 顶层 TopoDS_Shape
        self._solid = None           # 第一个 Solid（钣金件通常就一个）
        self._face_shapes = {}       # face_idx -> TopoDS_Face
        self._edge_shapes = {}       # edge_idx -> TopoDS_Edge
        self._vertex_shapes = {}     # vertex_idx -> TopoDS_Vertex

    # ── 主入口 ──

    def extract(self) -> dict:
        """执行 BREP 拓扑抽取，返回结构化数据"""
        # 1. 读取 STEP 文件，获取原生形状
        self._read_step()

        # 2. 构建面-边-顶点拓扑图
        self._build_topology_graph()

        # 3. 提取每个面的几何参数
        faces = self._extract_all_faces()

        # 4. 提取每条边的几何参数
        edges = self._extract_all_edges()

        # 5. 提取所有顶点
        vertices = self._extract_all_vertices()

        # 6. 检测统一板厚
        thickness, thickness_pairs = self._detect_shell_thickness(faces)

        # 7. 分类面片
        classification = self._classify_faces(faces)

        # 8. 确定基准底板
        base_face_idx = self._find_base_face(faces, classification['planar_faces'])

        return {
            'filename': self.filename,
            'filepath': self.filepath,
            'faces': faces,
            'edges': edges,
            'vertices': vertices,
            'total_faces': len(faces),
            'total_edges': len(edges),
            'total_vertices': len(vertices),
            'thickness': thickness,
            'thickness_pairs': thickness_pairs,
            'base_face_idx': base_face_idx,
            'classification': classification,
        }

    # ── 步骤 1: 读取 STEP ──

    def _read_step(self):
        """读取 STEP 文件，提取第一个 Solid"""
        result = cq.importers.importStep(self.filepath)
        shapes = result.vals() if hasattr(result, 'vals') else [result]

        # 取第一个 shape
        topo_shape = list(shapes)[0].wrapped if shapes else None
        if topo_shape is None:
            raise ValueError("无法读取 STEP 几何体")

        self._shape = topo_shape

        # 提取第一个 Solid（或 Shell）
        exp = TopExp_Explorer(topo_shape, TopAbs_SOLID)
        if exp.More():
            self._solid = exp.Current()
        else:
            exp = TopExp_Explorer(topo_shape, TopAbs_SHELL)
            if exp.More():
                self._solid = exp.Current()
            else:
                self._solid = topo_shape

    # ── 步骤 2: 构建拓扑图 ──

    def _build_topology_graph(self):
        """构建面-边拓扑邻接图

        核心数据结构:
          - face_shapes:   face_idx -> TopoDS_Face
          - edge_shapes:   edge_idx -> TopoDS_Edge
          - edge_to_faces: edge_idx -> [face_idx, ...]  (通常是 2 个面)
          - face_to_edges: face_idx -> [edge_idx, ...]  (围成该面的边)
        """
        self._face_shapes.clear()
        self._edge_shapes.clear()
        self._vertex_shapes.clear()

        # ── 收集所有 Face ──
        face_exp = TopExp_Explorer(self._solid, TopAbs_FACE)
        face_idx = 0
        while face_exp.More():
            face = TopoDS.Face_s(face_exp.Current())
            self._face_shapes[face_idx] = face
            face_idx += 1
            face_exp.Next()

        # ── 收集所有 Edge ──
        edge_exp = TopExp_Explorer(self._solid, TopAbs_EDGE)
        edge_idx = 0
        while edge_exp.More():
            edge = TopoDS.Edge_s(edge_exp.Current())
            self._edge_shapes[edge_idx] = edge
            edge_idx += 1
            edge_exp.Next()

        # ── 收集所有 Vertex ──
        vert_exp = TopExp_Explorer(self._solid, TopAbs_VERTEX)
        vert_idx = 0
        while vert_exp.More():
            vertex = TopoDS.Vertex_s(vert_exp.Current())
            self._vertex_shapes[vert_idx] = vertex
            vert_idx += 1
            vert_exp.Next()

        # ── 构建 Edge <-> Faces 映射 ──
        #     遍历每个面->每条线框->每条边，用 IsSame() 匹配到已知边列表
        #     区分外边界 (outer wire) 和内孔 (inner wires)
        self._edge_to_faces = defaultdict(list)
        self._face_to_edges = defaultdict(list)
        self._face_outer_edges = defaultdict(list)   # 外边界边
        self._face_inner_edges = defaultdict(list)   # 内孔边

        for fid, face in self._face_shapes.items():
            # 获取外边界线框
            outer_wire = BRepTools.OuterWire_s(face)
            wire_exp = TopExp_Explorer(face, TopAbs_WIRE)
            while wire_exp.More():
                wire = wire_exp.Current()
                is_outer = (not outer_wire.IsNull() and outer_wire.IsSame(wire))
                edge_exp = TopExp_Explorer(wire, TopAbs_EDGE)
                while edge_exp.More():
                    edge = TopoDS.Edge_s(edge_exp.Current())
                    # 在已知边列表中找到匹配的边（IsSame 比较 TShape）
                    for eid, known_edge in self._edge_shapes.items():
                        if known_edge.IsSame(edge):
                            if fid not in self._edge_to_faces[eid]:
                                self._edge_to_faces[eid].append(fid)
                            if eid not in self._face_to_edges[fid]:
                                self._face_to_edges[fid].append(eid)
                            # 区分外边界/内孔
                            target = self._face_outer_edges[fid] if is_outer else self._face_inner_edges[fid]
                            if eid not in target:
                                target.append(eid)
                            break
                    edge_exp.Next()
                wire_exp.Next()

        # ── 构建 Face -> Adjacent Faces（通过共享边） ──
        self._face_adjacency = defaultdict(set)
        for eid, face_list in self._edge_to_faces.items():
            for i, fi in enumerate(face_list):
                for fj in face_list[i+1:]:
                    self._face_adjacency[fi].add(fj)
                    self._face_adjacency[fj].add(fi)

    # ── 步骤 3: 提取面参数 ──

    def _extract_all_faces(self) -> list:
        """提取所有面的几何参数和拓扑信息"""
        faces = []
        for fi in sorted(self._face_shapes.keys()):
            face = self._face_shapes[fi]
            params = self._extract_face_params(face)
            area = self._compute_face_area(face)
            faces.append({
                'face_idx': fi,
                'surface_type': params.get('surface_type', 'Unknown'),
                'surface_params': params,
                'area': round(area, 4),
                'bounding_edges': sorted(self._face_to_edges.get(fi, [])),
                'outer_edges': sorted(self._face_outer_edges.get(fi, [])),
                'inner_edges': sorted(self._face_inner_edges.get(fi, [])),
                'adjacent_faces': sorted(self._face_adjacency.get(fi, [])),
            })
        return faces

    def _extract_face_params(self, face) -> dict:
        """提取单个面的曲面类型和几何参数"""
        try:
            adaptor = BRepAdaptor_Surface(face)
            surf_type = adaptor.GetType()

            if surf_type == GeomAbs_Plane:
                plane = adaptor.Plane()
                ax3 = plane.Position()
                axis = ax3.Axis()
                loc = ax3.Location()
                return {
                    'surface_type': 'Plane',
                    'normal': _dir_to_tuple(axis.Direction()),
                    'origin': _point_to_tuple(loc),
                }

            elif surf_type == GeomAbs_Cylinder:
                cyl = adaptor.Cylinder()
                ax3 = cyl.Position()
                axis = ax3.Axis()
                loc = axis.Location()
                radius = cyl.Radius()
                return {
                    'surface_type': 'Cylinder',
                    'radius': round(radius, 4),
                    'diameter': round(radius * 2, 4),
                    'axis_origin': _point_to_tuple(loc),
                    'axis_direction': _dir_to_tuple(axis.Direction()),
                }

            elif surf_type == GeomAbs_Torus:
                torus = adaptor.Torus()
                ax3 = torus.Position()
                axis = ax3.Axis()
                loc = axis.Location()
                major_r = torus.MajorRadius()
                minor_r = torus.MinorRadius()
                return {
                    'surface_type': 'Torus',
                    'major_radius': round(major_r, 4),
                    'minor_radius': round(minor_r, 4),
                    'axis_origin': _point_to_tuple(loc),
                    'axis_direction': _dir_to_tuple(axis.Direction()),
                }

            elif surf_type == GeomAbs_Cone:
                cone = adaptor.Cone()
                ax3 = cone.Position()
                axis = ax3.Axis()
                loc = axis.Location()
                semi_angle = cone.SemiAngle()
                ref_radius = cone.RefRadius()
                return {
                    'surface_type': 'Cone',
                    'semi_angle_rad': round(semi_angle, 6),
                    'semi_angle_deg': round(math.degrees(semi_angle), 2),
                    'ref_radius': round(ref_radius, 4),
                    'axis_origin': _point_to_tuple(loc),
                    'axis_direction': _dir_to_tuple(axis.Direction()),
                }

            elif surf_type == GeomAbs_Sphere:
                sphere = adaptor.Sphere()
                center = sphere.Location()
                radius = sphere.Radius()
                return {
                    'surface_type': 'Sphere',
                    'radius': round(radius, 4),
                    'diameter': round(radius * 2, 4),
                    'center': _point_to_tuple(center),
                }

            else:
                type_names = {
                    GeomAbs_BezierSurface: 'Bezier',
                    GeomAbs_BSplineSurface: 'BSpline',
                    GeomAbs_SurfaceOfRevolution: 'Revolution',
                    GeomAbs_SurfaceOfExtrusion: 'Extrusion',
                    GeomAbs_OffsetSurface: 'Offset',
                }
                return {'surface_type': type_names.get(surf_type, f'Type_{surf_type}')}

        except Exception as e:
            return {'surface_type': 'Unknown', 'error': str(e)}

    def _compute_face_area(self, face) -> float:
        """计算面的表面积"""
        try:
            props = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, props)
            return props.Mass()
        except Exception:
            return 0.0

    # ── 步骤 4: 提取边参数 ──

    def _extract_all_edges(self) -> list:
        """提取所有边的几何参数和拓扑信息"""
        edges = []
        for ei in sorted(self._edge_shapes.keys()):
            edge = self._edge_shapes[ei]
            params = self._extract_edge_params(edge)
            verts = self._extract_edge_vertices(edge)
            edges.append({
                'edge_idx': ei,
                'curve_type': params.get('curve_type', 'Unknown'),
                'curve_params': params,
                'start_vertex': verts[0] if verts else None,
                'end_vertex': verts[1] if len(verts) > 1 else None,
                'adjacent_faces': sorted(self._edge_to_faces.get(ei, [])),
            })
        return edges

    def _extract_edge_params(self, edge) -> dict:
        """提取边的曲线类型和几何参数"""
        try:
            adaptor = BRepAdaptor_Curve(edge)
            curve_type = adaptor.GetType()
            first = adaptor.FirstParameter()
            last = adaptor.LastParameter()

            if curve_type == GeomAbs_Line:
                line = adaptor.Line()
                direction = line.Direction()
                p1 = gp_Pnt(); adaptor.D0(first, p1)
                p2 = gp_Pnt(); adaptor.D0(last, p2)
                return {
                    'curve_type': 'Line',
                    'direction': _dir_to_tuple(direction),
                    'start_point': _point_to_tuple(p1),
                    'end_point': _point_to_tuple(p2),
                    'length': round(p1.Distance(p2), 4),
                }

            elif curve_type == GeomAbs_Circle:
                circle = adaptor.Circle()
                radius = circle.Radius()
                center = circle.Location()
                axis = circle.Axis()
                return {
                    'curve_type': 'Circle',
                    'radius': round(radius, 4),
                    'diameter': round(radius * 2, 4),
                    'center': _point_to_tuple(center),
                    'axis_direction': _dir_to_tuple(axis.Direction()),
                    'circumference': round(2 * math.pi * radius, 4),
                }

            elif curve_type == GeomAbs_Ellipse:
                ellipse = adaptor.Ellipse()
                major_r = ellipse.MajorRadius()
                minor_r = ellipse.MinorRadius()
                center = ellipse.Location()
                return {
                    'curve_type': 'Ellipse',
                    'major_radius': round(major_r, 4),
                    'minor_radius': round(minor_r, 4),
                    'center': _point_to_tuple(center),
                }

            else:
                type_names = {
                    GeomAbs_BSplineCurve: 'BSpline',
                    GeomAbs_BezierCurve: 'Bezier',
                    GeomAbs_Hyperbola: 'Hyperbola',
                    GeomAbs_Parabola: 'Parabola',
                    GeomAbs_OtherCurve: 'Other',
                }
                return {
                    'curve_type': type_names.get(curve_type, f'Type_{curve_type}'),
                    'parameter_range': [round(first, 4), round(last, 4)],
                }

        except Exception as e:
            return {'curve_type': 'Unknown', 'error': str(e)}

    def _extract_edge_vertices(self, edge) -> list:
        """提取边的起点和终点坐标"""
        verts = []
        try:
            vert_exp = TopExp_Explorer(edge, TopAbs_VERTEX)
            while vert_exp.More():
                v = TopoDS.Vertex_s(vert_exp.Current())
                pnt = BRep_Tool.Pnt_s(v)
                verts.append(_point_to_tuple(pnt))
                vert_exp.Next()
        except Exception:
            pass
        return verts

    # ── 步骤 5: 提取顶点 ──

    def _extract_all_vertices(self) -> list:
        """提取所有顶点坐标"""
        vertices = []
        for vi in sorted(self._vertex_shapes.keys()):
            v = self._vertex_shapes[vi]
            try:
                pnt = BRep_Tool.Pnt_s(v)
                vertices.append({
                    'vertex_idx': vi,
                    'point': _point_to_tuple(pnt),
                })
            except Exception:
                vertices.append({
                    'vertex_idx': vi,
                    'point': None,
                })
        return vertices

    # ── 步骤 6: 板厚检测 ──

    def _detect_shell_thickness(self, faces: list) -> tuple:
        """检测钣金等厚壳体的统一板厚 t

        算法:
          1. 遍历所有平面对（不限于邻接面——壳体内外表面不共享边）
          2. 平行面对 (|dot| > 0.99) 计算面间垂直距离
             - dot > 0.99: 同向平行（BREP 实体中两个面的法向量都朝外，
               同一面墙的两个面可能同向）
             - dot < -0.99: 反向平行（经典情况，一面朝上、一面朝下）
          3. 取出现次数最多的距离作为统一板厚 t
          4. 返回 t 和确认的厚度面对列表
        """
        candidates = []

        planar_indices = [
            fi for fi, f in enumerate(faces)
            if f['surface_type'] == 'Plane'
        ]

        for i in range(len(planar_indices)):
            fi = planar_indices[i]
            fa = faces[fi]
            na = fa['surface_params'].get('normal')
            oa = fa['surface_params'].get('origin')
            if na is None or oa is None:
                continue

            for j in range(i + 1, len(planar_indices)):
                fj = planar_indices[j]
                fb = faces[fj]
                nb = fb['surface_params'].get('normal')
                ob = fb['surface_params'].get('origin')
                if nb is None or ob is None:
                    continue

                dot_n = _vec_dot(na, nb)
                # 平行（同向或反向）
                if abs(abs(dot_n) - 1.0) < 0.01:
                    # 计算两面之间的垂直距离
                    dist = abs(_vec_dot(_vec_sub(ob, oa), na))
                    if 0.3 < dist < 10.0:
                        candidates.append((fi, fj, round(dist, 4)))

        # 统计距离分布，取最常见的（钣金等厚）
        if not candidates:
            return None, []

        dist_counts = defaultdict(int)
        for fi, fj, d in candidates:
            dist_counts[d] += 1

        # 按出现次数降序排列
        sorted_dists = sorted(dist_counts.items(), key=lambda x: -x[1])
        thickness = sorted_dists[0][0]

        # 收集所有厚度为 t 的面对
        thickness_pairs = [
            (fi, fj) for fi, fj, d in candidates
            if abs(d - thickness) < 0.05
        ]

        return round(thickness, 2), thickness_pairs

    # ── 步骤 7: 面片分类 ──

    def _classify_faces(self, faces: list) -> dict:
        """分类面片类型

        平面 (Planar):
          - 折弯法兰、底板

        圆柱面 (Cylindrical) — 分为两类:
          a) 折弯内 R 面: 连接 >=2 个不共面平面的圆柱面 -> 折弯标记
          b) 孔侧壁/其他圆柱面: 其余圆柱面

        圆角面 / 折弯过渡面 (Fillet / Blend):
          - Torus: 连接两个不共面平面 -> 折弯 R 过渡，minor_radius = R
          - Cylinder: 连接两个不共面平面 -> 折弯 R 过渡，radius = R
          - BSpline/Bezier: 连接两个不共面平面 -> 折弯过渡（较少见）
        """
        planar = []
        cylindrical_other = []
        fillet = []       # 折弯过渡面
        other = []

        for f in faces:
            st = f['surface_type']
            fi = f['face_idx']

            if st == 'Plane':
                planar.append(fi)

            elif st == 'Cylinder':
                # 检查是否为折弯过渡面：连接 >=2 个不共面平面
                adj_planes = [
                    aj for aj in f['adjacent_faces']
                    if aj < len(faces) and faces[aj]['surface_type'] == 'Plane'
                ]
                is_bend = False
                if len(adj_planes) >= 2:
                    # 找一对不共面的平面来判定此圆柱面是否为折弯过渡面
                    for pi in range(len(adj_planes)):
                        for pj in range(pi + 1, len(adj_planes)):
                            na = faces[adj_planes[pi]]['surface_params'].get('normal')
                            nb = faces[adj_planes[pj]]['surface_params'].get('normal')
                            if na and nb:
                                dot_ab = abs(_vec_dot(na, nb))
                                if dot_ab < 0.9999:  # 夹角 > 0.8° -> 折弯
                                    is_bend = True
                                    # 保存全部相邻平面（可能 >2 个，如跨零件宽度的长圆角柱面）
                                    fillet.append({
                                        'face_idx': fi,
                                        'surface_type': 'Cylinder',
                                        'bend_radius': f['surface_params'].get('radius'),
                                        'adjacent_planes': adj_planes,
                                        'axis_origin': f['surface_params'].get('axis_origin'),
                                        'axis_direction': f['surface_params'].get('axis_direction'),
                                    })
                                    break
                            if is_bend:
                                break
                        if is_bend:
                            break
                if not is_bend:
                    cylindrical_other.append(fi)

            elif st in ('Torus', 'BSpline', 'Bezier'):
                adj_planes = [
                    aj for aj in f['adjacent_faces']
                    if aj < len(faces) and faces[aj]['surface_type'] == 'Plane'
                ]
                if len(adj_planes) >= 2:
                    na = faces[adj_planes[0]]['surface_params'].get('normal')
                    nb = faces[adj_planes[1]]['surface_params'].get('normal')
                    if na and nb:
                        dot_ab = abs(_vec_dot(na, nb))
                        if dot_ab < 0.9999:
                            r = None
                            if st == 'Torus':
                                r = f['surface_params'].get('minor_radius')
                            elif st in ('BSpline', 'Bezier'):
                                for aj in f['adjacent_faces']:
                                    if aj < len(faces):
                                        aj_st = faces[aj]['surface_type']
                                        if aj_st == 'Cylinder':
                                            r = faces[aj]['surface_params'].get('radius')
                                            break
                                        elif aj_st == 'Torus':
                                            r = faces[aj]['surface_params'].get('minor_radius')
                                            break
                            fillet.append({
                                'face_idx': fi,
                                'surface_type': st,
                                'bend_radius': r,
                                'adjacent_planes': adj_planes,
                                'axis_origin': f['surface_params'].get('axis_origin'),
                                'axis_direction': f['surface_params'].get('axis_direction'),
                            })
                        else:
                            other.append(fi)
                    else:
                        other.append(fi)
                else:
                    other.append(fi)
            else:
                other.append(fi)

        return {
            'planar_faces': planar,
            'cylindrical_faces': cylindrical_other,
            'fillet_faces': fillet,
            'other_faces': other,
        }

    # ── 步骤 8: 基准底板 ──

    def _find_base_face(self, faces: list, planar_indices: list) -> int:
        """选择面积最大的平面作为展开基准面"""
        best_fi = None
        best_area = -1
        for fi in planar_indices:
            area = faces[fi]['area']
            if area > best_area:
                best_area = area
                best_fi = fi
        return best_fi


# ── 测试入口 ──

if __name__ == '__main__':
    import json

    test_dir = r'd:\Users\liyis\Desktop\pythonproject\py06再实验\三维数模'

    for model_name in ['YA-1131-505.stp', '2ROM31-6061 ---.stp']:
        filepath = os.path.join(test_dir, model_name)
        if not os.path.exists(filepath):
            print(f"跳过: {model_name} (文件不存在)")
            continue

        print(f"\n{'='*70}")
        print(f"  BREP 拓扑抽取: {model_name}")
        print(f"{'='*70}")

        extractor = BrepTopologyExtractor(filepath)
        topo = extractor.extract()

        print(f"\n[基本统计]")
        print(f"  总面数:   {topo['total_faces']}")
        print(f"  总边数:   {topo['total_edges']}")
        print(f"  总顶点数: {topo['total_vertices']}")
        print(f"  板厚 t:   {topo['thickness']} mm")

        # 面类型分布
        st_count = defaultdict(int)
        for f in topo['faces']:
            st_count[f['surface_type']] += 1
        print(f"\n[面类型分布]")
        for st, cnt in sorted(st_count.items(), key=lambda x: -x[1]):
            print(f"  {st}: {cnt} 个")

        # 分类结果
        cls = topo['classification']
        print(f"\n[分类]  面片分类:")
        print(f"  平面 (Planar):     {len(cls['planar_faces'])} 个")
        print(f"  圆柱面 (Cylinder): {len(cls['cylindrical_faces'])} 个")
        print(f"  圆角面 (Fillet):   {len(cls['fillet_faces'])} 个")
        print(f"  其他:              {len(cls['other_faces'])} 个")

        # 圆角面/折弯过渡面详情
        if cls['fillet_faces']:
            print(f"\n[圆角] 折弯过渡面:")
            for fl in cls['fillet_faces']:
                r = fl.get('bend_radius') or fl.get('minor_radius')
                print(f"    face#{fl['face_idx']} ({fl.get('surface_type', '?')}): "
                      f"R={r}mm, "
                      f"轴线={fl.get('axis_direction')}, "
                      f"连接平面={fl['adjacent_planes']}")

        # 基准面
        base_fi = topo['base_face_idx']
        if base_fi is not None:
            base_f = topo['faces'][base_fi]
            print(f"\n[基准] 基准底板:")
            print(f"    face#{base_fi}, 面积={base_f['area']:.1f} mm^2, "
                  f"法向={base_f['surface_params'].get('normal')}")

        # 厚度面对
        if topo['thickness_pairs']:
            print(f"\n[厚度] 厚度面对 (t={topo['thickness']}mm):")
            for fi, fj in topo['thickness_pairs'][:8]:
                print(f"    face#{fi} <-> face#{fj}")
            if len(topo['thickness_pairs']) > 8:
                print(f"    ... 共 {len(topo['thickness_pairs'])} 对")

        # 面邻接关系抽样
        print(f"\n[邻接] 面邻接关系 (前 15 条):")
        count = 0
        for fi in sorted(cls['planar_faces'])[:10]:
            f = topo['faces'][fi]
            for fj in f['adjacent_faces']:
                if fj > fi and count < 15:
                    fj_type = topo['faces'][fj]['surface_type']
                    shared_edges = [
                        eid for eid in f['bounding_edges']
                        if eid in topo['faces'][fj]['bounding_edges']
                    ]
                    print(f"    face#{fi} (平面) <-> face#{fj} ({fj_type}) "
                          f"-- 共享边: {shared_edges}")
                    count += 1

    print(f"\n{'='*70}")
    print("  [OK] Step 1 BREP 拓扑抽取完成")
    print(f"{'='*70}")
