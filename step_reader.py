"""
STEP 文件读取与 3D 网格导出模块
使用 OpenCASCADE (cadquery-ocp) 内核，完整提取 STEP 文件的所有几何特征
"""

import os
import json
import numpy as np
from collections import defaultdict

import cadquery as cq
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool, BRep_Builder
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import (
    TopAbs_FACE, TopAbs_EDGE, TopAbs_WIRE, TopAbs_SOLID,
    TopAbs_SHELL, TopAbs_COMPOUND, TopAbs_COMPSOLID
)
from OCP.TopoDS import TopoDS, TopoDS_Compound, TopoDS_Iterator
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Pnt, gp_Vec
from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCP.GeomAbs import (
    GeomAbs_Plane, GeomAbs_Cylinder, GeomAbs_Cone,
    GeomAbs_Sphere, GeomAbs_Torus, GeomAbs_BezierSurface,
    GeomAbs_BSplineSurface, GeomAbs_SurfaceOfRevolution,
    GeomAbs_SurfaceOfExtrusion, GeomAbs_OffsetSurface,
    GeomAbs_Line, GeomAbs_Circle, GeomAbs_Ellipse,
    GeomAbs_Hyperbola, GeomAbs_Parabola, GeomAbs_BSplineCurve,
    GeomAbs_BezierCurve, GeomAbs_OtherCurve,
)
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.TDF import TDF_LabelSequence
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_ColorCurv
from OCP.TDataStd import TDataStd_Name
from OCP.TNaming import TNaming_NamedShape
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_ChildIterator


class StepReader:
    """STEP 文件读取器，提取完整几何特征并生成网格数据"""

    def __init__(self, linear_deflection: float = 0.1, angular_deflection: float = 0.1):
        """
        初始化读取器

        Args:
            linear_deflection: 线性偏差 (越小越精细，单位 mm)
            angular_deflection: 角度偏差 (弧度，越小越精细；0.1rad≈5.7°，曲面更光滑)
        """
        self.linear_deflection = linear_deflection
        self.angular_deflection = angular_deflection

    def read(self, filepath: str) -> dict:
        """
        读取 STEP 文件，返回完整模型数据

        Returns:
            dict: {
                'filename': str,
                'solids': [{'name': str, 'faces': [...], 'edges': [...], 'color': [...]}],
                'total_faces': int,
                'total_edges': int,
                'bbox': {...}
            }
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"文件不存在: {filepath}")

        filename = os.path.basename(filepath)

        # 使用 cadquery 导入几何
        result = cq.importers.importStep(filepath)

        shapes = result.vals() if hasattr(result, 'vals') else [result]

        all_solids = []
        all_face_meshes = []
        all_edge_meshes = []

        for shape in shapes:
            topo_shape = shape.wrapped
            solids_data = self._extract_shape_data(topo_shape)
            all_solids.extend(solids_data)

        # 计算包围盒
        bbox = self._compute_bbox(all_solids)

        return {
            'filename': filename,
            'filepath': filepath,
            'solids': all_solids,
            'total_solids': len(all_solids),
            'total_faces': sum(len(s['faces']) for s in all_solids),
            'total_edges': sum(len(s['edges']) for s in all_solids),
            'total_triangles': sum(s['triangle_count'] for s in all_solids),
            'bbox': bbox,
            'linear_deflection': self.linear_deflection,
        }

    def _read_colors_with_xcaf(self, filepath: str) -> dict:
        """使用 XCAF 读取器提取面和边的颜色信息"""
        color_map = {'faces': {}, 'edges': {}}

        try:
            app = XCAFApp_Application.GetApplication_s()
            doc = TDocStd_Document(TCollection_ExtendedString("step_doc"))
            app.InitDocument(doc)

            shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
            color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

            reader = STEPCAFControl_Reader()
            reader.SetName(filename := os.path.basename(filepath))
            reader.SetColorMode(True)
            reader.SetLayerMode(True)
            reader.SetNameMode(True)
            reader.SetPropsMode(True)

            status = reader.ReadFile(filepath)
            if status != 1:  # IFSelect_RetDone
                return color_map

            reader.Transfer(doc)

            # 遍历所有形状标签获取颜色
            labels = TDF_LabelSequence()
            shape_tool.GetShapes(labels)

            for i in range(labels.Length()):
                label = labels.Value(i + 1)
                # 检查面颜色
                face_color = Quantity_Color()
                if color_tool.GetColor(label, XCAFDoc_ColorSurf, face_color):
                    r, g, b = face_color.GetRGB().Get()
                    # 获取这个标签对应的形状
                    shape = shape_tool.GetShape(label)
                    if not shape.IsNull():
                        exp = TopExp_Explorer(shape, TopAbs_FACE)
                        while exp.More():
                            face = exp.Current()
                            hash_code = face.HashCode(1000000)
                            color_map['faces'][hash_code] = [r, g, b]
                            exp.Next()
        except Exception as e:
            # XCAF 颜色读取失败不影响几何数据提取
            pass

        return color_map

    def _extract_shape_data(self, topo_shape) -> list:
        """提取单个 TopoDS_Shape 的数据"""
        solids = []

        # 遍历 Solids
        exp = TopExp_Explorer(topo_shape, TopAbs_SOLID)
        solid_idx = 0
        while exp.More():
            solid = exp.Current()
            solid_data = self._process_solid(solid, solid_idx)
            if solid_data['faces']:
                solids.append(solid_data)
            solid_idx += 1
            exp.Next()

        # 如果没有 Solid，尝试作为 Shell 处理
        if not solids:
            exp = TopExp_Explorer(topo_shape, TopAbs_SHELL)
            while exp.More():
                shell = exp.Current()
                solid_data = self._process_shell(shell, 0)
                if solid_data['faces']:
                    solids.append(solid_data)
                exp.Next()

        return solids

    def _process_solid(self, solid, idx: int) -> dict:
        """处理单个 Solid，同时提取网格数据和几何特征参数"""
        all_vertices = []
        all_triangles = []
        vertex_offset = 0
        face_vertex_offsets = []
        face_params_list = []

        # 先对 Solid 进行网格化
        mesh = BRepMesh_IncrementalMesh(solid, self.linear_deflection, False, self.angular_deflection)

        # 遍历面
        face_exp = TopExp_Explorer(solid, TopAbs_FACE)
        face_idx = 0
        while face_exp.More():
            face = face_exp.Current()

            # 1) 三角剖分（网格数据，用于 3D 渲染）
            face_data = self._tessellate_face(face)

            if face_data and face_data['vertices']:
                nv = len(face_data['vertices']) // 3
                all_vertices.extend(face_data['vertices'])
                offset_tris = [t + vertex_offset for t in face_data['triangles']]
                all_triangles.extend(offset_tris)
                face_vertex_offsets.append({
                    'face_idx': face_idx,
                    'vertex_offset': vertex_offset,
                    'vertex_count': nv,
                    'triangle_count': len(face_data['triangles']) // 3,
                    'color': face_data.get('color', [0.7, 0.7, 0.7]),
                    'surface_type': face_data.get('surface_type', 'Unknown'),
                })
                vertex_offset += nv

            # 2) 提取面的几何参数（工艺特征数据，不含网格顶点）
            #    注意：BRepAdaptor_Surface 需要 TopoDS_Face，需转换
            face_params = self._extract_face_params(TopoDS.Face_s(face))
            face_params['face_idx'] = face_idx
            face_params_list.append(face_params)

            face_idx += 1
            face_exp.Next()

        # 提取边（线框采样 + 曲线参数）
        edge_exp = TopExp_Explorer(solid, TopAbs_EDGE)
        edge_idx = 0
        edge_vertices = []
        edge_params_list = []
        while edge_exp.More():
            edge = edge_exp.Current()
            edge_verts = self._tessellate_edge(edge)
            edge_params = self._extract_edge_params(TopoDS.Edge_s(edge))
            edge_params['edge_idx'] = edge_idx
            edge_params_list.append(edge_params)

            if edge_verts:
                edge_vertices.append({
                    'edge_idx': edge_idx,
                    'vertices': edge_verts,
                    'curve_type': edge_params.get('curve_type', 'Unknown'),
                })
            edge_idx += 1
            edge_exp.Next()

        # 获取 Solid 名称
        name = f"Solid_{idx + 1}"

        # 3) 计算物理属性（体积、表面积、质心）
        physical_props = self._compute_physical_properties(solid)

        # 4) 检测制造特征（孔、圆角、倒角）
        features = self._detect_features(face_params_list, edge_params_list)

        return {
            'name': name,
            'type': 'Solid',
            'faces': face_vertex_offsets,
            'edges': edge_vertices,
            'face_params': face_params_list,
            'edge_params': edge_params_list,
            'vertices': all_vertices,
            'triangles': all_triangles,
            'triangle_count': len(all_triangles) // 3,
            'vertex_count': len(all_vertices) // 3,
            'face_count': face_idx,
            'edge_count': edge_idx,
            'physical_properties': physical_props,
            'features': features,
        }

    def _process_shell(self, shell, idx: int) -> dict:
        """处理 Shell（与 Solid 类似）"""
        # 构建一个临时的 Compound 来网格化
        builder = BRep_Builder()
        compound = TopoDS_Compound()
        builder.MakeCompound(compound)
        builder.Add(compound, shell)

        return self._process_solid(compound, idx)

    def _tessellate_face(self, face_input) -> dict:
        """对面进行三角剖分"""
        try:
            # 正确地将 TopoDS_Shape 转换为 TopoDS_Face
            face = TopoDS.Face_s(face_input)
            top_loc = TopLoc_Location()
            triangulation = BRep_Tool.Triangulation_s(face, top_loc)

            if triangulation is None or triangulation.NbNodes() == 0:
                return None

            # 获取节点和三角形
            n_nodes = triangulation.NbNodes()
            n_triangles = triangulation.NbTriangles()

            # 变换矩阵
            transform = top_loc.Transformation()

            # 提取顶点
            vertices = []
            for i in range(1, n_nodes + 1):
                pnt = triangulation.Node(i)
                pnt.Transform(transform)
                vertices.extend([pnt.X(), pnt.Y(), pnt.Z()])

            # 提取三角形
            tri_indices = []
            for i in range(1, n_triangles + 1):
                tri = triangulation.Triangle(i)
                # OpenCASCADE 使用 1-based 索引
                tri_indices.extend([tri.Value(1) - 1, tri.Value(3) - 1, tri.Value(2) - 1])  # 翻转法线方向

            # 获取面颜色（简化处理，使用基于曲面类型的默认颜色）
            color = None
            surface_type = self._get_surface_type(face)
            color = self._get_default_color(surface_type)

            return {
                'vertices': vertices,
                'triangles': tri_indices,
                'color': color,
                'surface_type': surface_type,
            }

        except Exception as e:
            return None

    # 边线采样固定间距（mm）。独立于面网格精度，保证即便面网格为了
    # 解析速度使用较粗 deflection，轮廓边线（直线/圆/样条）依然光顺精确。
    EDGE_SAMPLE_STEP = 0.1
    EDGE_MAX_POINTS = 800

    def _tessellate_edge(self, edge_input) -> list:
        """对边进行离散化，返回线段顶点。

        采样间距固定为 EDGE_SAMPLE_STEP（0.1mm），不再跟随
        linear_deflection，确保轮廓不会被解析精度拖累。
        单条边最多 EDGE_MAX_POINTS 段，防止超大模型输出爆炸。
        """
        try:
            from OCP.BRepAdaptor import BRepAdaptor_Curve
            from OCP.GCPnts import GCPnts_UniformAbscissa

            edge = TopoDS.Edge_s(edge_input)
            adaptor = BRepAdaptor_Curve(edge)
            first = adaptor.FirstParameter()
            last = adaptor.LastParameter()

            # 直线边：只需首尾两个端点，避免整条线被 0.1mm
            # 细采样产生成千上万个冗余点（JSON 膨胀、前端变慢）
            if adaptor.GetType() == GeomAbs_Line:
                p1 = gp_Pnt(); v1 = gp_Vec()
                p2 = gp_Pnt(); v2 = gp_Vec()
                adaptor.D0(first, p1)
                adaptor.D0(last, p2)
                return [p1.X(), p1.Y(), p1.Z(),
                        p2.X(), p2.Y(), p2.Z()]

            # 曲线（圆/样条等）：按固定细间距采样
            abscissa = GCPnts_UniformAbscissa(adaptor, self.EDGE_SAMPLE_STEP, first, last)
            if not abscissa.IsDone():
                return []

            n_sampled = abscissa.NbPoints()
            if n_sampled > self.EDGE_MAX_POINTS:
                # 长边/长圆弧：按每边上限重新均匀采样
                step2 = self.EDGE_SAMPLE_STEP * n_sampled / float(self.EDGE_MAX_POINTS)
                abscissa = GCPnts_UniformAbscissa(adaptor, step2, first, last)
                if not abscissa.IsDone():
                    return []
                n_sampled = abscissa.NbPoints()

            vertices = []
            if n_sampled > self.EDGE_MAX_POINTS:
                # 重采样后仍偏多：均匀抽点，严格不超过上限
                ratio = (n_sampled - 1) / float(self.EDGE_MAX_POINTS - 1)
                for k in range(self.EDGE_MAX_POINTS):
                    i = 1 + int(round(k * ratio))
                    param = abscissa.Parameter(i)
                    pnt = gp_Pnt()
                    vec = gp_Vec()
                    adaptor.D1(param, pnt, vec)
                    vertices.extend([pnt.X(), pnt.Y(), pnt.Z()])
            else:
                for i in range(1, n_sampled + 1):
                    param = abscissa.Parameter(i)
                    pnt = gp_Pnt()
                    vec = gp_Vec()
                    adaptor.D1(param, pnt, vec)
                    vertices.extend([pnt.X(), pnt.Y(), pnt.Z()])

            return vertices

        except Exception:
            return []

    def _get_surface_type(self, face) -> str:
        """获取面的曲面类型"""
        try:
            adaptor = BRepAdaptor_Surface(face)
            surf_type = adaptor.GetType()

            type_map = {
                GeomAbs_Plane: 'Plane',
                GeomAbs_Cylinder: 'Cylinder',
                GeomAbs_Cone: 'Cone',
                GeomAbs_Sphere: 'Sphere',
                GeomAbs_Torus: 'Torus',
                GeomAbs_BezierSurface: 'Bezier',
                GeomAbs_BSplineSurface: 'BSpline',
                GeomAbs_SurfaceOfRevolution: 'Revolution',
                GeomAbs_SurfaceOfExtrusion: 'Extrusion',
                GeomAbs_OffsetSurface: 'Offset',
            }
            return type_map.get(surf_type, f'Type_{surf_type}')
        except Exception:
            return 'Unknown'

    def _get_default_color(self, surface_type: str) -> list:
        """根据曲面类型返回默认颜色"""
        color_map = {
            'Plane': [0.65, 0.65, 0.70],
            'Cylinder': [0.55, 0.65, 0.75],
            'Cone': [0.60, 0.65, 0.70],
            'Sphere': [0.50, 0.60, 0.75],
            'Torus': [0.55, 0.60, 0.70],
            'Bezier': [0.70, 0.60, 0.65],
            'BSpline': [0.68, 0.62, 0.68],
            'Revolution': [0.60, 0.65, 0.70],
            'Extrusion': [0.62, 0.65, 0.72],
            'Offset': [0.65, 0.63, 0.70],
        }
        return color_map.get(surface_type, [0.70, 0.70, 0.70])

    # ── 几何参数提取（工艺特征数据）─────────────────────────────────────────

    def _extract_face_params(self, face) -> dict:
        """提取面的几何参数：圆柱→半径/轴线，锥面→角度，环面→圆角半径等"""
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
                    'normal': [round(axis.Direction().X(), 6),
                              round(axis.Direction().Y(), 6),
                              round(axis.Direction().Z(), 6)],
                    'origin': [round(loc.X(), 4), round(loc.Y(), 4), round(loc.Z(), 4)],
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
                    'axis_origin': [round(loc.X(), 4), round(loc.Y(), 4), round(loc.Z(), 4)],
                    'axis_direction': [round(axis.Direction().X(), 6),
                                      round(axis.Direction().Y(), 6),
                                      round(axis.Direction().Z(), 6)],
                }

            elif surf_type == GeomAbs_Cone:
                cone = adaptor.Cone()
                ax3 = cone.Position()
                axis = ax3.Axis()
                loc = axis.Location()
                apex = cone.Apex()
                semi_angle = cone.SemiAngle()
                ref_radius = cone.RefRadius()
                return {
                    'surface_type': 'Cone',
                    'semi_angle_rad': round(semi_angle, 6),
                    'semi_angle_deg': round(semi_angle * 180.0 / 3.141592653589793, 2),
                    'ref_radius': round(ref_radius, 4),
                    'apex': [round(apex.X(), 4), round(apex.Y(), 4), round(apex.Z(), 4)],
                    'axis_origin': [round(loc.X(), 4), round(loc.Y(), 4), round(loc.Z(), 4)],
                    'axis_direction': [round(axis.Direction().X(), 6),
                                      round(axis.Direction().Y(), 6),
                                      round(axis.Direction().Z(), 6)],
                }

            elif surf_type == GeomAbs_Sphere:
                sphere = adaptor.Sphere()
                center = sphere.Location()
                radius = sphere.Radius()
                return {
                    'surface_type': 'Sphere',
                    'radius': round(radius, 4),
                    'diameter': round(radius * 2, 4),
                    'center': [round(center.X(), 4), round(center.Y(), 4), round(center.Z(), 4)],
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
                    'minor_radius': round(minor_r, 4),  # 圆角半径
                    'axis_origin': [round(loc.X(), 4), round(loc.Y(), 4), round(loc.Z(), 4)],
                    'axis_direction': [round(axis.Direction().X(), 6),
                                      round(axis.Direction().Y(), 6),
                                      round(axis.Direction().Z(), 6)],
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

        except Exception:
            return {'surface_type': 'Unknown'}

    def _extract_edge_params(self, edge) -> dict:
        """提取边的曲线类型和几何参数（直线长度 / 圆半径 / 椭圆等）"""
        try:
            adaptor = BRepAdaptor_Curve(edge)
            curve_type = adaptor.GetType()
            first = adaptor.FirstParameter()
            last = adaptor.LastParameter()

            if curve_type == GeomAbs_Line:
                line = adaptor.Line()
                direction = line.Direction()
                # 计算实际端点（参数范围）
                p1 = gp_Pnt(); v1 = gp_Vec()
                p2 = gp_Pnt(); v2 = gp_Vec()
                adaptor.D0(first, p1)
                adaptor.D0(last, p2)
                return {
                    'curve_type': 'Line',
                    'direction': [round(direction.X(), 6),
                                 round(direction.Y(), 6),
                                 round(direction.Z(), 6)],
                    'start_point': [round(p1.X(), 4), round(p1.Y(), 4), round(p1.Z(), 4)],
                    'end_point': [round(p2.X(), 4), round(p2.Y(), 4), round(p2.Z(), 4)],
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
                    'center': [round(center.X(), 4), round(center.Y(), 4), round(center.Z(), 4)],
                    'axis_direction': [round(axis.Direction().X(), 6),
                                      round(axis.Direction().Y(), 6),
                                      round(axis.Direction().Z(), 6)],
                    'circumference': round(2 * 3.141592653589793 * radius, 4),
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
                    'center': [round(center.X(), 4), round(center.Y(), 4), round(center.Z(), 4)],
                }

            else:
                type_names = {
                    GeomAbs_Hyperbola: 'Hyperbola',
                    GeomAbs_Parabola: 'Parabola',
                    GeomAbs_BSplineCurve: 'BSpline',
                    GeomAbs_BezierCurve: 'Bezier',
                    GeomAbs_OtherCurve: 'Other',
                }
                return {
                    'curve_type': type_names.get(curve_type, f'Type_{curve_type}'),
                    'parameter_range': [round(first, 4), round(last, 4)],
                }

        except Exception:
            return {'curve_type': 'Unknown'}

    def _compute_physical_properties(self, shape) -> dict:
        """计算形状的物理属性：体积、表面积、质心（使用 GProp）"""
        try:
            # 体积属性（OCP 绑定方法名带 _s 后缀）
            vol_props = GProp_GProps()
            BRepGProp.VolumeProperties_s(shape, vol_props)
            volume = vol_props.Mass()
            cog = vol_props.CentreOfMass()

            # 表面积属性
            surf_props = GProp_GProps()
            BRepGProp.SurfaceProperties_s(shape, surf_props)
            surface_area = surf_props.Mass()

            return {
                'volume': round(volume, 4),
                'surface_area': round(surface_area, 4),
                'center_of_mass': [round(cog.X(), 4), round(cog.Y(), 4), round(cog.Z(), 4)],
            }
        except Exception:
            return {'volume': None, 'surface_area': None, 'center_of_mass': None}

    def _detect_features(self, face_params_list: list, edge_params_list: list) -> dict:
        """从面和边参数中检测制造特征：孔、圆角、倒角"""
        features = {
            'holes': [],
            'fillets': [],
            'chamfers': [],
            'cylindrical_faces': [],
            'conical_faces': [],
            'torus_faces': [],
        }

        for fp in face_params_list:
            st = fp.get('surface_type', '')

            if st == 'Cylinder':
                radius = fp.get('radius')
                info = {
                    'face_idx': fp.get('face_idx'),
                    'radius': radius,
                    'diameter': round(radius * 2, 4) if radius else None,
                    'axis_direction': fp.get('axis_direction'),
                    'axis_origin': fp.get('axis_origin'),
                }
                features['cylindrical_faces'].append(info)
                # 所有圆柱面列为潜在孔
                # （后续可通过拓扑分析区分通孔/盲孔/外圆柱凸台）
                features['holes'].append(info)

            elif st == 'Torus':
                major_r = fp.get('major_radius')
                minor_r = fp.get('minor_radius')
                t_info = {
                    'face_idx': fp.get('face_idx'),
                    'major_radius': major_r,
                    'minor_radius': minor_r,
                    'axis_direction': fp.get('axis_direction'),
                }
                features['torus_faces'].append(t_info)
                if minor_r:
                    features['fillets'].append({
                        'face_idx': fp.get('face_idx'),
                        'radius': minor_r,
                    })

            elif st == 'Cone':
                semi_angle = fp.get('semi_angle_deg')
                c_info = {
                    'face_idx': fp.get('face_idx'),
                    'semi_angle_deg': semi_angle,
                    'ref_radius': fp.get('ref_radius'),
                    'axis_direction': fp.get('axis_direction'),
                }
                features['conical_faces'].append(c_info)
                # 半角 40°~50° 通常为倒角特征
                if semi_angle and 40 <= semi_angle <= 50:
                    features['chamfers'].append(c_info)

        return features

    def _compute_bbox(self, solids_data: list) -> dict:
        """计算整体包围盒"""
        if not solids_data:
            return {'min': [0, 0, 0], 'max': [0, 0, 0], 'center': [0, 0, 0], 'size': [0, 0, 0]}

        all_verts = []
        for solid in solids_data:
            verts = solid['vertices']
            for i in range(0, len(verts), 3):
                all_verts.append([verts[i], verts[i+1], verts[i+2]])

        if not all_verts:
            return {'min': [0, 0, 0], 'max': [0, 0, 0], 'center': [0, 0, 0], 'size': [0, 0, 0]}

        arr = np.array(all_verts)
        bmin = arr.min(axis=0).tolist()
        bmax = arr.max(axis=0).tolist()
        center = ((arr.min(axis=0) + arr.max(axis=0)) / 2).tolist()
        size = (arr.max(axis=0) - arr.min(axis=0)).tolist()

        return {
            'min': [round(v, 4) for v in bmin],
            'max': [round(v, 4) for v in bmax],
            'center': [round(v, 4) for v in center],
            'size': [round(v, 4) for v in size],
        }

    def export_mesh_json(self, filepath: str, output_path: str = None) -> str:
        """
        读取 STEP 文件并导出为 JSON 网格数据

        Args:
            filepath: STEP 文件路径
            output_path: 输出 JSON 路径，默认与 STEP 文件同目录

        Returns:
            输出文件路径
        """
        data = self.read(filepath)

        # 构建紧凑的导出数据
        export_data = {
            'filename': data['filename'],
            'bbox': data['bbox'],
            'stats': {
                'solids': data['total_solids'],
                'faces': data['total_faces'],
                'edges': data['total_edges'],
                'triangles': data['total_triangles'],
            },
            'solids': [],
        }

        for solid in data['solids']:
            s = {
                'name': solid['name'],
                'vertices': solid['vertices'],
                'triangles': solid['triangles'],
                'faces': solid['faces'],
                'edges': [{'vertices': e['vertices']} for e in solid['edges']],
            }
            export_data['solids'].append(s)

        if output_path is None:
            base = os.path.splitext(filepath)[0]
            output_path = base + '_mesh.json'

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(export_data, f, ensure_ascii=False)

        print(f"[OK] 导出完成: {output_path}")
        print(f"  - Solids: {data['total_solids']}")
        print(f"  - Faces: {data['total_faces']}")
        print(f"  - Edges: {data['total_edges']}")
        print(f"  - Triangles: {data['total_triangles']:,}")
        print(f"  - 文件大小: {os.path.getsize(output_path) / 1024 / 1024:.2f} MB")

        return output_path

    def export_features_json(self, filepath: str, output_path: str = None) -> str:
        """
        读取 STEP 文件并导出工艺特征数据为独立 JSON（不含网格顶点/三角形）

        生成的 JSON 文件轻量、可读，适合：
        - 发送给其他人查看模型制造特征
        - 作为下游工艺规划系统的输入
        - 人工审查孔/圆角/倒角/曲面类型

        Args:
            filepath: STEP 文件路径
            output_path: 输出 JSON 路径，默认与 STEP 文件同目录，后缀 _features.json

        Returns:
            输出文件路径
        """
        data = self.read(filepath)

        # 构建轻量特征数据（不含大量网格顶点坐标）
        feature_data = {
            'file_info': {
                'filename': data['filename'],
                'filepath': data['filepath'],
                'units': 'mm',
                'linear_deflection': data['linear_deflection'],
            },
            'bbox': data['bbox'],
            'statistics': {
                'solids': data['total_solids'],
                'faces': data['total_faces'],
                'edges': data['total_edges'],
                'triangles': data['total_triangles'],
            },
            'solids': [],
            'summary': {},
        }

        # 汇总统计
        all_surface_types = defaultdict(int)
        all_curve_types = defaultdict(int)
        total_holes = 0
        total_fillets = 0
        total_chamfers = 0
        total_volume = 0
        total_surface_area = 0
        all_diameters = []
        all_fillet_radii = []

        for solid in data['solids']:
            # ── 面特征 ──
            faces_out = []
            for fp in solid.get('face_params', []):
                st = fp.get('surface_type', 'Unknown')
                all_surface_types[st] += 1

                face_entry = {
                    'id': fp.get('face_idx'),
                    'surface_type': st,
                }
                # 按类型填充有意义的参数
                if st == 'Cylinder':
                    face_entry['diameter'] = fp.get('diameter')
                    face_entry['axis_direction'] = fp.get('axis_direction')
                    face_entry['axis_origin'] = fp.get('axis_origin')
                    if fp.get('diameter'):
                        all_diameters.append(fp['diameter'])
                elif st == 'Cone':
                    face_entry['semi_angle_deg'] = fp.get('semi_angle_deg')
                    face_entry['axis_direction'] = fp.get('axis_direction')
                elif st == 'Torus':
                    face_entry['minor_radius'] = fp.get('minor_radius')
                    face_entry['major_radius'] = fp.get('major_radius')
                    if fp.get('minor_radius'):
                        all_fillet_radii.append(fp['minor_radius'])
                elif st == 'Plane':
                    face_entry['normal'] = fp.get('normal')
                elif st == 'Sphere':
                    face_entry['diameter'] = fp.get('diameter')
                    face_entry['center'] = fp.get('center')

                faces_out.append(face_entry)

            # ── 边特征 ──
            edges_out = []
            for ep in solid.get('edge_params', []):
                ct = ep.get('curve_type', 'Unknown')
                all_curve_types[ct] += 1

                edge_entry = {
                    'id': ep.get('edge_idx'),
                    'curve_type': ct,
                }
                if ct == 'Line':
                    edge_entry['length'] = ep.get('length')
                elif ct == 'Circle':
                    edge_entry['radius'] = ep.get('radius')
                    edge_entry['diameter'] = ep.get('diameter')
                    edge_entry['center'] = ep.get('center')
                    edge_entry['axis_direction'] = ep.get('axis_direction')
                elif ct == 'Ellipse':
                    edge_entry['major_radius'] = ep.get('major_radius')
                    edge_entry['minor_radius'] = ep.get('minor_radius')

                edges_out.append(edge_entry)

            # ── 物理属性 ──
            phys = solid.get('physical_properties', {})
            if phys.get('volume'):
                total_volume += phys['volume']
            if phys.get('surface_area'):
                total_surface_area += phys['surface_area']

            # ── 特征 ──
            feats = solid.get('features', {})
            total_holes += len(feats.get('holes', []))
            total_fillets += len(feats.get('fillets', []))
            total_chamfers += len(feats.get('chamfers', []))

            solid_out = {
                'name': solid['name'],
                'type': solid['type'],
                'face_count': solid['face_count'],
                'edge_count': solid['edge_count'],
                'physical_properties': phys,
                'faces': faces_out,
                'edges': edges_out,
                'features': feats,
            }
            feature_data['solids'].append(solid_out)

        # ── 汇总摘要 ──
        feature_data['summary'] = {
            'total_volume': round(total_volume, 4),
            'total_surface_area': round(total_surface_area, 4),
            'surface_types': dict(all_surface_types),
            'curve_types': dict(all_curve_types),
            'holes': total_holes,
            'fillets': total_fillets,
            'chamfers': total_chamfers,
            'unique_diameters': sorted(set(round(d, 2) for d in all_diameters)) if all_diameters else [],
            'unique_fillet_radii': sorted(set(round(r, 2) for r in all_fillet_radii)) if all_fillet_radii else [],
        }

        # 生成人类可读的摘要文字
        summary_parts = []
        if total_holes:
            dia_str = ', '.join(f'Φ{d}' for d in feature_data['summary']['unique_diameters'][:5])
            if len(feature_data['summary']['unique_diameters']) > 5:
                dia_str += '…'
            summary_parts.append(f'{total_holes} 个圆柱面 ({dia_str})')
        if total_fillets:
            r_str = ', '.join(f'R{r}' for r in feature_data['summary']['unique_fillet_radii'][:5])
            if len(feature_data['summary']['unique_fillet_radii']) > 5:
                r_str += '…'
            summary_parts.append(f'{total_fillets} 个圆角 ({r_str})')
        if total_chamfers:
            summary_parts.append(f'{total_chamfers} 个倒角')
        feature_data['summary']['text'] = '; '.join(summary_parts) if summary_parts else '简单几何体（无孔/圆角/倒角）'

        # 写入文件
        if output_path is None:
            base = os.path.splitext(filepath)[0]
            output_path = base + '_features.json'

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(feature_data, f, ensure_ascii=False, indent=2)

        print(f"[OK] 特征数据导出完成: {output_path}")
        print(f"  - 文件大小: {os.path.getsize(output_path) / 1024:.1f} KB")
        print(f"  - 面类型: {dict(all_surface_types)}")
        print(f"  - 边类型: {dict(all_curve_types)}")
        print(f"  - 特征: {feature_data['summary']['text']}")

        return output_path


class StepFileScanner:
    """扫描目录下的 STEP 文件"""

    @staticmethod
    def scan(directory: str) -> list:
        """扫描并返回 STEP 文件列表"""
        files = []
        for f in os.listdir(directory):
            if f.lower().endswith(('.stp', '.step')):
                full_path = os.path.join(directory, f)
                size = os.path.getsize(full_path)
                files.append({
                    'name': f,
                    'path': full_path,
                    'size': size,
                    'size_str': f'{size/1024:.1f} KB' if size < 1024*1024 else f'{size/1024/1024:.1f} MB'
                })
        return sorted(files, key=lambda x: x['name'])



if __name__ == '__main__':
    import sys

    reader = StepReader(linear_deflection=0.5)

    # 测试读取
    test_dir = r'd:\Users\liyis\Desktop\pythonproject\py06再实验\三维数模'
    files = StepFileScanner.scan(test_dir)

    print(f"找到 {len(files)} 个 STEP 文件:\n")
    for f in files:
        print(f"  {f['name']} ({f['size_str']})")

    if files:
        print(f"\n{'='*60}")
        print(f"读取测试: {files[0]['name']}")
        print(f"{'='*60}")
        data = reader.read(files[0]['path'])
        print(f"\n模型信息:")
        print(f"  文件名: {data['filename']}")
        print(f"  Solids: {data['total_solids']}")
        print(f"  Faces: {data['total_faces']}")
        print(f"  Edges: {data['total_edges']}")
        print(f"  Triangles: {data['total_triangles']:,}")
        print(f"  包围盒: {data['bbox']['size']}")
        print(f"  中心: {data['bbox']['center']}")

        # 物理属性
        for s in data['solids']:
            phys = s.get('physical_properties', {})
            if phys.get('volume'):
                print(f"\n物理属性 ({s['name']}):")
                print(f"  体积: {phys['volume']:.2f} mm^3")
                print(f"  表面积: {phys['surface_area']:.2f} mm^2")
                print(f"  质心: {[round(c, 4) for c in phys['center_of_mass']]}")

            # 面类型统计
            st_count = defaultdict(int)
            for fp in s.get('face_params', []):
                st_count[fp.get('surface_type', 'Unknown')] += 1
            print(f"\n面类型分布 ({s['name']}):")
            for st, cnt in sorted(st_count.items(), key=lambda x: -x[1]):
                print(f"  {st}: {cnt} 个")

            # 检测到的特征
            feats = s.get('features', {})
            print(f"\n特征检测 ({s['name']}):")
            print(f"  圆柱面（潜在孔）: {len(feats.get('cylindrical_faces', []))} 个")
            if feats.get('cylindrical_faces'):
                for h in feats['cylindrical_faces'][:5]:
                    print(f"    face#{h['face_idx']}: Φ{h['diameter']:.1f}mm 轴线={h['axis_direction']}")
            print(f"  环面（圆角）: {len(feats.get('torus_faces', []))} 个")
            if feats.get('fillets'):
                for fl in feats['fillets'][:5]:
                    print(f"    face#{fl['face_idx']}: R{fl['radius']:.1f}mm")
            print(f"  锥面（含倒角）: {len(feats.get('conical_faces', []))} 个")
            print(f"  倒角: {len(feats.get('chamfers', []))} 个")

        print(f"\n导出网格 JSON...")
        reader.export_mesh_json(files[0]['path'])

        print(f"\n导出特征 JSON...")
        reader.export_features_json(files[0]['path'])
