"""
几何工具函数 (geometry_utils.py)

作者: 模具自动生成系统开发组
用途: 提供各模块共享的几何计算工具 —— 包围盒、体积、表面积、形状有效性检查。
      基于 OpenCASCADE 内核实现（本工程使用 cadquery-ocp 提供的 OCP 绑定，
      即 pythonocc 风格 API，底层为 OCCT 7.8）。
"""

from typing import Tuple

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

# (x_min, y_min, z_min, x_max, y_max, z_max)
BBox = Tuple[float, float, float, float, float, float]


def shape_bbox(shape, use_triangulation: bool = False) -> BBox:
    """计算形状的包围盒。

    返回 (x_min, y_min, z_min, x_max, y_max, z_max)。

    参数:
        use_triangulation: 是否优先使用已有三角网格（未网格化时耗时，默认 False）。
    """
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, use_triangulation)
    return box.Get()


def shape_volume(shape) -> float:
    """计算闭合实体的体积（mm³）。"""
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()


def shape_surface_area(shape) -> float:
    """计算形状的表面积（mm²）。"""
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, props)
    return props.Mass()


def is_valid_shape(shape, check_geometry: bool = True) -> bool:
    """检查形状有效性。

    1. 必做 IsNull() 检查（引用为空则无效）；
    2. check_geometry=True 时再用 BRepCheck_Analyzer 做几何合法性检查
       （分析器偶发异常时按“未通过”处理，但不向上抛出）。
    """
    if shape is None or shape.IsNull():
        return False
    if not check_geometry:
        return True
    try:
        return bool(BRepCheck_Analyzer(shape).IsValid())
    except Exception:  # noqa: BLE001 —— 分析器自身异常不扩散
        return False


def triangulate_shape(shape, deflection: float = 0.5, angular: float = 0.5):
    """三角化形状，返回 (N,3,3) 三角形顶点坐标数组（供可视化 / 采样检验用）。

    deflection 为线性偏差（mm），越小网格越密、大尺寸零件请按需放大。
    """
    import numpy as np
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    BRepMesh_IncrementalMesh(shape, deflection, False, angular, True)
    tris = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current())
        exp.Next()
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            continue
        trsf = loc.Transformation()
        nodes = np.array([[tri.Node(i).Transformed(trsf).X(),
                           tri.Node(i).Transformed(trsf).Y(),
                           tri.Node(i).Transformed(trsf).Z()]
                          for i in range(1, tri.NbNodes() + 1)])
        rev = (face.Orientation() == TopAbs_REVERSED)
        for i in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(i).Get()
            if rev:
                a, c = c, a
            tris.append([nodes[a - 1], nodes[b - 1], nodes[c - 1]])
    return np.asarray(tris) if tris else np.zeros((0, 3, 3))


if __name__ == "__main__":
    # 自测入口
    from io_utils import ensure_utf8_stdout

    ensure_utf8_stdout()
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt

    box = BRepPrimAPI_MakeBox(gp_Pnt(-10.0, -10.0, 0.0), gp_Pnt(10.0, 10.0, 20.0)).Shape()
    print("包围盒:", shape_bbox(box))
    print("体积:", shape_volume(box))
    print("表面积:", shape_surface_area(box))
    print("有效性:", is_valid_shape(box))
