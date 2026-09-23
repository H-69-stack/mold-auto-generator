"""
镜像 / 旋转模型生成器
基于 OpenCASCADE (CadQuery / OCP bindings)

* 镜像：关于 xOy / xOz / yOz 面（可过质心或过坐标原点）
* 旋转（2026-09 新增）：绕 x / y / z 轴，顺时针或逆时针，角度自填
  两者可单独用，也可叠加（先镜像后旋转），输出一个 STEP 文件供下载。
"""

import os
import cadquery as cq
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.gp import gp_Pnt, gp_Dir, gp_Ax1, gp_Ax2, gp_Trsf
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_SHELL

from naming_utils import part_number_from_filename

AXES = {
    'x': {'label': 'X 轴', 'dir': (1.0, 0.0, 0.0)},
    'y': {'label': 'Y 轴', 'dir': (0.0, 1.0, 0.0)},
    'z': {'label': 'Z 轴', 'dir': (0.0, 0.0, 1.0)},
}


class MirrorGenerator:
    """
    生成关于指定坐标面的镜像模型

    xOy 面: 法向 (0,0,1)，经过质心时为 Z=cz 平面，镜像后 Z → 2*cz - Z
    xOz 面: 法向 (0,1,0)，经过质心时为 Y=cy 平面，镜像后 Y → 2*cy - Y
    yOz 面: 法向 (1,0,0)，经过质心时为 X=cx 平面，镜像后 X → 2*cx - X
    """

    PLANES = {
        'xOy': {'label': 'XOY 面 (Z 方向)', 'normal': (0, 0, 1)},
        'xOz': {'label': 'XOZ 面 (Y 方向)', 'normal': (0, 1, 0)},
        'yOz': {'label': 'YOZ 面 (X 方向)', 'normal': (1, 0, 0)},
    }

    def __init__(self):
        self._shape = None
        self._centroid = None

    def load(self, filepath: str):
        """加载模型（STEP / IGES / STL 都行），提取第一个 Solid/Shell。"""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"文件不存在: {filepath}")
        ext = os.path.splitext(filepath)[1].lower()
        if ext not in ('.stp', '.step'):
            # STL / IGES：统一走 model_reader（STL 会先缝合网格成实体）
            from model_reader import read_product_model
            shape = read_product_model(filepath).shape
            for face_type in (TopAbs_SOLID, TopAbs_SHELL):
                got = None
                exp = TopExp_Explorer(shape, face_type)
                while exp.More():
                    got = exp.Current()
                    break
                if got is not None:
                    shape = got
                    break
            self._shape = shape
            self._centroid = self._compute_centroid(shape)
            return shape
        result = cq.importers.importStep(filepath)
        shapes = result.vals() if hasattr(result, 'vals') else []
        if not shapes:
            raise ValueError("STEP 文件未读取到几何体")
        shape = None
        for shp in shapes:
            s = shp.wrapped
            for face_type in (TopAbs_SOLID, TopAbs_SHELL):
                exp = TopExp_Explorer(s, face_type)
                while exp.More():
                    shape = exp.Current()
                    break
                if shape:
                    break
            if shape:
                break
        if shape is None:
            raise ValueError("未找到 Solid 或 Shell 实体")
        self._shape = shape
        self._centroid = self._compute_centroid(shape)
        return shape

    def _compute_centroid(self, shape):
        """计算实体质心（体积属性），失败时退化为包围盒中心"""
        try:
            p = GProp_GProps()
            BRepGProp.VolumeProperties_s(shape, p)
            c = p.CentreOfMass()
            return (c.X(), c.Y(), c.Z())
        except Exception:
            b = Bnd_Box()
            BRepBndLib.Add_s(shape, b)
            x1, y1, z1, x2, y2, z2 = b.Get()
            return ((x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2)

    def get_centroid(self):
        return self._centroid

    def mirror(self, plane: str = 'xOy', center_on_centroid: bool = True):
        """
        应用镜像变换

        Args:
            plane: 'xOy' | 'xOz' | 'yOz'
            center_on_centroid:
                True  → 镜像面经过模型质心（平行于指定坐标面）
                False → 镜像面为坐标面本身 (Z=0 / Y=0 / X=0)

        Returns:
            镜像后的 TopoDS_Shape
        """
        if plane not in self.PLANES:
            raise ValueError(f"不支持的镜像面: {plane}，可选: {list(self.PLANES.keys())}")

        cfg = self.PLANES[plane]
        nx, ny, nz = cfg['normal']

        if center_on_centroid and self._centroid:
            ox, oy, oz = self._centroid
        else:
            ox = oy = oz = 0.0

        trsf = gp_Trsf()
        trsf.SetMirror(gp_Ax2(gp_Pnt(ox, oy, oz), gp_Dir(nx, ny, nz)))

        transformer = BRepBuilderAPI_Transform(self._shape, trsf)
        transformer.Build()
        return transformer.Shape()

    def rotate(self, axis: str = 'z', degrees: float = 90.0,
               direction: str = 'ccw', center_on_centroid: bool = True):
        """绕 x / y / z 轴旋转。

        Args:
            axis: 'x' | 'y' | 'z'
            degrees: 角度（度，正数）
            direction: 'ccw' 逆时针 / 'cw' 顺时针（从该轴正向往原点看）
            center_on_centroid: 旋转轴是否通过模型质心（否则通过坐标原点）

        Returns:
            旋转后的 TopoDS_Shape
        """
        if axis not in AXES:
            raise ValueError(f"不支持的旋转轴: {axis}，可选: {list(AXES.keys())}")
        ang = abs(float(degrees))
        if ang <= 0.0:
            raise ValueError("旋转角度必须大于 0")
        if str(direction).lower() in ('cw', 'clockwise', '顺时针', '-1'):
            ang = -ang
        dx, dy, dz = AXES[axis]['dir']
        if center_on_centroid and self._centroid:
            ox, oy, oz = self._centroid
        else:
            ox = oy = oz = 0.0
        trsf = gp_Trsf()
        trsf.SetRotation(gp_Ax1(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz)), ang * 3.141592653589793 / 180.0)
        transformer = BRepBuilderAPI_Transform(self._shape, trsf)
        transformer.Build()
        return transformer.Shape()

    def apply(self, mirror: bool = True, plane: str = 'xOy',
              center_on_centroid: bool = True,
              rotate: bool = False, axis: str = 'z', degrees: float = 90.0,
              direction: str = 'ccw', rotate_about_centroid: bool = True):
        """按顺序应用变换：先镜像、再旋转。返回 (shape, 描述后缀)。"""
        shape = self._shape
        suffix = []
        if mirror:
            shape = MirrorGenerator._as_transform(shape, self._mirror_trsf(
                plane, center_on_centroid))
            suffix.append(f"MIRROR_{plane}")
        if rotate:
            shape = MirrorGenerator._as_transform(shape, self._rotate_trsf(
                axis, degrees, direction, rotate_about_centroid))
            d = 'CW' if str(direction).lower() in ('cw', 'clockwise', '顺时针', '-1') else 'CCW'
            suffix.append(f"ROT_{axis.upper()}_{d}_{abs(float(degrees)):g}deg")
        return shape, "_".join(suffix)

    @staticmethod
    def _as_transform(shape, trsf):
        transformer = BRepBuilderAPI_Transform(shape, trsf)
        transformer.Build()
        return transformer.Shape()

    def _mirror_trsf(self, plane: str, center_on_centroid: bool = True):
        if plane not in self.PLANES:
            raise ValueError(f"不支持的镜像面: {plane}，可选: {list(self.PLANES.keys())}")
        nx, ny, nz = self.PLANES[plane]['normal']
        if center_on_centroid and self._centroid:
            ox, oy, oz = self._centroid
        else:
            ox = oy = oz = 0.0
        trsf = gp_Trsf()
        trsf.SetMirror(gp_Ax2(gp_Pnt(ox, oy, oz), gp_Dir(nx, ny, nz)))
        return trsf

    def _rotate_trsf(self, axis: str, degrees: float, direction: str = 'ccw',
                     center_on_centroid: bool = True):
        if axis not in AXES:
            raise ValueError(f"不支持的旋转轴: {axis}，可选: {list(AXES.keys())}")
        ang = abs(float(degrees))
        if ang <= 0.0:
            raise ValueError("旋转角度必须大于 0")
        if str(direction).lower() in ('cw', 'clockwise', '顺时针', '-1'):
            ang = -ang
        dx, dy, dz = AXES[axis]['dir']
        if center_on_centroid and self._centroid:
            ox, oy, oz = self._centroid
        else:
            ox = oy = oz = 0.0
        trsf = gp_Trsf()
        trsf.SetRotation(gp_Ax1(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz)),
                         ang * 3.141592653589793 / 180.0)
        return trsf

    def generate(self, filepath: str, plane: str = 'xOy',
                 output_dir: str = None, center_on_centroid: bool = True,
                 product_name: str = None, rotate: bool = False,
                 axis: str = 'z', degrees: float = 90.0, direction: str = 'ccw',
                 rotate_about_centroid: bool = True,
                 suffix: str = None, mirror: bool = True) -> dict:
        """加载模型并生成"镜像 / 旋转 / 镜像+旋转"后的模型，导出为一个新 STEP 文件。

        Args:
            filepath: 输入文件（.stp/.step/.iges/.igs/.stl）
            plane: 镜像面 'xOy' / 'xOz' / 'yOz'
            output_dir: 输出目录（默认与源文件同目录）
            center_on_centroid: 镜像面是否经过模型质心
            product_name: 输出文件名基础名（默认取"管件编号"，见 naming_utils）
            rotate/axis/degrees/direction/rotate_about_centroid: 旋转参数
            mirror: 是否做镜像（False 时只旋转）
            suffix: 直接指定文件名后缀（不传则按操作自动拼）

        Returns:
            {'output_path', 'plane', 'plane_label', 'centroid', 'bbox',
             'rotated', 'rotate_axis', 'rotate_deg', 'rotate_dir', 'mirrored'}
        """
        self.load(filepath)

        name = product_name or part_number_from_filename(filepath)
        if not output_dir:
            output_dir = os.path.dirname(os.path.abspath(filepath))
        os.makedirs(output_dir, exist_ok=True)

        shape, auto_suffix = self.apply(
            mirror=bool(mirror), plane=plane,
            center_on_centroid=bool(center_on_centroid),
            rotate=bool(rotate), axis=axis, degrees=degrees, direction=direction,
            rotate_about_centroid=bool(rotate_about_centroid))
        if not auto_suffix:
            raise ValueError("镜像和旋转至少要选一个")
        tag = suffix if suffix else auto_suffix

        out_path = os.path.join(output_dir, f"{name}_{tag}.stp")
        w = STEPControl_Writer()
        w.Transfer(shape, STEPControl_AsIs)
        if w.Write(out_path) != IFSelect_RetDone:
            raise RuntimeError(f"导出失败: {out_path}")

        b = Bnd_Box()
        BRepBndLib.Add_s(shape, b)
        x1, y1, z1, x2, y2, z2 = b.Get()

        return {
            'output_path': out_path,
            'plane': plane,
            'plane_label': self.PLANES.get(plane, {}).get('label', ''),
            'centroid': [round(v, 3) for v in self._centroid],
            'mirrored': bool(mirror),
            'rotated': bool(rotate),
            'rotate_axis': axis if rotate else None,
            'rotate_deg': abs(float(degrees)) if rotate else 0.0,
            'rotate_dir': ('CW' if str(direction).lower() in
                           ('cw', 'clockwise', '顺时针', '-1') else 'CCW') if rotate else None,
            'tag': tag,
            'bbox': {
                'min': [x1, y1, z1],
                'max': [x2, y2, z2],
                'size': [x2 - x1, y2 - y1, z2 - z1],
            },
        }


if __name__ == '__main__':
    import sys
    test_file = "YA-1131-505.stp"
    if not os.path.exists(test_file):
        test_file = os.path.join(os.path.dirname(__file__), "YA-1131-505.stp")
    if not os.path.exists(test_file):
        print("未找到测试文件 YA-1131-505.stp")
        sys.exit(1)

    out_dir = os.path.join(os.path.dirname(__file__), "mold_output")
    os.makedirs(out_dir, exist_ok=True)

    gen = MirrorGenerator()
    for plane in ('xOy', 'xOz', 'yOz'):
        try:
            r = gen.generate(test_file, plane=plane, output_dir=out_dir)
            print(f"\n[结果] {plane} 镜像: {r['output_path']}")
            print(f"  质心: {r['centroid']}")
            print(f"  镜像后包围盒: {r['bbox']['size']}")
        except Exception as e:
            print(f"[失败] {plane}: {e}")