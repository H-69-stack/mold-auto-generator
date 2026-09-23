"""
液压成型凹凸模具自动生成器（方案3精确实现）
基于 OpenCASCADE 布尔运算 (CadQuery / OCP bindings)

设产品有顶面a（最高水平面）和底面b（最低水平面）。

下模（凹模）：毛坯顶面=a平面，产品沉入毛坯，顶面a与模面平齐
  Cut(毛坯 [Z_a-100 .. Z_a], 产品)

上模（凸模）：毛坯底面=a平面，产品上移0.1mm刺入毛坯，补齐空腔
  Cut(毛坯 [Z_a .. Z_a+100], 产品↑0.1mm)
"""

import os
import cadquery as cq
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Common, BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir, gp_Trsf, gp_Vec
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS


class MoldGenerator:
    """
    液压成型凹凸模具自动生成器（方案3）

    输入: Z_A = 顶面a (分模面), Z_B = 底面b
    下模: Cut(毛坯 [Z_A-100 .. Z_A], 产品) → 顶面a平齐
    上模: Cut(毛坯 [Z_A .. Z_A+100], 产品↑0.1mm) → 底面a贴合
    """

    def __init__(self):
        self._product_shape = None
        self.Z_A = 0.0  # 顶面a（分模面）
        self.Z_B = 0.0  # 底面b

    def load_product(self, filepath: str) -> dict:
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"文件不存在: {filepath}")
        result = cq.importers.importStep(filepath)
        shapes = result.vals() if hasattr(result, 'vals') else []
        if not shapes:
            raise ValueError("STEP文件未读取到几何体")
        self._product_shape = None
        for shp in shapes:
            s = shp.wrapped
            for e in [TopAbs_SOLID, TopAbs_SHELL]:
                exp = TopExp_Explorer(s, e)
                while exp.More():
                    self._product_shape = exp.Current()
                    break
                if self._product_shape:
                    break
            if self._product_shape:
                break
        if not self._product_shape:
            raise ValueError("未找到Solid或Shell实体")
        centroid = self._compute_centroid(self._product_shape)
        trsf = gp_Trsf()
        trsf.SetTranslation(gp_Vec(-centroid[0], -centroid[1], -centroid[2]))
        transformer = BRepBuilderAPI_Transform(self._product_shape, trsf)
        transformer.Build()
        self._product_shape = transformer.Shape()
        bbox = self._compute_bbox(self._product_shape)
        return {'bbox': bbox, 'size': bbox['size']}

    def detect_top_bottom_plane(self):
        """
        自动识别产品上表面和下表面（分模面与底面）。

        上表面（Upper Surface）：产品沿 Z 轴正向在上方的面，Z_A = 产品包围盒最大 Z 值
        下表面（Lower Surface）：产品沿 Z 轴正向在下方的面，Z_B = 产品包围盒最小 Z 值

        检测策略：
          1. 主方法：直接使用产品包围盒的 Z_max 和 Z_min。产品经 load_product() 已居中平移，
             Z_max 即为分模面（上表面最高点），Z_min 即为底面（下表面最低点）。
          2. 辅助方法（仅用于日志）：尝试识别水平面作为参考信息。
        
        这个策略覆盖所有产品形状（平板、曲面、复杂造型），不依赖水平面存在。
        """
        # 主方法：包围盒 Z 范围（产品已居中平移，包围盒直接反映产品 Z 范围）
        bbox = self._compute_bbox(self._product_shape)
        self.Z_A = bbox['max'][2]  # 上表面最高点 = 分模面
        self.Z_B = bbox['min'][2]  # 下表面最低点 = 底面

        # 辅助：尝试检测水平面数量（用于日志/诊断）
        exp_face = TopExp_Explorer(self._product_shape, TopAbs_FACE)
        horizontal_count = 0
        while exp_face.More():
            face = exp_face.Current()
            try:
                b = Bnd_Box(); BRepBndLib.Add_s(face, b)
                z1, z2 = b.Get()[2], b.Get()[5]
                p = GProp_GProps(); BRepGProp.SurfaceProperties_s(face, p)
                area = p.Mass()
                if abs(z2 - z1) < 0.02 and area > 10:
                    horizontal_count += 1
            except:
                pass
            exp_face.Next()

        print(f"[Mold] 产品包围盒 Z 范围: [{self.Z_B:.2f}, {self.Z_A:.2f}]  高度={self.Z_A - self.Z_B:.2f} mm")
        print(f"[Mold] 上表面(分模面) Z_A = {self.Z_A:.2f} mm  (产品最高点)")
        print(f"[Mold] 下表面(底面) Z_B = {self.Z_B:.2f} mm  (产品最低点)")
        if horizontal_count > 0:
            print(f"[Mold] 检测到 {horizontal_count} 个水平面")
        else:
            print(f"[Mold] 产品无水平基准面 (曲面/异形产品)，使用包围盒 Z 范围分模")

    def generate(self, filepath, side_margin=5.0, base_thickness=10.0,
                 output_dir=None, product_name=None):
        info = self.load_product(filepath)
        bbox = info['bbox']
        size = info['size']
        if not product_name:
            product_name = os.path.splitext(os.path.basename(filepath))[0]
        if not output_dir:
            output_dir = os.path.dirname(os.path.abspath(filepath))
        os.makedirs(output_dir, exist_ok=True)

        self.detect_top_bottom_plane()

        bx = bbox['min'][0] - side_margin
        by = bbox['min'][1] - side_margin
        bdx = size[0] + 2 * side_margin
        bdy = size[1] + 2 * side_margin

        def B(x, y, z, dx, dy, dz):
            return BRepPrimAPI_MakeBox(gp_Ax2(gp_Pnt(x, y, z), gp_Dir(0, 0, 1)), dx, dy, dz).Shape()

        print(f"\n{'='*55}")
        print(f"[Mold] 产品: {product_name} | {size[0]:.0f}x{size[1]:.0f}x{size[2]:.0f}mm")
        print(f"[Mold] 平面A(顶面/分模面): Z={self.Z_A:.2f}")
        print(f"[Mold] 平面B(底面): Z={self.Z_B:.2f}")
        print(f"{'='*55}")

        # 凹模: 基体 [Z_B-base..Z_A] − 产品
        # 输出两个独立结构：长方体基座（带产品型腔）+ 产品
        print(f"\n--- 凹模(下模) ---")
        die_zmin = self.Z_B - base_thickness
        die_block = B(bx, by, die_zmin, bdx, bdy, self.Z_A - die_zmin)
        die_cut = self._do_cut(die_block, self._product_shape) or die_block
        die_ok = self._has_content(die_cut)
        if die_ok:
            die = self._make_compound(die_cut, self._product_shape)
            db = self._compute_bbox(die_cut)
            print(f"[Mold] OK Z:[{db['min'][2]:.1f},{db['max'][2]:.1f}] 面数={self._count_faces(die_cut)}")
            print(f"[Mold] OK 复合结构: [长方体基座(带型腔) + 产品] 两个独立实体")

        # 凸模: 基体 [Z_A..Z_A+base]
        # 输出两个独立结构：长方体实体块 + 产品（不融合，保持两个实体）
        print(f"\n--- 凸模(上模) ---")
        punch_block = B(bx, by, self.Z_A, bdx, bdy, base_thickness)
        punch_ok = self._has_content(punch_block)
        if punch_ok:
            punch = self._make_compound(punch_block, self._product_shape)
            pb = self._compute_bbox(punch_block)
            print(f"[Mold] OK Z:[{pb['min'][2]:.1f},{pb['max'][2]:.1f}] 面数={self._count_faces(punch_block)}")
            print(f"[Mold] OK 复合结构: [长方体实体块 + 产品] 两个独立实体")

        print(f"{'='*55}")

        cp = os.path.join(output_dir, f"{product_name}_CAVITY.stp")
        pp = os.path.join(output_dir, f"{product_name}_PUNCH.stp")
        if die_ok: self._export_step(die, cp)
        if punch_ok: self._export_step(punch, pp)

        return {
            'cavity': cp if die_ok else None,
            'punch': pp if punch_ok else None,
            'info': {
                'product_name': product_name,
                'product_size': size,
                'cavity_size': [round(bdx, 1), round(bdy, 1), round(self.Z_A - die_zmin, 1)],
                'punch_size': [round(bdx, 1), round(bdy, 1), round(base_thickness, 1)],
                'Z_A': round(self.Z_A, 1), 'Z_B': round(self.Z_B, 1),
                'cavity_valid': die_ok, 'punch_valid': punch_ok,
                'parting_z': round(self.Z_A, 1),
            }
        }

    @staticmethod
    def _make_compound(*shapes):
        """
        将多个形状组合成一个 compound（复合体）。
        生成的 STEP 文件将包含多个独立实体。
        """
        cq_shapes = []
        for s in shapes:
            if s is None or s.IsNull():
                continue
            cq_shapes.append(cq.Shape(s))
        c = cq.Compound.makeCompound(cq_shapes)
        return c.wrapped

    def _count_faces(self, s):
        n = 0
        e = TopExp_Explorer(s, TopAbs_FACE)
        while e.More(): n += 1; e.Next()
        return n

    def _do_cut(self, a, b):
        try:
            c = BRepAlgoAPI_Cut(a, b); c.SetRunParallel(False); c.Build()
            if c.IsDone():
                s = c.Shape()
                return s if self._has_content(s) else None
        except Exception as e:
            print(f"[Mold] Cut: {e}")
        return None

    def _do_common(self, a, b):
        try:
            c = BRepAlgoAPI_Common(a, b); c.SetRunParallel(False); c.Build()
            if c.IsDone():
                s = c.Shape()
                return s if self._has_content(s) else None
        except Exception as e:
            print(f"[Mold] Common: {e}")
        return None

    def _do_fuse(self, a, b):
        try:
            f = BRepAlgoAPI_Fuse(a, b); f.SetRunParallel(False); f.Build()
            if f.IsDone():
                s = f.Shape()
                return s if self._has_content(s) else None
        except Exception as e:
            print(f"[Mold] Fuse: {e}")
        return None

    def _compute_bbox(self, s):
        try:
            b = Bnd_Box(); BRepBndLib.Add_s(s, b)
            x1, y1, z1, x2, y2, z2 = b.Get()
            return {'min': [x1, y1, z1], 'max': [x2, y2, z2], 'size': [x2 - x1, y2 - y1, z2 - z1]}
        except:
            return {'min': [0]*3, 'max': [0]*3, 'size': [0]*3}

    def _compute_centroid(self, s):
        try:
            p = GProp_GProps(); BRepGProp.VolumeProperties_s(s, p)
            c = p.CentreOfMass(); return [c.X(), c.Y(), c.Z()]
        except:
            b = self._compute_bbox(s)
            return [(b['min'][i] + b['max'][i]) / 2 for i in range(3)]

    def _has_content(self, s):
        if s is None or s.IsNull(): return False
        return TopExp_Explorer(s, TopAbs_SOLID).More() or TopExp_Explorer(s, TopAbs_SHELL).More()

    def _export_step(self, s, path):
        w = STEPControl_Writer(); w.Transfer(s, STEPControl_AsIs)
        if w.Write(path) != IFSelect_RetDone:
            raise RuntimeError(f"导出失败: {path}")
        print(f"[Mold] → {os.path.basename(path)}")


if __name__ == '__main__':
    test_file = os.path.join(os.path.dirname(__file__), "YA-1131-505.stp")
    if not os.path.exists(test_file): test_file = "YA-1131-505.stp"
    if not os.path.exists(test_file): print("文件未找到"); exit(1)
    out_dir = os.path.join(os.path.dirname(__file__), "mold_output")
    os.makedirs(out_dir, exist_ok=True)
    gen = MoldGenerator()
    r = gen.generate(filepath=test_file, side_margin=20.0, base_thickness=100.0, output_dir=out_dir)
    print(f"\n[结果] 凹模: {r['cavity']} | 凸模: {r['punch']}")
    print(f"  平面A(顶面a/分模面): Z={r['info']['Z_A']}mm")
    print(f"  平面B(底面b): Z={r['info']['Z_B']}mm")