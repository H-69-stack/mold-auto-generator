"""
长方体三维数模生成系统
功能：输入长宽高，生成长方体并导出 STEP (STP) 格式三维数模文件
"""

import os
import sys
import math

try:
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
    from OCC.Core.IFSelect import IFSelect_RetDone
    OCC_AVAILABLE = True
except ImportError:
    OCC_AVAILABLE = False

import tkinter as tk
from tkinter import ttk, messagebox, filedialog


class StlBoxWriter:
    """纯 Python ASCII STL 格式生成器 - 直接输出三角面片"""

    def __init__(self, length: float, width: float, height: float):
        self.L = max(abs(length), 1e-6)
        self.W = max(abs(width), 1e-6)
        self.H = max(abs(height), 1e-6)

    def write(self, filepath: str) -> str:
        L, W, H = self.L, self.W, self.H
        name = f'Box_{L:.0f}x{W:.0f}x{H:.0f}'

        # 8 vertices
        v = [(0,0,0), (L,0,0), (L,W,0), (0,W,0),
             (0,0,H), (L,0,H), (L,W,H), (0,W,H)]

        # 6 faces, each face = 2 triangles, outward normal
        # Face indices (CCW from outside): bottom, top, front, back, left, right
        faces = [
            (0,3,2,1, (0,0,-1)),    # bottom
            (4,5,6,7, (0,0,1)),     # top
            (0,1,5,4, (0,-1,0)),    # front
            (3,7,6,2, (0,1,0)),     # back
            (0,4,7,3, (-1,0,0)),    # left
            (1,2,6,5, (1,0,0)),     # right
        ]

        lines = [f"solid {name}"]
        for a, b, c, d, normal in faces:
            lines.append(f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}")
            lines.append("    outer loop")
            for idx in (a, b, c):
                lines.append(f"      vertex {v[idx][0]:.6f} {v[idx][1]:.6f} {v[idx][2]:.6f}")
            lines.append("    endloop")
            lines.append("  endfacet")
            lines.append(f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}")
            lines.append("    outer loop")
            for idx in (a, c, d):
                lines.append(f"      vertex {v[idx][0]:.6f} {v[idx][1]:.6f} {v[idx][2]:.6f}")
            lines.append("    endloop")
            lines.append("  endfacet")
        lines.append(f"endsolid {name}")

        with open(filepath, 'w', encoding='ascii') as f:
            f.write('\n'.join(lines) + '\n')
        return filepath


class StepBoxWriter:
    """纯 Python ISO 10303-21 STEP AP214 格式生成器 - 长方体 B-Rep 边界表示"""

    def __init__(self, length: float, width: float, height: float):
        self.L = max(abs(length), 1e-6)
        self.W = max(abs(width), 1e-6)
        self.H = max(abs(height), 1e-6)
        self._next = 1
        self._buf: list[str] = []
        self._edge_curve_ids: dict = {}

    def _nid(self) -> int:
        n = self._next
        self._next += 1
        return n

    def _put(self, s: str) -> None:
        self._buf.append(s)

    def write(self, filepath: str) -> str:
        return self._build(filepath)

    def _make_face(self, point_id: int, normal_id: int, ref_id: int,
                   edges: list[tuple]) -> int:
        ap = self._nid()
        self._put(f"#{ap}=AXIS2_PLACEMENT_3D('',#{point_id},#{normal_id},#{ref_id});")
        pl = self._nid()
        self._put(f"#{pl}=PLANE('',#{ap});")
        oe_ids = []
        for s, e, fwd in edges:
            oe = self._nid()
            ec = self._edge_curve_ids.get((s, e))
            if ec is None:
                ec = self._edge_curve_ids.get((e, s))
                fwd = not fwd
            flag = '.T.' if fwd else '.F.'
            self._put(f"#{oe}=ORIENTED_EDGE('',*,*,#{ec},{flag});")
            oe_ids.append(oe)
        el = self._nid()
        self._put(f"#{el}=EDGE_LOOP('',({','.join(f'#{i}' for i in oe_ids)}));")
        fob = self._nid()
        self._put(f"#{fob}=FACE_OUTER_BOUND('',#{el},.T.);")
        af = self._nid()
        self._put(f"#{af}=ADVANCED_FACE('',(#{fob}),#{pl},.T.);")
        return af

    def _build(self, filepath: str) -> str:
        L, W, H = self.L, self.W, self.H
        name = f'BOX_{L:.1f}x{W:.1f}x{H:.1f}'
        dt = '2026-07-25T15:00:00+08:00'

        ctx_id = self._nid(); apd_id = self._nid(); prd_id = self._nid()
        pdf_id = self._nid(); pdd_id = self._nid(); pdc_id = self._nid()
        prpc_id= self._nid(); pds_id = self._nid(); sdr_id = self._nid()
        sr_id  = self._nid(); mc_id  = self._nid(); de_id  = self._nid()
        si_l_id= self._nid(); si_sa_id=self._nid(); lmu_id = self._nid()
        cbu_id = self._nid(); umu_id = self._nid(); grc_id = self._nid()

        # 8 Cartesian Points
        pts = {}
        for pt in ((0,0,0),(L,0,0),(L,W,0),(0,W,0),(0,0,H),(L,0,H),(L,W,H),(0,W,H)):
            nid = self._nid(); pts[pt] = nid
            self._put(f"#{nid}=CARTESIAN_POINT('',({pt[0]},{pt[1]},{pt[2]}));")

        vtx = {}
        for pt, cpid in pts.items():
            nid = self._nid(); vtx[pt] = nid
            self._put(f"#{nid}=VERTEX_POINT('',#{cpid});")

        dir_ids = {}
        for d in ((1,0,0),(0,1,0),(0,0,1),(-1,0,0),(0,-1,0),(0,0,-1)):
            nid = self._nid(); dir_ids[d] = nid
            self._put(f"#{nid}=DIRECTION('',({d[0]},{d[1]},{d[2]}));")

        vec_ids = {}
        for d in ((1,0,0),(0,1,0),(0,0,1),(-1,0,0),(0,-1,0),(0,0,-1)):
            nid = self._nid(); vec_ids[d] = nid
            self._put(f"#{nid}=VECTOR('',#{dir_ids[d]},1000.);")

        for s, e, d in [
            ((0,0,0),(L,0,0),(1,0,0)),((L,0,0),(L,W,0),(0,1,0)),
            ((L,W,0),(0,W,0),(-1,0,0)),((0,W,0),(0,0,0),(0,-1,0)),
            ((0,0,H),(L,0,H),(1,0,0)),((L,0,H),(L,W,H),(0,1,0)),
            ((L,W,H),(0,W,H),(-1,0,0)),((0,W,H),(0,0,H),(0,-1,0)),
            ((0,0,0),(0,0,H),(0,0,1)),((L,0,0),(L,0,H),(0,0,1)),
            ((L,W,0),(L,W,H),(0,0,1)),((0,W,0),(0,W,H),(0,0,1)),
        ]:
            ln = self._nid()
            self._put(f"#{ln}=LINE('',#{pts[s]},#{vec_ids[d]});")
            ec = self._nid()
            self._put(f"#{ec}=EDGE_CURVE('',#{vtx[s]},#{vtx[e]},#{ln},.T.);")
            self._edge_curve_ids[(s, e)] = ec

        face_ids = []

        # Bottom (z=0, outward -Z): loop v1→v2→v3→v4→v1 (CCW from below)
        face_ids.append(self._make_face(
            pts[(0,0,0)], dir_ids[(0,0,-1)], dir_ids[(1,0,0)],
            [((0,0,0),(L,0,0),True),((L,0,0),(L,W,0),True),
             ((L,W,0),(0,W,0),True),((0,W,0),(0,0,0),True)]))

        # Top (z=H, outward +Z): loop v5→v8→v7→v6→v5 (CW from above → CCW when viewed from +Z)
        face_ids.append(self._make_face(
            pts[(0,0,H)], dir_ids[(0,0,1)], dir_ids[(1,0,0)],
            [((0,0,H),(0,W,H),True),((0,W,H),(L,W,H),True),
             ((L,W,H),(L,0,H),True),((L,0,H),(0,0,H),True)]))

        # Front (y=0, outward -Y): loop v2→v6→v5→v1→v2 (CCW from outside -Y)
        face_ids.append(self._make_face(
            pts[(0,0,0)], dir_ids[(0,-1,0)], dir_ids[(1,0,0)],
            [((0,0,0),(L,0,0),True),((L,0,0),(L,0,H),True),
             ((L,0,H),(0,0,H),True),((0,0,H),(0,0,0),True)]))

        # Back (y=W, outward +Y): loop v3→v4→v8→v7→v3
        face_ids.append(self._make_face(
            pts[(0,W,0)], dir_ids[(0,1,0)], dir_ids[(1,0,0)],
            [((0,W,0),(L,W,0),True),((L,W,0),(L,W,H),True),
             ((L,W,H),(0,W,H),True),((0,W,H),(0,W,0),True)]))

        # Left (x=0, outward -X): loop v1→v5→v8→v4→v1
        face_ids.append(self._make_face(
            pts[(0,0,0)], dir_ids[(-1,0,0)], dir_ids[(0,1,0)],
            [((0,0,0),(0,W,0),True),((0,W,0),(0,W,H),True),
             ((0,W,H),(0,0,H),True),((0,0,H),(0,0,0),True)]))

        # Right (x=L, outward +X): loop v3→v7→v6→v2→v3
        face_ids.append(self._make_face(
            pts[(L,0,0)], dir_ids[(1,0,0)], dir_ids[(0,1,0)],
            [((L,0,0),(L,W,0),True),((L,W,0),(L,W,H),True),
             ((L,W,H),(L,0,H),True),((L,0,H),(L,0,0),True)]))

        cs = self._nid()
        self._put(f"#{cs}=CLOSED_SHELL('',({','.join(f'#{i}' for i in face_ids)}));")
        msb = self._nid()
        self._put(f"#{msb}=MANIFOLD_SOLID_BREP('',#{cs});")

        header = [
            f"#{ctx_id}=APPLICATION_CONTEXT('core data for automotive mechanical design processes');",
            f"#{apd_id}=APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ctx_id});",
            f"#{prd_id}=PRODUCT('{name}','{name}','',(#{mc_id}));",
            f"#{pdf_id}=PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#{prd_id},.NOT_KNOWN.);",
            f"#{pdc_id}=PRODUCT_DEFINITION_CONTEXT('detailed design',#{apd_id},'design');",
            f"#{pdd_id}=PRODUCT_DEFINITION('design','',#{pdf_id},#{pdc_id});",
            f"#{prpc_id}=PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{prd_id}));",
            f"#{mc_id}=MECHANICAL_CONTEXT('',#{apd_id},'mechanical');",
            f"#{de_id}=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);",
            f"#{si_l_id}=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));",
            f"#{si_sa_id}=(NAMED_UNIT(*)SI_UNIT(*,.STERADIAN.)SOLID_ANGLE_UNIT());",
            f"#{lmu_id}=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.),#{si_l_id});",
            f"#{cbu_id}=CONVERSION_BASED_UNIT('degree',#{lmu_id})PLANE_ANGLE_UNIT()NAMED_UNIT(*);",
            f"#{umu_id}=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#{si_l_id},'DISTANCE_ACCURACY_VALUE','Maximum model space distance between geometric entities at asserted connectivities');",
            f"#{grc_id}=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{si_l_id},#{cbu_id},#{si_sa_id}))GLOBAL_UNIT_ASSIGNED_CONTEXT((#{umu_id})))REPRESENTATION_CONTEXT('','');",
            f"#{sr_id}=SHAPE_REPRESENTATION('',(#{msb}),#{grc_id});",
            f"#{pds_id}=PRODUCT_DEFINITION_SHAPE('','',#{pdd_id});",
            f"#{sdr_id}=SHAPE_DEFINITION_REPRESENTATION(#{pds_id},#{sr_id});",
        ]
        self._buf = header + self._buf

        with open(filepath, 'w', encoding='ascii') as f:
            f.write("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('BoxGenerator'),'2;1');\n")
            f.write(f"FILE_NAME('{name}.stp','{dt}',('BoxGenerator'),(''),'FreeCAD','BoxGenerator','');\n")
            f.write("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 '));\nENDSEC;\n\nDATA;\n")
            for line in self._buf:
                f.write(line + "\n")
            f.write("ENDSEC;\n\nEND-ISO-10303-21;\n")
        return filepath


class BoxGeneratorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("长方体三维数模生成系统")
        self.root.geometry("540x520")
        self.root.resizable(False, False)
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.bg = "#f5f7fa"; self.pc = "#2563eb"; self.ac = "#16a34a"; self.tc = "#1e293b"
        self.root.configure(bg=self.bg)
        self.style.configure('TFrame', background=self.bg)
        self.style.configure('TLabel', background=self.bg, foreground=self.tc, font=('Microsoft YaHei', 10))
        self.style.configure('TButton', font=('Microsoft YaHei', 10), padding=8)
        self.style.configure('TEntry', font=('Microsoft YaHei', 11), padding=6)
        self.default_output_dir = os.path.join(os.path.expanduser("~"), "Desktop", "BoxModels")
        os.makedirs(self.default_output_dir, exist_ok=True)
        self._build_ui()

    def _build_ui(self):
        hf = tk.Frame(self.root, bg=self.pc, height=60)
        hf.pack(fill=tk.X); hf.pack_propagate(False)
        tk.Label(hf, text="📦 长方体三维数模生成系统", font=('Microsoft YaHei', 16, 'bold'), fg="white", bg=self.pc).pack(expand=True)
        mf = ttk.Frame(self.root, padding=(25, 15)); mf.pack(fill=tk.BOTH, expand=True)
        df = tk.LabelFrame(mf, text="  📐 请输入长方体尺寸参数  ", font=('Microsoft YaHei', 11, 'bold'), fg=self.pc, bg=self.bg, padx=20, pady=15, relief=tk.GROOVE, bd=1)
        df.pack(fill=tk.X, pady=(0, 12))
        for label, vn, default in [("长 度 (X)：",'length',"100.0"),("宽 度 (Y)：",'width',"50.0"),("高 度 (Z)：",'height',"30.0")]:
            fr = tk.Frame(df, bg=self.bg); fr.pack(fill=tk.X, pady=5)
            tk.Label(fr, text=label, font=('Microsoft YaHei', 11), bg=self.bg).pack(side=tk.LEFT)
            var = tk.StringVar(value=default); setattr(self, f'{vn}_var', var)
            ttk.Entry(fr, textvariable=var, width=22, font=('Microsoft YaHei', 11)).pack(side=tk.RIGHT, padx=(5,10))
            tk.Label(fr, text="mm", font=('Microsoft YaHei', 10), bg=self.bg, fg="#64748b").pack(side=tk.RIGHT)
        pf = tk.LabelFrame(mf, text="  💾 输出设置  ", font=('Microsoft YaHei', 11, 'bold'), fg=self.pc, bg=self.bg, padx=20, pady=15, relief=tk.GROOVE, bd=1)
        pf.pack(fill=tk.X, pady=(0, 12))
        ofr = tk.Frame(pf, bg=self.bg); ofr.pack(fill=tk.X, pady=5)
        tk.Label(ofr, text="保存路径：", font=('Microsoft YaHei', 11), bg=self.bg).pack(side=tk.LEFT)
        self.output_var = tk.StringVar(value=self.default_output_dir)
        ttk.Entry(ofr, textvariable=self.output_var, width=32, font=('Microsoft YaHei', 10)).pack(side=tk.LEFT, padx=(5,5), fill=tk.X, expand=True)
        ttk.Button(ofr, text="浏览...", command=self._browse, width=8).pack(side=tk.RIGHT)
        nfr = tk.Frame(pf, bg=self.bg); nfr.pack(fill=tk.X, pady=5)
        tk.Label(nfr, text="文件名称：", font=('Microsoft YaHei', 11), bg=self.bg).pack(side=tk.LEFT)
        self.filename_var = tk.StringVar(value="Box_100x50x30")
        ttk.Entry(nfr, textvariable=self.filename_var, width=25, font=('Microsoft YaHei', 10)).pack(side=tk.RIGHT, padx=(5,10))
        tk.Label(nfr, text=".stp", font=('Microsoft YaHei', 10), bg=self.bg, fg="#64748b").pack(side=tk.RIGHT)
        bf = tk.Frame(mf, bg=self.bg); bf.pack(fill=tk.X, pady=(5,10))
        self.preview_label = tk.Label(bf, text="将生成：100.0 × 50.0 × 30.0 mm 的长方体", font=('Microsoft YaHei', 9), fg="#64748b", bg=self.bg)
        self.preview_label.pack(pady=(0,10))
        eng = "pythonocc-core (OpenCASCADE)" if OCC_AVAILABLE else "纯 Python STEP 生成器"
        tk.Label(bf, text=f"引擎: {eng}", font=('Microsoft YaHei', 8), fg="#16a34a" if OCC_AVAILABLE else "#d97706", bg=self.bg).pack(pady=(0,5))
        self.gen_btn = tk.Button(bf, text="🔨  生成长方体并导出 STP", font=('Microsoft YaHei', 12, 'bold'), fg="white", bg=self.ac, activebackground="#15803d", activeforeground="white", relief=tk.FLAT, padx=25, pady=10, cursor="hand2", command=self._generate)
        self.gen_btn.pack(pady=5)
        self.status_var = tk.StringVar(value="✅ 就绪 — 输入参数后点击生成按钮")
        tk.Label(self.root, textvariable=self.status_var, font=('Microsoft YaHei', 9), fg="#475569", bg="#e2e8f0", anchor=tk.W, padx=12, pady=6).pack(fill=tk.X, side=tk.BOTTOM)
        for v in (self.length_var, self.width_var, self.height_var):
            v.trace_add('write', lambda *a: self._preview())

    def _preview(self):
        try:
            l = float(self.length_var.get()); w = float(self.width_var.get()); h = float(self.height_var.get())
            self.preview_label.config(text=f"将生成：{l} × {w} × {h} mm 的长方体")
            self.filename_var.set(f"Box_{l:.0f}x{w:.0f}x{h:.0f}")
        except ValueError:
            pass

    def _browse(self):
        d = filedialog.askdirectory(initialdir=self.output_var.get(), title="选择保存目录")
        if d: self.output_var.set(d)

    def _generate(self):
        try:
            L = float(self.length_var.get()); W = float(self.width_var.get()); H = float(self.height_var.get())
        except ValueError:
            messagebox.showerror("输入错误", "请输入有效的数字"); return
        if L <= 0 or W <= 0 or H <= 0:
            messagebox.showerror("输入错误", "长、宽、高必须大于 0"); return
        out_dir = self.output_var.get().strip() or self.default_output_dir
        self.output_var.set(out_dir); os.makedirs(out_dir, exist_ok=True)
        fname = self.filename_var.get().strip() or f"Box_{L:.0f}x{W:.0f}x{H:.0f}"
        self.filename_var.set(fname)
        stp_path = os.path.join(out_dir, f"{fname}.stp")
        self.status_var.set("⏳ 正在生成...")
        self.gen_btn.config(state=tk.DISABLED, text="⏳ 生成中..."); self.root.update()
        try:
            if OCC_AVAILABLE:
                box = BRepPrimAPI_MakeBox(L, W, H).Shape()
                sw = STEPControl_Writer(); sw.Transfer(box, STEPControl_AsIs)
                if sw.Write(stp_path) != IFSelect_RetDone: raise RuntimeError("写入失败")
            else:
                StepBoxWriter(L, W, H).write(stp_path)
            self.status_var.set(f"✅ 已保存：{stp_path}")
            self.gen_btn.config(state=tk.NORMAL, text="🔨  生成长方体并导出 STP")
            messagebox.showinfo("成功", f"已保存到：\n{stp_path}\n尺寸：{L}×{W}×{H} mm")
        except Exception as e:
            self.status_var.set(f"❌ 失败：{e}")
            self.gen_btn.config(state=tk.NORMAL, text="🔨  生成长方体并导出 STP")
            messagebox.showerror("失败", str(e))


def main():
    BoxGeneratorApp(tk.Tk()); tk.mainloop()


if __name__ == "__main__":
    main()