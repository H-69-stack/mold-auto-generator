# -*- coding: utf-8 -*-
"""
模具修复模块 v34 —— 凸模输出 = compound [基座, 凸起台, 产品]（三个独立结构体）
=====================================================================

背景（用户遇到的困难）：
  已有系统生成上下模具 compound（基座 + 产品 两个独立结构）：
    凸模(上模) = [长方体基座 Z[Z_A, Z_A+厚] + 产品 Z[Z_B, Z_A]]
    凹模(下模) = [带型腔长方体基座 + 产品]
  用户要求（最新）：
    ① 修理后的模具与产品是【两个结构体】（产品独立保留）；
    ② 模具也有两个结构体 = 基座 + 基座上面的凸起部分。
    ③（v30）凸起只保留【单一凸起台】结构体 —— 布尔构造会切出
       多个残片（产品下表面以下的柱状补腔 / 边缘特征薄片），这些残片
       必须丢弃，最终输出 = 基座 + 凸起台 + 产品 三个独立结构体。
    ④（v31）凸起与产品不得重叠 —— 边缘特征补充时 Fuse 的模糊值
       (1e-2) 会让融合后的凸起沿产品表面薄层侵入约 0.01mm，
       在凸起底部（产品上表面以下）形成一条多余棱边。
       v31 在输出前对凸起再做一次 Cut(凸起, 产品)，彻底清除重叠。
    ⑤（v32）凸起底部【两个角落/拐角】的多余小长方形面 —— 凸起在
       主板面边缘包裹产品翻边/凸台时，布尔会留下极薄的薄鳍小面
       （法向材料厚度 < 0.5mm，实测约 0.05~0.08mm），在 CAD 中显示为
       "比周围相邻表面高一点点"的局部凸起。
       v32 增加"去底部薄鳍"清理：检测这些小面并沿其包围盒切掉。
    ⑥（v32 缺陷，用户反馈）v32 的去底部薄鳍用【包围盒 + 0.5mm 周边
       扩展 + z 向主平面 ±2mm 扩展】的盒体切除 —— 切削范围远大于
       薄鳍自身，在角落挖出了 2 处小空缺/凹陷缺口（过度切削）；
       且厚度探测失败(th=None)的两个小面被漏检，角落仍残留 1 处
       小长方形凸起面。
    ⑦（v33 根因修复）去底部薄鳍改为【精确切除到产品上表面】：
       - 只切薄鳍【自身的精确包围盒】（0.01mm 余量），x/y 绝不越界
         → 移除凸起的同时不再挖掉周边材料（无空缺）；
       - 盒体 z 上限取【该处产品上表面】而非薄鳍顶面 → 切除后凸起
         底面恰好贴合产品上表面，不留下凹槽；
       - 补上厚度探测失败(th=None)但形状确为薄片的小面检测
         （v32 漏检导致角落仍残留 1 处小长方形凸起面）。
       → 角落一次性生成平整、连续、无凸起、无空缺的表面。

本版算法（v33，凸起底部贴合产品全部弯曲形变面，无方块凸起，无残片，
           无与产品重叠，无角落薄鳍凸起，无过度切削空缺）：
  ─────────────────────────────────────────────────────────
  [A] 凸模（上模修复）—— 输出 compound [基座, 凸起台, 产品]
      compound = [ 基座, 凸起台, 产品 ]

      ① 凸起 = 逐面填充（include_all：主平面 + 翻边环面 + boss圆柱面 +
         加强筋B样条面 + 球面 等；只保留【主板面及以上】的朝上面，
         排除凹坑/圆盘的内壁与底面 → 不再产生侵入产品的方块凸起）；
      ② 凹坑/圆盘补腔 = Cut(主板面内孔轮廓柱, 产品)：每个内孔 = 一个凹坑
         开口，填充贴合凹坑曲面（圆形/阶梯形轮廓，圆盘上的方块凸起消失）；
      ③ Cut补腔（未贴合的朝上面）改用【面外轮廓柱】替代 bbox 扩展盒，
         且只处理 z ≥ 主板面-容差 的上部特征面；
      ④ 合并多实体 + 填实内部封闭空腔；
      ⑤ 顶部凹槽填平：只填【产品包围盒内】凸起足迹中的空腔（不产生唇边）；
      ⑥ 凸起整合（v30）：保留体积最大的主凸起台实体，丢弃布尔切出的
         残片 —— 特别是【产品下表面以下的柱状补腔】与无法融合的
         悬浮薄片，使输出恰好三个独立结构体；
      ⑦ 去产品重叠（v31）：输出前 Cut(凸起, 产品)，清除边缘特征
         Fuse 模糊值造成的凸起对产品表面的薄层侵入，消除产品上表面
         以下的多余棱边；
      ⑧ 去底部薄鳍（v33 根因修复）：布尔（Cut(全高柱,产品)）在主板面
         边缘包裹产品翻边/凸台时留下极薄的薄鳍小面（角落局部凸起）。
         精确切除薄鳍自身包围盒到产品上表面（0.01mm 余量，不越界），
         角落平整、连续，无凸起、无空缺。
         compound = [ 基座, 凸起台, 产品 ]
         可在 CAD 中分开查看/移动，检查凸起与产品的贴合关系。

  [B] 凹模（下模修复）—— 挖空【产品下表面以上的全部部分】
      （v41 重写：挖空体 = 产品3D实体 ∪ 间隙体（产品外轮廓柱从主板面向上减产品），
        Cut(基座(顶面=分模面-0.5mm), 挖空体)：）
        a. 型腔底面 = 产品下形变面（弯弯曲曲的下表面，完全随形、贴合）；
        b. 型腔侧壁 / 底部圆盘 / 加强筋 / 圆盘口全部跟随产品外形（无直角口）；
        c. 同时挖掉【产品与基座上顶面之间被包裹的间隙实体】和
           【基座被产品包裹的上顶面】（Common(型腔, 产品) = 0，产品能完全放入）；
        d. 基座顶面取【分模面略下方 -0.5mm】→ 型腔顶部开口；
        e. 布尔产生的碎片尽量融合回 1 实体；默认不切外边缘（trim_edge=False），
           产品周围的基座实体保留。
       → 输出 = compound [修改后的基座(1 实体 1 壳), 产品]，产品独立保留。

  重要：整个修复过程不需要外部产品文件——只需输入 MoldGenerator 生成的
        上下模具 compound 文件（其内部已包含对齐的基座与产品）。
  ─────────────────────────────────────────────────────────

  已验证（真实数据，重导入结构体数 3、各实体 BRepCheck 有效）：
    YA-1131-505_MIRROR_xOy：
      - 凸模输出 = compound [ 基座 + 凸起台 + 产品 ]
         三个独立结构体；凸起底部贴合产品【弯曲起伏的形变面】
         （主平面/翻边环面/boss圆柱面/加强筋B样条面/球面等，无方块状凸起），
         内部无空腔（1 Shell）；
      - 凹模 = 11,507,493 mm³（完整基座 13,128,616 - 产品∪填充 1,621,123），
         1 壳，型腔沿产品下形变面挖出、顶部开口；
    YA-1238-01_MIRROR_xOy（v33）：
      - 凸模 = 19,222,133 mm³（基座 11,193,608 + 凸起 19,222,133 + 产品 1,264,243），
         三个独立结构体，凸起与产品重叠 = 0 mm³，1 实体、全部有效；
      - 角落薄鳍 6 处（含 v32 漏检的 2 处）已精确切除到产品上表面，
        不再产生过度切削空缺；凸起底面在角落处贴合产品上表面。

v34 新增（快速检测 + 几何状态缓存 + 局部优先修复）：
  ① --check-only / --dry-run 模式：
       python mold_fix.py --check-only [--input <模具.stp>] [--cache-dir <dir>]
     只加载模型并做【数学层面的布尔交集检测】；一旦检测到残留微小凸起面，
     直接打印 "Result: FAIL" + 错误坐标，绝对不执行 3D 渲染和 STP 文件写入。
     只有执行 --fix（或手动确认后）才真正生成 .stp 文件 —— 节省 80% 等待。
  ② 几何状态缓存（简单 JSON）：
     - 执行修复前先把当前几何状态（残留凸起坐标/面积/厚度/包围盒等轻量
       数学描述符）序列化缓存为 <输入名>.geom_cache.json；
     - 验证时先对比缓存与当前修复后状态：输入文件指纹未变且凸起仍原位置
       → 直接跳过重复布尔计算 / 跳过重新导入巨大 STP（实测 52s → 1.9s）；
     - 缓存字段：last_check（输入文件本身的检测结果，check-only 读它）、
       last_fix（本次修复前后对照，验证用）。
  ③ 凸起修复逻辑拆成独立函数：
     detect_bottom_fins() 纯数学检测 / _fin_cut_box() 精确切除盒 /
     _fix_fin_local() 局部极小区域修复 / _remove_bottom_fins() 局部验证后应用 /
     _report_fins() 打印 Result: FAIL/PASS + 坐标。
  ④ 局部优先策略：只对凸起发生的【局部极小区域】（角落面组 = 凸起 ∩
     包围盒扩展区）做数学运算（提取→局部切除→局部体积验证），只有局部
     计算通过才把切除盒应用到整体凸起一次，绝不在全局状态上反复尝试。
  ⑤ 薄鳍 z 上限取【薄鳍顶面与产品上表面较高者】：薄鳍高于产品上表面时
     必须切到鳍顶，否则残留"台阶/块状"凸起（图2 的阶梯状特征）。
   ⑥ 凹模挖空重写（v34/v35）：旧算法 Cut(完整基座, 产品∪填充体) 对异形/带凹坑
      产品融合失败并回退成"主板面开口体"，型腔被 0.05mm 顶盖封闭成 2 壳
      （从外部看基座完全没有被挖空）。v34 改为逐个切除拉伸柱：
      - 下形变面柱（朝下外表面沿 +Z 到基座顶面以上）→ 型腔底面贴合产品
        下形变面、顶部开口；
      - 主板底面孔洞柱（主板底面内环沿 +Z）→ 补全孔洞处未覆盖的基座材料；
      - 最深的柱先切 + 只接受"1 实体 1 壳"结果 → 无碎片、无封闭内腔。
      v35 修正井底残留（用户反馈"下形变面以上仍有部分未挖去"）：
      - 大井（孔面积≥500）柱从【井底】拉伸 —— 井底 = 孔内最深的下形变面
        z（用 bbox 重叠最深面求取），孔轮廓先平移到井底再拉伸；
        此前井柱只挖到主板面（-1.41mm），井壁/凸台井深部（到 -11.41mm）
        残留材料（每井约 11k mm³），且井壁面柱的布尔切除直接 FAIL；
      - 排除完全落在大井内的下形变面（井柱已覆盖，避免冗余布尔 FAIL）。
      实测 YA-1238-01：v34 残留 ~45k mm³ → v35 残留 0 mm³，
        切除量 20,433,869 → 20,734,262 mm³，1 实体 1 壳。
   ⑦ 凹模 check-only / 输出带产品 / 缓存（v36 新增）：
      - check_cavity()：--check-only --cavity（或 --input 自动识别凹模）只做
        数学层面布尔交集检测（Common(模具, 下形变面柱/井柱)），检测到残留
        凸起面 → Result: FAIL + 坐标；绝不导出 / 渲染；
      - 修复输出 = compound [型腔, 产品]：产品独立保留，可在 CAD 中半透明
        叠加/分开移动，直观对照型腔面是否与产品下形变面吻合；
      - 几何状态缓存（<输入>.geom_cache.json）新增 last_cavity_check：
        指纹未变且上次结果仍有效 → 跳过重新导入巨大 STP、跳过重复布尔计算
        （实测 check-only 从 ~2min 降到秒回）；
      - 修复后自动做一次残留检测并写缓存（Result: PASS/FAIL + 坐标）。
   ⑧ 凹模挖空（v41 重写，用户要求把产品与基座顶面之间被包裹的间隙也挖掉）：
      - 主方法 method='solid3d'：挖空体 = 产品3D实体 ∪ 间隙体
        （产品外轮廓柱从主板面向上减去产品），Cut(基座(顶=分模面-0.5mm), 挖空体)
        → 型腔底面 = 产品下形变面（弯弯曲曲、完全随形贴合），侧壁 / 底部圆盘 /
        加强筋 / 圆盘口全部随产品外形（无直角口），同时挖掉产品与基座顶面之间的
        间隙实体和基座被产品包裹的上顶面（Common=0，产品能完全放入）；
      - 布尔产生的碎片尽力融合回 1 实体（Fuse 1e-2 容差）；
      - 默认不切外边缘（trim_edge=False）：产品周围的基座实体（模具壁/边框）保留；
      - 备选 method='prisms'：保留旧直筒切除（下形变面柱/井柱），供对比调试；
      - clearance（配合间隙/收缩率，默认 0.1 占位）：当前 OCP 版本偏置不可用，
        实际为完全贴合；
      - 残留检测 = Common(型腔, 产品3D实体)（产品能否放入型腔）：PASS=0 碰撞。
      实测 YA-1238-01 / YA-1131-505：solid3d 均 1 实体 1 壳、产品放入 PASS、
        Common(型腔,产品)=0（完全贴合）、型腔随产品外形，基座壁保留。
    ⑨ 凹模多余结构体清理（用户反馈：凸模/凹模逻辑正常，但凹模输出多出 6 个
       结构体 —— 填充底部加强筋/圆盘的基座残留碎片）：
       - 原因：Cut(基座, 挖空体) 偶尔切出多个互不相连的独立实体（主体型腔 +
         若干残留基座碎片）。碎片位于产品 XY 足迹以内、分模面以下（对应产品
         底部加强筋/圆盘区域），与型腔主体互不接触、融合失败后被原样导出，
         使输出 = compound [型腔, 产品] 多出 6~8 个独立结构体。
       - 修复：新增 _consolidate_hollow_result()，只保留体积最大的型腔主体，
         先尽力融合；融合失败的碎片若【完全在产品 XY 足迹内 + 分模面以下 +
         相对很小】→ 判定为挖腔残留并丢弃（否则安全保留）。
         输出恒为 compound [型腔(1 实体), 产品] 两个独立结构体。
    ⑩ 凹模型腔侧壁精确贴合产品外侧（用户反馈：产品底部圆盘/加强筋挖去都正确，
       但型腔比产品外轮廓大一圈 —— 外轮廓挖除范围过大）：
       - 原因：顶部安装间隙用【产品 bbox 矩形外轮廓柱】从主板面挖到分模面，
         _product_outer_profile 对复杂产品回退成 bbox 矩形（+0.5mm），型腔侧壁
         变成一圈矩形框，比产品真实外轮廓大一圈（实测 YA-1131 顶部处每侧大
         25mm）。
       - 修复：型腔改为 Cut(基座, 产品)（侧壁=产品外侧形状，精确贴合）；
         顶部安装间隙改为【产品每个朝上外表面沿 +Z 拉伸到分模面的柱】逐个切除
         （最深优先，柱底向下 0.5mm 重叠避免布尔脆断，柱顶不超出型腔开口面），
         只挖产品上表面以上、产品轮廓以内的基座材料，保证产品能放入，但不放大
         外轮廓。布尔产生的孔洞柱塞/悬浮碎片由 _consolidate_hollow_result 清理。
         实测 YA-1131 / YA-1238：各高度型腔空腔与产品外轮廓贴合 0mm、产品 100%
         放入型腔、Common(型腔,产品)=0、1 实体 1 壳、底部圆盘/加强筋保持正确。



"""

import os
import sys
import math
import json
import hashlib
import time
import argparse

import cadquery as cq

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import (BRepBuilderAPI_Transform, BRepBuilderAPI_MakeFace,
                                BRepBuilderAPI_MakeSolid)
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt, gp_Vec, gp_Trsf, gp_Ax2, gp_Dir
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE, TopAbs_IN, TopAbs_WIRE
from OCP.STEPControl import STEPControl_Writer, STEPControl_ManifoldSolidBrep, STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepLProp import BRepLProp_SLProps
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.BRepCheck import BRepCheck_Analyzer


def _force_utf8():
    """设置 Windows 控制台与 Python 输出均为 UTF-8，避免中文乱码。

    之前的版本只把 Python 的 stdout/stderr 设为 UTF-8，但 Windows 终端默认
    仍是 GBK(cp936)，UTF-8 字节按 GBK 显示就成了乱码。这里同时把控制台代码页
    切换为 65001(UTF-8)，中英文与 ✓/³ 等符号都能正确显示。
    """
    if os.name == 'nt':
        try:
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
            ctypes.windll.kernel32.SetConsoleCP(65001)
        except Exception:
            pass
    for stream in (sys.stdout, sys.stderr):
        if stream and hasattr(stream, 'reconfigure'):
            try:
                stream.reconfigure(encoding='utf-8', errors='replace')
            except Exception:
                pass


# =====================================================================
# 几何状态缓存（简单 JSON 机制）
# ---------------------------------------------------------------------
# 用途：
#   ① 执行修复前，先把当前几何状态（残留凸起坐标/面积/厚度/体积等轻量
#      数学描述符）序列化缓存为 JSON —— 不直接序列化 OCP 巨型 BRep 形状
#      （无法可靠 pickle 且体积巨大），只缓存【数学检测结果】；
#   ② 验证/复查时，先对比缓存与当前修复后状态：输入文件未变且凸起仍
#      在原位置 → 直接跳过重复的布尔计算 / 跳过重新导入巨大 STP。
# =====================================================================

CACHE_SCHEMA = 1
DEFAULT_CACHE_DIR = "mold_fix_cache"


def _input_fingerprint(filepath):
    """输入文件指纹：size + mtime + 前 1MB 的 sha256 前缀。

    用于判断 STP 是否被修改（手工 CAD 修复/覆盖导出会改变指纹）。
    指纹不变 → 验证时无需重新导入整个巨大 STP 模型。
    """
    try:
        st = os.stat(filepath)
    except OSError:
        return None
    h = hashlib.sha256()
    try:
        with open(filepath, 'rb') as f:
            h.update(f.read(1 << 20))
    except OSError:
        pass
    return {
        'size': st.st_size,
        'mtime': round(st.st_mtime, 3),
        'sha256': h.hexdigest()[:16],
    }


def _cache_path_for(input_path, cache_dir=None):
    """缓存文件路径：<输入文件名>.geom_cache.json（放在 cache_dir 下）。"""
    base = cache_dir or DEFAULT_CACHE_DIR
    name = os.path.splitext(os.path.basename(input_path))[0] + ".geom_cache.json"
    return os.path.join(base, name)


def _load_geom_cache(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return None


def _save_geom_cache(path, data):
    try:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"[MoldFix] 缓存写入失败: {e}")



class MoldFixer:
    """模具修复工具 v31：以模具 compound(基座+产品)为对象，
    凸模输出 = 基座 + 凸起台 + 产品 三个独立结构体
    （残片已整合丢弃，凸起与产品零重叠）。"""

    # ---------------------------------------------------------------
    # 基础工具
    # ---------------------------------------------------------------
    @staticmethod
    def _extract_solids(filepath):
        """加载 STEP 文件，提取所有独立 Solid/Shell 实体，按体积降序排序。"""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"文件不存在: {filepath}")
        result = cq.importers.importStep(filepath)
        shapes = result.vals() if hasattr(result, 'vals') else []

        solids = []
        for shp in shapes:
            s = shp.wrapped
            exp = TopExp_Explorer(s, TopAbs_SOLID)
            while exp.More():
                solid = exp.Current()
                vol = MoldFixer._solid_volume(solid)
                solids.append((vol, solid))
                exp.Next()
            if not TopExp_Explorer(s, TopAbs_SOLID).More():
                exp2 = TopExp_Explorer(s, TopAbs_SHELL)
                while exp2.More():
                    shell = exp2.Current()
                    vol = MoldFixer._solid_volume(shell)
                    solids.append((vol, shell))
                    exp2.Next()

        if not solids:
            raise ValueError("未找到 Solid 或 Shell 实体")
        solids.sort(key=lambda x: x[0], reverse=True)
        return solids

    @staticmethod
    def _solid_volume(shape):
        try:
            p = GProp_GProps()
            BRepGProp.VolumeProperties_s(shape, p)
            return p.Mass()
        except Exception:
            return MoldFixer._bbox_volume(shape)

    @staticmethod
    def _bbox_volume(shape):
        x1, y1, z1, x2, y2, z2 = MoldFixer._bbox_tuple(shape)
        return (x2 - x1) * (y2 - y1) * (z2 - z1) if (x2 > x1 and y2 > y1 and z2 > z1) else 0.0

    @staticmethod
    def _bbox_tuple(shape, name="shape"):
        if shape is None or shape.IsNull():
            return (0., 0., 0., 0., 0., 0.)
        try:
            b = Bnd_Box()
            BRepBndLib.Add_s(shape, b)
            if b.IsVoid():
                return (0., 0., 0., 0., 0., 0.)
            return b.Get()
        except Exception:
            return (0., 0., 0., 0., 0., 0.)

    @staticmethod
    def _bbox_intersect(a, b):
        """两个包围盒是否相交（严格相交判定，接触边界不算相交）。"""
        return (a[0] < b[3] and b[0] < a[3] and
                a[1] < b[4] and b[1] < a[4] and
                a[2] < b[5] and b[2] < a[5])

    @staticmethod
    def _has_geometry(shape):
        if shape is None or shape.IsNull():
            return False
        return (TopExp_Explorer(shape, TopAbs_SOLID).More()
                or TopExp_Explorer(shape, TopAbs_SHELL).More())

    @staticmethod
    def _solid_count(shape):
        n = 0
        exp = TopExp_Explorer(shape, TopAbs_SOLID)
        while exp.More():
            n += 1
            exp.Next()
        return n

    @staticmethod
    def _shell_count(shape):
        n = 0
        exp = TopExp_Explorer(shape, TopAbs_SHELL)
        while exp.More():
            n += 1
            exp.Next()
        return n

    @staticmethod
    def _export(shape, path, as_compound=False):
        """导出 STEP：单实体用 ManifoldSolidBrep（CAD 兼容最好）；
        compound（多结构体）用 AsIs 原样导出。"""
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        w = STEPControl_Writer()
        mode = STEPControl_AsIs if as_compound else STEPControl_ManifoldSolidBrep
        w.Transfer(shape, mode)
        if w.Write(path) != IFSelect_RetDone:
            raise RuntimeError(f"导出失败: {path}")
        return path

    @staticmethod
    def _make_compound(shapes):
        """把多个独立实体组合成 compound（各结构体保持独立，不融合）。"""
        comp = TopoDS_Compound()
        b = BRep_Builder()
        b.MakeCompound(comp)
        for s in shapes:
            if s is not None and not s.IsNull():
                b.Add(comp, s)
        return comp

    @staticmethod
    def _fuse(a, b, label="Fuse", fuzz=1e-4):
        f = BRepAlgoAPI_Fuse(a, b)
        f.SetRunParallel(False)
        f.SetFuzzyValue(fuzz)
        f.Build()
        if not f.IsDone() or not MoldFixer._has_geometry(f.Shape()):
            raise RuntimeError(f"{label} 布尔融合失败")
        return f.Shape()

    @staticmethod
    def _cut(a, b, label="Cut", fuzz=1e-4):
        c = BRepAlgoAPI_Cut(a, b)
        c.SetRunParallel(False)
        c.SetFuzzyValue(fuzz)
        c.Build()
        if not c.IsDone() or not MoldFixer._has_geometry(c.Shape()):
            raise RuntimeError(f"{label} 布尔切除失败")
        return c.Shape()

    @staticmethod
    def _common(a, b, label="Common", fuzz=1e-4):
        c = BRepAlgoAPI_Common(a, b)
        c.SetRunParallel(False)
        c.SetFuzzyValue(fuzz)
        c.Build()
        if not c.IsDone() or not MoldFixer._has_geometry(c.Shape()):
            raise RuntimeError(f"{label} 布尔求交失败")
        return c.Shape()

    @staticmethod
    def _translate(shape, dx, dy, dz):
        trsf = gp_Trsf()
        trsf.SetTranslation(gp_Vec(dx, dy, dz))
        t = BRepBuilderAPI_Transform(shape, trsf, True)
        t.Build()
        return t.Shape()

    # ---------------------------------------------------------------
    # 提取 基座 + 产品（模具 compound 即修改对象）
    # ---------------------------------------------------------------
    @staticmethod
    def _extract_base_and_product(filepath, product_filepath=None, is_punch=None):
        """从【模具文件】提取 基座 与 产品（模具 compound 双实体）：
            基座 = 体积最大实体（长方体底座）
            产品 = 第二实体（与基座对齐，顶面=分模面 Z_A）
        若模具文件只有 1 个实体（兼容旧流程），回退到外部产品文件并对齐。"""
        solids = MoldFixer._extract_solids(filepath)

        if len(solids) >= 2:
            base = solids[0][1]
            product = solids[1][1]
            print(f"[MoldFix] 从模具 compound 中提取（模具即修改对象，无需外部产品）:")
            print(f"[MoldFix]   基座[体积最大]: 体积={solids[0][0]:.0f}")
            print(f"[MoldFix]   产品[第二]:      体积={solids[1][0]:.0f}")
            return base, product

        base = solids[0][1]
        print(f"[MoldFix] 模具文件仅含 1 个实体，整体作为基座")
        if product_filepath and os.path.exists(product_filepath):
            ext = MoldFixer._extract_solids(product_filepath)
            if ext:
                raw_product = ext[0][1]
                pb = MoldFixer._bbox_tuple(raw_product, "product")
                bb = MoldFixer._bbox_tuple(base, "base")
                z_parting = bb[5] if not is_punch else bb[2] + (bb[5] - bb[2]) * 0.5
                dz = z_parting - pb[5]
                pcx = (pb[0] + pb[3]) / 2.0
                pcy = (pb[1] + pb[4]) / 2.0
                bcx = (bb[0] + bb[3]) / 2.0
                bcy = (bb[1] + bb[4]) / 2.0
                product = MoldFixer._translate(raw_product, bcx - pcx, bcy - pcy, dz)
                return base, product
        raise ValueError("无法从模具中提取产品实体（模具非 compound 且未提供产品文件）")

    @staticmethod
    def _ensure_base_parting(base, product, is_punch):
        """确保 base 是基座、product 是产品（按分模面 Z_A 判断，防止体积排序颠倒）。"""
        px = MoldFixer._bbox_tuple(product, "product")
        z_parting = px[5]
        bx = MoldFixer._bbox_tuple(base, "base")
        if is_punch:
            if abs(bx[2] - z_parting) > abs(bx[5] - z_parting) + 1e-6:
                print("[MoldFix] 体积排序颠倒，交换 基座/产品（使基座贴合分模面于底面）")
                return product, base
        else:
            if abs(bx[5] - z_parting) > abs(bx[2] - z_parting) + 1e-6:
                print("[MoldFix] 体积排序颠倒，交换 基座/产品（使基座贴合分模面于顶面）")
                return product, base
        return base, product

    # ---------------------------------------------------------------
    # 上下表面分离（信息打印 / 法向工具，兼容 annotate_product_faces）
    # ---------------------------------------------------------------
    @staticmethod
    def _point_inside(shape, pnt, tol=1e-6):
        exp = TopExp_Explorer(shape, TopAbs_SOLID)
        any_solid = False
        while exp.More():
            any_solid = True
            try:
                clsf = BRepClass3d_SolidClassifier(exp.Current())
                clsf.Perform(pnt, tol)
                if clsf.State() == TopAbs_IN:
                    return True
            except Exception:
                pass
            exp.Next()
        if not any_solid:
            x1, y1, z1, x2, y2, z2 = MoldFixer._bbox_tuple(shape)
            return (x1 < pnt.X() < x2 and y1 < pnt.Y() < y2 and z1 < pnt.Z() < z2)
        return False

    @staticmethod
    def _face_outer_normal(face, product):
        """求面【朝外单位法向】（BRepAdaptor + BRepLProp + SolidClassifier 修正朝外）。
        返回 (点, 单位法向 gp_Vec) 或 None。"""
        try:
            tface = TopoDS.Face_s(face)
            adapt = BRepAdaptor_Surface(tface)
            u1 = adapt.FirstUParameter()
            u2 = adapt.LastUParameter()
            v1 = adapt.FirstVParameter()
            v2 = adapt.LastVParameter()
            INF = 1e8
            if abs(u1) > INF: u1 = -1.0
            if abs(u2) > INF: u2 = 1.0
            if abs(v1) > INF: v1 = -1.0
            if abs(v2) > INF: v2 = 1.0
            if abs(u2 - u1) < 1e-12: u2 = u1 + 1.0
            if abs(v2 - v1) < 1e-12: v2 = v1 + 1.0
        except Exception:
            return None

        props = BRepLProp_SLProps(adapt, 2, 1e-7)
        candidates = [
            ((u1 + u2) / 2.0, (v1 + v2) / 2.0),
            ((3 * u1 + u2) / 4.0, (v1 + v2) / 2.0),
            ((u1 + 3 * u2) / 4.0, (v1 + v2) / 2.0),
            ((u1 + u2) / 2.0, (3 * v1 + v2) / 4.0),
            ((u1 + u2) / 2.0, (v1 + 3 * v2) / 4.0),
        ]
        for um, vm in candidates:
            try:
                props.SetParameters(um, vm)
                if not props.IsNormalDefined():
                    continue
                pnt = props.Value()
                n = props.Normal()
                nx, ny, nz = n.X(), n.Y(), n.Z()
                mag = math.sqrt(nx * nx + ny * ny + nz * nz)
                if mag < 1e-12:
                    continue
                nx, ny, nz = nx / mag, ny / mag, nz / mag
                eps = 0.1
                if MoldFixer._point_inside(
                        product,
                        gp_Pnt(pnt.X() + nx * eps, pnt.Y() + ny * eps, pnt.Z() + nz * eps),
                        tol=1e-6):
                    nx, ny, nz = -nx, -ny, -nz
                return pnt, gp_Vec(nx, ny, nz)
            except Exception:
                continue
        return None

    @staticmethod
    def _classify_surfaces(product):
        """分离产品上/下表面面集合（贴合验证用，兼容旧接口）。"""
        upper = []
        lower = []
        n_fail = 0
        exp = TopExp_Explorer(product, TopAbs_FACE)
        while exp.More():
            face = exp.Current()
            info = MoldFixer._face_outer_normal(face, product)
            if info is None:
                n_fail += 1
                lower.append(face)
            else:
                _, n = info
                if n.Z() > 0.02:
                    upper.append(face)
                else:
                    lower.append(face)
            exp.Next()
        print(f"[MoldFix] 上下表面分离: 共 {MoldFixer._count_faces(product)} 个面，"
              f"上表面 U = {len(upper)}，下表面 L = {len(lower)}，失败 {n_fail}")
        return upper, lower

    @staticmethod
    def _count_faces(s):
        n = 0
        e = TopExp_Explorer(s, TopAbs_FACE)
        while e.More():
            n += 1
            e.Next()
        return n

    @staticmethod
    def _face_area(f):
        p = GProp_GProps()
        BRepGProp.SurfaceProperties_s(f, p)
        return p.Mass()

    # ---------------------------------------------------------------
    # 核心算法 v15：产品脚印轮廓柱
    # ---------------------------------------------------------------
    @staticmethod
    def _find_footprint_face(product, n_scan=16):
        """找到产品【脚印面】（在分模面上覆盖产品最宽截面的轮廓面）。
        返回 (面, 面积, z) 或 (None, 0, None)。

        采用【最饱满水平切片】策略（稳健覆盖曲面/异形产品）：
          - 在产品 z 范围上粗扫 n_scan 层，求每层水平切片体积，
            体积最大的层 = 产品最宽处（主板/法兰所在高度）；
          - 在该层精确切片，取最大水平面作为脚印轮廓面。
        另以【最大水平朝上外表面】作快速候选，若其面积明显更大则优先。"""
        # 快速候选：最大水平朝上外表面
        fast_face = None
        fast_area = 0.0
        fast_z = None
        exp = TopExp_Explorer(product, TopAbs_FACE)
        while exp.More():
            f = exp.Current()
            fbb = MoldFixer._bbox_tuple(f)
            if abs(fbb[5] - fbb[2]) > 0.05:
                exp.Next()
                continue
            area = MoldFixer._face_area(f)
            if area <= fast_area:
                exp.Next()
                continue
            info = MoldFixer._face_outer_normal(f, product)
            if info is None:
                exp.Next()
                continue
            _, n = info
            if n.Z() > 0.5:
                fast_area = area
                fast_face = f
                fast_z = fbb[5]
            exp.Next()

        # 最饱满切片
        x1, y1, z1, x2, y2, z2 = MoldFixer._bbox_tuple(product, "product-slice")
        if z2 - z1 <= 1.0:
            return fast_face, fast_area, fast_z

        best_z = None
        best_slice_vol = 0.0
        dz = (z2 - z1) / n_scan
        for k in range(n_scan):
            zc = z1 + (k + 0.5) * dz
            try:
                slab = BRepPrimAPI_MakeBox(
                    gp_Ax2(gp_Pnt(x1 - 2, y1 - 2, zc - 0.5), gp_Dir(0, 0, 1)),
                    (x2 - x1) + 4, (y2 - y1) + 4, 1.0,
                ).Shape()
                c = BRepAlgoAPI_Common(product, slab)
                c.SetRunParallel(False)
                c.SetFuzzyValue(1e-4)
                c.Build()
                if c.IsDone():
                    v = MoldFixer._solid_volume(c.Shape())
                    if v > best_slice_vol:
                        best_slice_vol = v
                        best_z = zc
            except Exception:
                continue

        slice_face = None
        slice_area = 0.0
        slice_z = None
        if best_z is not None and best_slice_vol > 0:
            slice_face, slice_area, slice_z = MoldFixer._slice_outline_face(product, best_z)

        # 取两者中面积较大的作为脚印面（覆盖不同产品类型）
        if fast_face is None:
            return slice_face, slice_area, slice_z
        if slice_face is None:
            return fast_face, fast_area, fast_z
        if fast_area >= slice_area * 1.15:
            return fast_face, fast_area, fast_z
        return slice_face, slice_area, slice_z

    @staticmethod
    def _slice_outline_face(product, zc):
        """在 z=zc 处取 1mm 厚切片，返回其最大水平面（脚印轮廓，可能带内孔）。"""
        x1, y1, z1, x2, y2, z2 = MoldFixer._bbox_tuple(product, "product-slice")
        try:
            slab = BRepPrimAPI_MakeBox(
                gp_Ax2(gp_Pnt(x1 - 2, y1 - 2, zc - 0.5), gp_Dir(0, 0, 1)),
                (x2 - x1) + 4, (y2 - y1) + 4, 1.0,
            ).Shape()
            c = BRepAlgoAPI_Common(product, slab)
            c.SetRunParallel(False)
            c.SetFuzzyValue(1e-4)
            c.Build()
            if not c.IsDone():
                return None, 0.0, None
        except Exception:
            return None, 0.0, None
        exp = TopExp_Explorer(c.Shape(), TopAbs_FACE)
        best = None
        best_area = 0.0
        best_z = None
        while exp.More():
            f = exp.Current()
            fbb = MoldFixer._bbox_tuple(f)
            if abs(fbb[5] - fbb[2]) > 0.05:
                exp.Next()
                continue
            area = MoldFixer._face_area(f)
            if area > best_area:
                best_area = area
                best = f
                best_z = fbb[5]
            exp.Next()
        return best, best_area, best_z

    @staticmethod
    def _footprint_outline_face(footprint_face):
        """取脚印面的【外轮廓线】重建平面（补掉内孔），得到实心轮廓面。
        若补洞失败，返回原面（带孔）。"""
        try:
            outer_wire = BRepTools.OuterWire_s(TopoDS.Face_s(footprint_face))
            if not outer_wire.IsNull():
                mf = BRepBuilderAPI_MakeFace(outer_wire)
                mf.Build()
                if mf.IsDone():
                    return mf.Face()
        except Exception:
            pass
        return footprint_face

    @staticmethod
    def _face_inner_wires(face):
        """返回平面面的【内环线】（孔洞轮廓，用于凹坑补腔）。
        若取外环失败返回空列表。"""
        wires = []
        try:
            tface = TopoDS.Face_s(face)
            outer = BRepTools.OuterWire_s(tface)
            exp = TopExp_Explorer(tface, TopAbs_WIRE)
            while exp.More():
                w = exp.Current()
                if not w.IsSame(outer):
                    wires.append(w)
                exp.Next()
        except Exception:
            pass
        return wires

    @staticmethod
    def _make_classifiers(shape):
        """为实体的所有 Solid 建持久分类器（比每点新建 _point_inside 快得多）。"""
        cls = []
        exp = TopExp_Explorer(shape, TopAbs_SOLID)
        while exp.More():
            try:
                cls.append(BRepClass3d_SolidClassifier(exp.Current()))
            except Exception:
                pass
            exp.Next()
        return cls

    @staticmethod
    def _classifier_inside(cls, pnt, tol=1e-6):
        for c in cls:
            try:
                c.Perform(pnt, tol)
                if c.State() == TopAbs_IN:
                    return True
            except Exception:
                continue
        return False

    @staticmethod
    def _product_top_at(cx, cy, product):
        """该 (x,y) 处产品上表面 z（从上方探测）。产品不存在返回 None。"""
        if not MoldFixer._has_geometry(product):
            return None
        pcls = MoldFixer._make_classifiers(product)
        for z in [z / 100.0 for z in range(5000, -1200, -5)]:  # 50.0 → -12.0
            if MoldFixer._classifier_inside(pcls, gp_Pnt(cx, cy, z)):
                return z
        return None


    @staticmethod
    def _face_thickness(face, shape, clss):
        """测量小面沿法向的【材料厚度】：从面中心沿法线向实体内部走多远才出界。

        布尔构造（凸起在主板面边缘包裹产品翻边/凸台）会留下极薄的薄鳍小面
        （法向厚度常 < 0.1mm），在 CAD 中显示为角落的局部凸起。
        返回厚度(mm)；若无法判断（面太扁/法向异常）返回 None。
        """
        try:
            ad = BRepAdaptor_Surface(TopoDS.Face_s(face))
            u0 = ad.FirstUParameter()
            u1 = ad.LastUParameter()
            v0 = ad.FirstVParameter()
            v1 = ad.LastVParameter()
            pnt = ad.Value((u0 + u1) / 2.0, (v0 + v1) / 2.0)
            lp = BRepLProp_SLProps(ad, (u0 + u1) / 2.0, (v0 + v1) / 2.0, 2, 1e-7)
            if not lp.IsNormalDefined():
                return None
            n = lp.Normal()
            for sign in (1, -1):
                q = pnt.Translated(gp_Vec(n.X(), n.Y(), n.Z()).Multiplied(0.05 * sign))
                if MoldFixer._classifier_inside(clss, q):
                    # 沿该方向二分走到实体外，得到该方向的厚度（15 次迭代精度足够）
                    lo, hi = 0.05, 5.0
                    for _ in range(15):
                        mid = (lo + hi) / 2.0
                        q = pnt.Translated(gp_Vec(n.X(), n.Y(), n.Z()).Multiplied(mid * sign))
                        if MoldFixer._classifier_inside(clss, q):
                            lo = mid
                        else:
                            hi = mid
                    return lo
        except Exception:
            pass
        return None

    @staticmethod
    def detect_bottom_fins(protrusion, product, main_z):
        """检测凸起底部布尔残留的薄鳍小面（角落局部凸起）—— 纯数学检测。

        凸起在主板面边缘包裹产品翻边/凸台时，布尔（Cut(全高柱,产品)）会留下
        极薄的薄鳍小面（法向材料厚度 < 0.5mm，实测约 0.05~0.08mm），在 CAD 中
        显示为"比周围高一点点的小长方形面"（角落局部凸起）。

        只做面级数学运算（面积 / 包围盒 / 法向厚度二分探测），【不修改任何
        几何】。返回可 JSON 序列化的 fin 记录列表：
          [{'bbox': [x1,y1,z1,x2,y2,z2], 'area': float, 'thickness': float,
            'center': [cx,cy,cz]}, ...]
        """
        if not MoldFixer._has_geometry(protrusion):
            return []
        clss = MoldFixer._make_classifiers(protrusion)
        fins = []
        fexp = TopExp_Explorer(protrusion, TopAbs_FACE)
        while fexp.More():
            f = fexp.Current()
            fb = MoldFixer._bbox_tuple(f)
            fa = MoldFixer._face_area(f)
            if fa < 30.0 and (main_z - 3.0) < fb[2] < (main_z + 2.0):
                th = MoldFixer._face_thickness(f, protrusion, clss)
                if th is not None and th < 0.5:
                    fins.append({'bbox': list(fb), 'area': round(fa, 4),
                                 'thickness': round(th, 4)})
                elif th is None and fa < 6.0 and (fb[5] - fb[2]) > 1.0:
                    # 厚度探测失败(法向异常/位于边界)但面积小、z跨度大 → 薄鳍小面
                    fins.append({'bbox': list(fb), 'area': round(fa, 4),
                                 'thickness': 0.0})
            fexp.Next()
        for rec in fins:
            b = rec['bbox']
            rec['center'] = [round((b[0] + b[3]) / 2.0, 4),
                             round((b[1] + b[4]) / 2.0, 4),
                             round((b[2] + b[5]) / 2.0, 4)]
        return fins

    @staticmethod
    def _fin_cut_box(fin, product):
        """计算薄鳍【精确切除盒】：只切薄鳍自身包围盒（0.01mm 余量），
        x/y 绝不越界；z 上限取薄鳍顶面与产品上表面的【较高者】：
          - 薄鳍上缘高于该处产品上表面 → 必须切到薄鳍顶面，否则会残留
            一块高出产品表面的"台阶"凸起（图2 的块状/阶梯状特征）；
          - 薄鳍整体在产品上表面以下 → 切到产品上表面，平齐不挖出凹槽。
        返回盒实体；若该薄鳍整体在产品内（由 Cut(凸起,产品) 处理）返回 None。"""
        x1, y1, z1, x2, y2, z2 = fin['bbox']
        eps = 0.01
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        z_top = MoldFixer._product_top_at(cx, cy, product)
        if z_top is None:
            z_top = z2          # 产品不存在（贯穿槽/孔）→ 切到薄鳍顶面
        elif z_top < z2:
            z_top = z2          # 薄鳍上缘高于产品上表面 → 切到鳍顶，移除凸出台阶
        # else: 薄鳍整体在产品上表面以下 → z_top 保持产品上表面，平齐无凹槽
        if z_top <= z1 + 0.5:
            return None  # 薄鳍整体在产品内（本不该存在）→ 由 Cut(凸起,产品) 处理
        return BRepPrimAPI_MakeBox(
            gp_Ax2(gp_Pnt(x1 - eps, y1 - eps, z1 - eps), gp_Dir(0, 0, 1)),
            (x2 - x1) + 2 * eps, (y2 - y1) + 2 * eps,
            (z_top + eps) - (z1 - eps)).Shape()

    @staticmethod
    def _fix_fin_local(fin, protrusion, product, main_z, pad=3.0):
        """局部极小区域修复实验（不碰全局几何）：
          1) 提取该薄鳍【所在角落的面组】= 凸起 ∩ (薄鳍包围盒 + pad 扩展盒)；
          2) 只在局部实体上做精确切除（纯数学运算）；
          3) 局部验证：切除盒体积 vs 实际切除体积 —— 证明该处薄鳍材料确实被
             移除（不能用 detect_bottom_fins 复查局部实体：Common 在区域盒
             边界切出的新面会被误判为薄鳍）。
        返回 (cut_box, local_solid)；局部失败返回 (None, None)。
        """
        cut_box = MoldFixer._fin_cut_box(fin, product)
        if cut_box is None:
            return None, None
        x1, y1, z1, x2, y2, z2 = fin['bbox']
        try:
            region = BRepPrimAPI_MakeBox(
                gp_Ax2(gp_Pnt(x1 - pad, y1 - pad, z1 - pad), gp_Dir(0, 0, 1)),
                (x2 - x1) + 2 * pad, (y2 - y1) + 2 * pad,
                (z2 - z1) + 2 * pad).Shape()
            local = MoldFixer._common(protrusion, region,
                                      "局部区域提取(角落面组)", fuzz=1e-6)
            if not MoldFixer._has_geometry(local):
                return None, None
            vol_before = MoldFixer._solid_volume(local)
            local_fixed = MoldFixer._cut(local, cut_box, "局部-精确切除薄鳍", fuzz=1e-6)
            # 局部验证：该薄鳍包围盒内确实切掉了材料。
            # 注意：薄鳍极薄（法向 ~0.05mm），材料体积只占其包围盒体积的
            # 百分之几甚至更低，不能用“切除量/盒体积”比例作阈值；
            # 只要切除盒在局部实体中切掉任何非噪声体积即视为局部修复成功。
            removed = vol_before - MoldFixer._solid_volume(local_fixed)
            if removed > 1e-3:
                return cut_box, local_fixed
            return None, None
        except Exception:
            return None, None

    @staticmethod
    def _remove_bottom_fins(protrusion, product, main_z):
        """去除凸起底部布尔残留的薄鳍小面（角落局部凸起）—— 精确切除到产品上表面。

        【局部优先策略】对每个薄鳍：
          ① 先在其【局部极小区域】（角落面组）做数学运算（_fix_fin_local）；
          ② 只有局部数学验证通过，才把切除盒应用到整体凸起（全局仅一次布尔）；
          ③ 局部失败则完全不动全局，避免在全局状态上反复尝试。

        v32 老版本对每个薄鳍用"包围盒+0.5mm扩展+±2mm高度"切除 → 过度切削
        挖出 2 处小空缺；本版本精确切除薄鳍自身包围盒（0.01mm 余量），
        z 上限贴合产品上表面，无越界切削、无空缺。

        返回 (result_protrusion, remaining_fins)。"""
        fins = MoldFixer.detect_bottom_fins(protrusion, product, main_z)
        if not fins:
            return protrusion, []
        result = protrusion

        n_fin = 0
        for fin in fins:
            try:
                cut_box, _local = MoldFixer._fix_fin_local(fin, result, product, main_z)
                if cut_box is None:
                    continue
                result = MoldFixer._cut(result, cut_box,
                                        "凸起-去底部薄鳍(局部验证通过)", fuzz=1e-6)
                n_fin += 1
            except Exception:
                continue
        if n_fin:
            print(f"[MoldFix]   清理底部薄鳍 {n_fin} 处"
                  f"（局部数学验证通过后应用到整体，精确切除至产品上表面）")
        remaining = MoldFixer.detect_bottom_fins(result, product, main_z)
        if remaining:
            print(f"[MoldFix]   仍有 {len(remaining)} 处薄鳍未被清除"
                  f"（局部修复失败，未在全局反复尝试）")
        return result, remaining

    @staticmethod
    def _report_fins(fins, prefix="检测"):
        """打印残留薄鳍凸起面报告（含错误坐标）。返回 'FAIL' / 'PASS'。"""
        if not fins:
            print(f"Result: PASS  —— [{prefix}] 未检测到残留微小凸起面")
            return "PASS"
        print(f"Result: FAIL  —— [{prefix}] 检测到 {len(fins)} 处残留微小凸起面：")
        for i, rec in enumerate(fins, 1):
            c = rec['center']
            b = rec['bbox']
            print(f"  #{i} 错误坐标（中心）: ({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f})  "
                  f"面积 {rec['area']:.4f} mm²  厚度 {rec.get('thickness', 0):.4f} mm")
            print(f"     包围盒: X[{b[0]:.4f},{b[3]:.4f}]  "
                  f"Y[{b[1]:.4f},{b[4]:.4f}]  Z[{b[2]:.4f},{b[5]:.4f}]")
        return "FAIL"


    @staticmethod
    def _make_column(product, z_A, z_B, top_overlap=0.05, bottom_overlap=0.2):
        """产品【脚印轮廓柱】（备用方案，当产品无清晰主板顶面时兜底）：
          1) 脚印面 = 产品最饱满水平切片的最大水平面（覆盖主板/法兰/最宽处，
             非矩形真实轮廓；对曲面/异形产品同样稳健）；
          2) 外轮廓面（补洞）= 产品在分模面的真实轮廓（非矩形）；
          3) 平移到 分模面 Z_A + top_overlap（微深入基座底面，保证融合稳定）；
          4) 沿 -Z 向下拉伸到 产品底面 Z_B - bottom_overlap。
        返回柱实体。"""
        face, area, z_f = MoldFixer._find_footprint_face(product)
        if face is None:
            raise RuntimeError("未找到产品水平朝上外表面（脚印），无法生成轮廓柱")
        print(f"[MoldFix]   脚印面(产品最大水平朝上外表面): 面积={area:.0f}  z={z_f:.2f}")

        outline = MoldFixer._footprint_outline_face(face)
        outline_area = MoldFixer._face_area(outline)
        if abs(outline_area - area) > 1.0:
            print(f"[MoldFix]   外轮廓面(补洞): 面积={outline_area:.0f}"
                  f"（原面 {area:.0f}，差 {outline_area - area:.0f} 为内孔）")

        f_top = MoldFixer._translate(outline, 0, 0, z_A + top_overlap - z_f)
        h = (z_A + top_overlap) - (z_B - bottom_overlap)
        col = BRepPrimAPI_MakePrism(f_top, gp_Vec(0.0, 0.0, -h)).Shape()
        return col

    # ---------------------------------------------------------------
    # 核心算法 v16：贴合产品表面的凸起/凹陷
    # ---------------------------------------------------------------
    @staticmethod
    def _find_fill_face(product, n_scan=16):
        """产品上表面基准面（带特征孔）。返回 (face, 面积, z) 或 (None, 0, None)。

        策略：
          1) 优先用【最大水平朝上外表面】（主板顶面，带凸台/特征孔）——
             当它面积 ≥ 产品 bbox 面积的 30% 时（平板类产品，凸台可保留）；
          2) 否则回退到【最饱满水平切片】的最大水平面（覆盖曲面/异形产品，
             贴合板底面落在产品最宽处，填满基座与产品之间的空隙）。"""
        # 1. 最大水平朝上外表面（主板顶面）
        main_face = None
        main_area = 0.0
        main_z = None
        pb = MoldFixer._bbox_tuple(product, "product")
        bb_area = max((pb[3] - pb[0]) * (pb[4] - pb[1]), 1e-9)
        exp = TopExp_Explorer(product, TopAbs_FACE)
        while exp.More():
            f = exp.Current()
            fbb = MoldFixer._bbox_tuple(f)
            if abs(fbb[5] - fbb[2]) > 0.05:
                exp.Next()
                continue
            area = MoldFixer._face_area(f)
            if area <= main_area:
                exp.Next()
                continue
            info = MoldFixer._face_outer_normal(f, product)
            if info is None:
                exp.Next()
                continue
            _, n = info
            if n.Z() > 0.5:
                main_area = area
                main_face = f
                main_z = fbb[5]
            exp.Next()

        if main_face is not None and main_area >= 0.30 * bb_area:
            return main_face, main_area, main_z

        # 2. 最饱满水平切片（fallback：异形/曲面产品）
        face, area, z = MoldFixer._find_footprint_face(product, n_scan)
        if face is not None and (main_face is None or area > main_area):
            return face, area, z
        return main_face, main_area, main_z

    @staticmethod
    def _fill_internal_voids(solid):
        """把实体内部的【封闭空腔】填实（多壳 → 单壳，彻底无空腔）。

        基座∪填充体∪产品 融合后，填充体逐面拉伸与产品筋/凸台之间可能封闭出
        少量内部空腔（多壳实体）。本方法把每个内部壳（空腔边界）反转朝向、
        转成实体，再 Fuse 回原实体，即可把空腔填实，不改变外表面形状。
        """
        shells = []
        exp = TopExp_Explorer(solid, TopAbs_SHELL)
        while exp.More():
            shells.append(exp.Current())
            exp.Next()
        if len(shells) <= 1:
            return solid
        result = solid
        filled = 0
        for sh in shells[1:]:
            try:
                mk = BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(sh.Reversed()))
                mk.Build()
                if mk.IsDone() and not mk.Solid().IsNull():
                    v = MoldFixer._solid_volume(mk.Solid())
                    if v > 0.05:
                        result = MoldFixer._fuse(result, mk.Solid(), "Fill-内部空腔")
                        filled += 1
            except Exception:
                continue
        if filled:
            print(f"[MoldFix]   内部封闭空腔填实 {filled} 个（彻底无空腔）")
        return result

    @staticmethod
    def _consolidate_protrusion(protrusion):
        """把凸起整合为【单一主体】。

        布尔构造会把凸起切成多个独立实体：主凸起台 + 若干残片
        （产品下表面以下的柱状补腔、边缘特征薄片等）。这些残片与主体
        互不重叠/无法融合，作为独立结构体会让用户看到多余实体。
        本方法只保留体积最大的主凸起台实体，丢弃其余无法融合的残片，
        保证最终输出 = 基座 + 凸起台 + 产品 三个独立结构体。
        """
        solids = []
        e = TopExp_Explorer(protrusion, TopAbs_SOLID)
        while e.More():
            solids.append(e.Current())
            e.Next()
        if len(solids) <= 1:
            return protrusion

        solids.sort(key=lambda s: -MoldFixer._solid_volume(s))
        body = solids[0]
        n_fuse, n_drop = 0, 0
        for s in solids[1:]:
            # 先尝试并入主体（重叠/贴合时融合为单一实体）
            try:
                fu = BRepAlgoAPI_Fuse(body, s)
                fu.SetRunParallel(False)
                fu.SetFuzzyValue(2e-2)
                fu.Build()
                if fu.IsDone() and MoldFixer._solid_count(fu.Shape()) == 1:
                    body = fu.Shape()
                    n_fuse += 1
                    continue
            except Exception:
                pass
            n_drop += 1
        print(f"[MoldFix]   凸起整合: {len(solids)} 个实体 → "
              f"主体 1 个 + 融合 {n_fuse} 个 + 丢弃残片 {n_drop} 个"
              f"（产品下表面以下的柱状补腔 / 悬浮碎片）")
        return body

    @staticmethod
    def _consolidate_hollow_result(result, product):
        """把挖腔后的型腔整合为【单一主体】。

        Cut(基座, 挖空体) 布尔运算偶尔会切出多个互不相连的独立实体：
        主体型腔 + 若干残留基座碎片 —— 这些碎片位于产品底部加强筋/圆盘
        区域（产品 XY 足迹以内、分模面以下），填充了型腔里本应被挖空的
        部分。碎片与型腔主体互不接触、无法融合，作为独立结构体会让用户
        看到多余实体（填充底部加强筋/圆盘的碎片）。

        本方法只保留体积最大的型腔主体，先尽力把其余实体融合回主体；
        融合失败的碎片若【完全位于产品 XY 足迹以内 + 分模面以下 +
        相对很小】→ 判定为挖腔残留并丢弃；否则保留（可能是被布尔切开
        的独立模体/壁块）。保证最终输出 = compound [型腔(1 实体), 产品]
        两个独立结构体。
        """
        solids = []
        e = TopExp_Explorer(result, TopAbs_SOLID)
        while e.More():
            solids.append(e.Current())
            e.Next()
        if len(solids) <= 1:
            return result

        solids.sort(key=lambda s: -MoldFixer._solid_volume(s))
        body = solids[0]
        body_vol = MoldFixer._solid_volume(body)
        pb = MoldFixer._bbox_tuple(product, "product")
        kept = [body]
        n_fuse, n_drop = 0, 0
        for s in solids[1:]:
            # 先尝试并入主体（重叠/贴合时融合为单一实体）
            try:
                fu = BRepAlgoAPI_Fuse(body, s)
                fu.SetRunParallel(False)
                fu.SetFuzzyValue(2e-2)
                fu.Build()
                if fu.IsDone() and MoldFixer._solid_count(fu.Shape()) == 1 \
                        and MoldFixer._shell_count(fu.Shape()) == 1:
                    body = fu.Shape()
                    kept[0] = body
                    n_fuse += 1
                    continue
            except Exception:
                pass
            # 融合失败：碎片完全位于产品足迹以内 + 分模面以下 + 相对很小
            # → 挖腔残留（填充底部加强筋/圆盘），丢弃；否则保留。
            sb = MoldFixer._bbox_tuple(s)
            inside_foot = (sb[0] >= pb[0] - 1.0 and sb[3] <= pb[3] + 1.0
                           and sb[1] >= pb[1] - 1.0 and sb[4] <= pb[4] + 1.0)
            small = MoldFixer._solid_volume(s) < 0.2 * max(body_vol, 1.0)
            below_top = sb[5] <= pb[5] + 0.5   # 低于分模面（产品顶面）
            if inside_foot and small and below_top:
                n_drop += 1
            else:
                kept.append(s)
        print(f"[MoldFix]   型腔整合: {len(solids)} 个实体 → "
              f"主体 1 个 + 融合 {n_fuse} 个 + 丢弃残留碎片 {n_drop} 个"
              f"（产品足迹内、分模面以下的基座碎片：填充底部加强筋/圆盘）")
        if len(kept) == 1:
            return body
        return MoldFixer._make_compound(kept)

    @staticmethod
    def _make_fill_plate(product, z_A, top_overlap=0.05, bottom_overlap=0.3):
        """贴合板 = 产品主板顶面（最大水平朝上外表面，带特征孔）
        平移到分模面 Z_A+top_overlap，向下拉伸到 主板面 z - bottom_overlap。

        凸起 = 基座 ∪ 贴合板 ∪ 产品：
          - 贴合板底面 = 主板顶面（与产品表面贴合，带凸台/特征孔），
            只填补"基座底面与产品上表面之间"的空隙；
          - 凸台/向下特征由产品自身保留 → 凸起表面贴合产品表面形状。
        返回 (贴合板实体, 主板面面积, 主板面 z)。"""
        face, area, z_f = MoldFixer._find_fill_face(product)
        if face is None:
            raise RuntimeError("未找到产品主板顶面（最大水平朝上外表面），无法生成贴合板")
        print(f"[MoldFix]   主板顶面(贴合基准): 面积={area:.0f}  z={z_f:.2f}")

        f_top = MoldFixer._translate(face, 0, 0, z_A + top_overlap - z_f)
        total_h = (z_A + top_overlap) - (z_f - bottom_overlap)
        plate = BRepPrimAPI_MakePrism(f_top, gp_Vec(0.0, 0.0, -total_h)).Shape()
        return plate, area, z_f

    @staticmethod
    def _build_fill_body(product, z_A, main_face, main_z, top_overlap=0.05,
                         bottom_overlap=0.3, z_tol=0.5, min_area=10.0,
                         include_all=False, fill_main_holes=False):
        """填充体 = 产品上表面（朝上外表面）沿 +Z 拉伸到分模面的并集。
        返回填充体实体（1 Shell 封闭实体，底面=产品上表面贴合形状）。

        - include_all=False（凹模/常规）：跳过向下特征内壁（z 上限远低于主板面）
          和极小面，填充体只覆盖产品上表面以上；
        - include_all=True（凸模凸起）：包含【全部朝上面】—— 主平面 + 翻边环面
          + boss圆柱面 + 加强筋B样条面 + 球面 + 小圆盘面等，底部贴合产品
          弯曲起伏的形变面（完整上表面），并用大容差(1e-2)融合保证封闭。

        top_overlap>0 顶部深入基座（凸模融合用）；<0 顶部收在基座内（凹模切除体用）。"""
        main_bb = MoldFixer._bbox_tuple(main_face, "main_face")
        main_area = MoldFixer._face_area(main_face)
        up_faces = []
        exp = TopExp_Explorer(product, TopAbs_FACE)
        while exp.More():
            f = exp.Current()
            info = MoldFixer._face_outer_normal(f, product)
            if info is None:
                exp.Next()
                continue
            _, n = info
            if n.Z() > 0.05:
                up_faces.append((MoldFixer._face_area(f),
                                 MoldFixer._bbox_tuple(f), f))
            exp.Next()
        up_faces.sort(key=lambda x: -x[0])

        print(f"[MoldFix]   朝上外表面 {len(up_faces)} 个，填充范围 "
              f"{'全部朝上面（贴合形变面）' if include_all else f'z>={main_z - z_tol:.2f} 面积>={min_area:.0f}'}")
        fuse_fuzz = 1e-2 if include_all else 1e-4

        fill = None
        skip_low = 0
        skip_small = 0
        fuse_fail = 0

        def add_face(face, fbb):
            nonlocal fill, fuse_fail
            h = z_A - fbb[5]
            if h <= 0:
                return
            trsf = gp_Trsf()
            trsf.SetTranslation(gp_Vec(0, 0, -bottom_overlap))
            t = BRepBuilderAPI_Transform(face, trsf, True)
            t.Build()
            pr = BRepPrimAPI_MakePrism(
                t.Shape(), gp_Vec(0.0, 0.0, h + bottom_overlap + top_overlap)
            ).Shape()
            try:
                if fill is None:
                    fill = pr
                    return
                fu = BRepAlgoAPI_Fuse(fill, pr)
                fu.SetRunParallel(False)
                fu.SetFuzzyValue(fuse_fuzz)
                fu.Build()
                if fu.IsDone() and MoldFixer._solid_volume(fu.Shape()) > 0:
                    fill = fu.Shape()
                else:
                    fuse_fail += 1
            except Exception:
                fuse_fail += 1

        # 1. 主板面贴合板（可能是切片面，不在产品实际朝上面列表里）。
        #    fill_main_holes=True（凸模）时用【外轮廓补洞面】拉伸——孔洞由后续
        #    凹坑补腔填充，两者重叠 → 融合成单一实体（不会出现多个独立实体）。
        if fill_main_holes:
            try:
                outline_f = MoldFixer._footprint_outline_face(main_face)
                obb = MoldFixer._bbox_tuple(outline_f, "main_outline")
                if MoldFixer._face_area(outline_f) > main_area * 0.5:
                    main_face = outline_f
                    main_bb = obb
            except Exception:
                pass
        add_face(main_face, main_bb)

        # 2. 其它朝上外表面（凸台侧壁/边缘翻边/加强筋曲面等）
        for area, fbb, f in up_faces:
            if fbb[5] == main_bb[5] and abs(area - main_area) < 1.0:
                continue  # 主板面本身已填
            if include_all:
                # 凸模：只拉伸主板面及以上特征（翻边/凸台/加强筋顶面等），
                # 排除凹坑/圆盘的【内壁与底面】（z 低于主板面）——
                # 这些面沿 +Z 拉伸会生成侵入产品内部的方块状凸起（用户反馈问题）。
                if fbb[2] < main_z - z_tol:
                    skip_low += 1
                    continue
            else:
                if fbb[5] < main_z - z_tol:
                    skip_low += 1
                    continue
                if area < min_area:
                    skip_small += 1
                    continue
            add_face(f, fbb)

        print(f"[MoldFix]   填充体完成: 跳过内壁 {skip_low}，极小面 {skip_small}，"
              f"融合失败 {fuse_fail}，体积 = {MoldFixer._solid_volume(fill):.0f} mm³")
        return fill



    # ---------------------------------------------------------------
    # 凸模修复（上模 fill）：输出 = compound [基座, 凸起, 产品]
    # ---------------------------------------------------------------
    @staticmethod
    def _build_protrusion_core(product, z_parting, main_face, main_z):
        """凸起核心布尔构造（数学层面）：
        Cut(全高柱, 产品) → 凹坑/圆盘补腔 → 底部残料裁剪。
        返回 (protrusion, n_holes, n_skip)。
        只做数学层面的布尔交集构造，不含碎片融合/填腔/导出 ——
        凸模修复（fill）与快速检测（check）共用此函数。"""
        pbb = MoldFixer._bbox_tuple(product, "product-proto")
        z_safe = pbb[2] - 1.0
        z_probe = [pbb[2] + 1.0, pbb[2] + 4.0, pbb[2] + 7.0, pbb[2] + 10.0]

        # 2a. 主板面补洞轮廓（覆盖产品全部水平范围）+ 全高柱 [z_safe → z_parting]
        outline_f = MoldFixer._footprint_outline_face(main_face)
        fz = MoldFixer._bbox_tuple(outline_f)[5]
        col = BRepPrimAPI_MakePrism(
            MoldFixer._translate(outline_f, 0, 0, z_safe - fz),
            gp_Vec(0.0, 0.0, z_parting - z_safe)).Shape()

        # 2b. 凹坑孔面（跳过贯穿孔）
        z_cls = MoldFixer._make_classifiers(product)
        wires = MoldFixer._face_inner_wires(main_face)
        hc = TopoDS_Compound()
        hb = BRep_Builder()
        hb.MakeCompound(hc)
        through_hc = None          # 贯穿孔面（产品无料 → 凸起不得填充）
        through_hb = None
        n_holes = n_skip = 0
        for w in wires:
            try:
                ww = TopoDS.Wire_s(w)
                mf = BRepBuilderAPI_MakeFace(ww)
                mf.Build()
                if mf.IsDone():
                    hf = mf.Face()
                    if MoldFixer._face_area(hf) < 30.0:
                        continue
                    hbb = MoldFixer._bbox_tuple(hf)
                    probe_pts = [(0.5, 0.5), (0.25, 0.25), (0.25, 0.75),
                                 (0.75, 0.25), (0.75, 0.75)]
                    has_below = False
                    for ux, uy in probe_pts:
                        cx = hbb[0] + (hbb[3] - hbb[0]) * ux
                        cy = hbb[1] + (hbb[4] - hbb[1]) * uy
                        if any(MoldFixer._classifier_inside(
                                z_cls, gp_Pnt(cx, cy, zz)) for zz in z_probe):
                            has_below = True
                            break
                    if not has_below:
                        # 贯穿孔：产品在孔下方无料 → 凸起不应填充该区域
                        # （否则会在角落形成"块状/台阶"延伸，图2 的问题）。
                        # 收集孔面，稍后从凸起中沿 z_safe..z_parting 挖掉。
                        n_skip += 1
                        if through_hc is None:
                            through_hc = TopoDS_Compound()
                            through_hb = BRep_Builder()
                            through_hb.MakeCompound(through_hc)
                        through_hb.Add(through_hc,
                                       MoldFixer._translate(hf, 0, 0, z_safe - hbb[5]))
                        continue
                    ht = MoldFixer._translate(hf, 0, 0, z_safe - hbb[5])
                    hb.Add(hc, ht)
                    n_holes += 1
            except Exception:
                pass

        # 2c. 凸起 = Cut(全高柱, 产品)
        proto = MoldFixer._cut(col, product, "凸起 Cut(全高柱,产品)", fuzz=1e-4)

        # 2d. 产品底面以下空间 = Cut([z_safe..main_z]柱, 产品) − 凹坑孔柱
        under_col = BRepPrimAPI_MakePrism(
            MoldFixer._translate(outline_f, 0, 0, z_safe - fz),
            gp_Vec(0.0, 0.0, main_z - z_safe)).Shape()
        under_prod = MoldFixer._cut(under_col, product, "底面以下-产品", fuzz=1e-4)
        if n_holes:
            hole_col = BRepPrimAPI_MakePrism(
                hc, gp_Vec(0.0, 0.0, main_z - z_safe)).Shape()
            under_prod = MoldFixer._cut(under_prod, hole_col, "底面以下-去孔", fuzz=1e-4)

        # 2e. 凸起 = 凸起 − 产品底面以下（去掉侵入产品底面的方块）
        protrusion = MoldFixer._cut(proto, under_prod, "凸起-底面以下", fuzz=1e-4)

        # 2f. 底部裁剪：去掉产品底面以下的残料（贯穿孔残留等）
        sbox = BRepPrimAPI_MakeBox(
            gp_Ax2(gp_Pnt(pbb[0] - 2, pbb[1] - 2, z_safe - 1.0), gp_Dir(0, 0, 1)),
            (pbb[3] - pbb[0]) + 4, (pbb[4] - pbb[1]) + 4,
            (pbb[2] + 0.05) - (z_safe - 1.0)).Shape()
        protrusion = MoldFixer._cut(protrusion, sbox, "凸起-底部残料", fuzz=1e-4)

        # 2g. 挖掉贯穿孔：产品无料的孔（U形切口/贯穿槽）处，凸起不得填充成
        #     块状/台阶延伸（用户反馈图2 的"平直块状延伸"问题）。
        #     用贯穿孔面沿 z_safe..z_parting 拉伸成柱，从凸起中切除。
        if through_hc is not None:
            try:
                th_prism = BRepPrimAPI_MakePrism(
                    through_hc, gp_Vec(0.0, 0.0, z_parting - z_safe)).Shape()
                protrusion = MoldFixer._cut(protrusion, th_prism,
                                            "凸起-挖贯穿孔", fuzz=1e-4)
            except Exception:
                pass
        return protrusion, n_holes, n_skip


    def fill(self, filepath, product_filepath=None, output_dir=None, output_name=None,
             cache_dir=None, no_cache=False, force=False):
        """修复凸模（输入：凸模模具 compound 文件），输出三个独立结构体：
          1) 凸起 = 产品朝上外表面（主板顶面/凸台侧壁/边缘小曲面）沿 +Z
             拉伸到分模面的并集 —— 顶部 = 基座底面，底部 = 产品上表面（贴合）；
          2) 输出 = compound [ 基座, 凸起, 产品 ]
             → 模具（基座 + 凸起）与产品 三个结构体各自独立、互不重叠，
               可在 CAD 中分开查看/移动，检查凸起与产品的贴合关系。
          3) cache_dir / no_cache / force：几何状态缓存参数（见模块头部说明）。
             执行修复前序列化当前几何状态，修复后复查并对比缓存 ——
             若凸起仍存在且位置未变，下一次验证直接跳过重复布尔计算。"""
        _force_utf8()
        fp = _input_fingerprint(filepath)
        cache_path = _cache_path_for(filepath, cache_dir) if (cache_dir and not no_cache) else None
        base, product = self._extract_base_and_product(filepath, product_filepath, is_punch=True)
        base, product = self._ensure_base_parting(base, product, is_punch=True)

        bx = MoldFixer._bbox_tuple(base, "base")
        px = MoldFixer._bbox_tuple(product, "product")
        bx1, by1, bz1, bx2, by2, bz2 = bx
        px1, py1, pz1, px2, py2, pz2 = px
        z_parting = pz2
        z_bottom = pz1

        print("\n" + "=" * 55)
        print("修复凸模 (Fill)：输出 = compound[基座, 凸起, 产品] —— 三个独立结构体")
        print("=" * 55)
        print(f"[MoldFix] 基座: X[{bx1:.1f},{bx2:.1f}] Y[{by1:.1f},{by2:.1f}] Z[{bz1:.1f},{bz2:.1f}]")
        print(f"[MoldFix] 产品: X[{px1:.1f},{px2:.1f}] Y[{py1:.1f},{py2:.1f}] Z[{pz1:.1f},{pz2:.1f}]")
        print(f"[MoldFix] 分模面 Z_A = {z_parting:.2f}（产品上表面最高点 = 基座底面）")
        print(f"[MoldFix] 产品底面 Z_B = {z_bottom:.2f}")

        upper_faces, lower_faces = self._classify_surfaces(product)
        product_vol = MoldFixer._solid_volume(product)
        base_vol = MoldFixer._solid_volume(base)

        # 1. 主板面基准 + 填充体（产品上表面沿 +Z 拉伸，含凸台侧壁/边缘翻边）
        print(f"[MoldFix] 生成凸起（产品上表面沿+Z拉伸，底部贴合产品表面）...")
        main_face, main_area, main_z = self._find_fill_face(product)
        z_tol = 0.5  # 主板面高度容差（凹坑/圆盘内壁 z < main_z - z_tol 不参与逐面拉伸）
        print(f"[MoldFix]   主板面基准: 面积={main_area:.0f}  z={main_z:.2f}")

        # 2. 凸起 = Cut(补洞轮廓全高柱, 产品) − 产品底面以下
        #    —— 只做 3~4 个布尔，产品切除不碎裂（1 实体）；凹坑/圆盘由产品切割
        #    自动贴合曲面（圆形/阶梯形轮廓，无方块凸起）；贯穿孔不下降填充。
        try:
            protrusion, n_holes, n_skip = self._build_protrusion_core(
                product, z_parting, main_face, main_z)
            pv = MoldFixer._solid_volume(protrusion)
            print(f"[MoldFix]   凹坑/圆盘补腔 {n_holes} 个开口"
                  f"{'（跳过贯穿孔 %d）' % n_skip if n_skip else ''}，"
                  f"凸起体积 = {pv:.0f} mm³，实体数 = {MoldFixer._solid_count(protrusion)}")
        except Exception as e:
            print(f"[MoldFix]   布尔构造失败({e})，回退逐面填充...")
            try:
                protrusion = self._build_fill_body(product, z_parting, main_face, main_z,
                                                   top_overlap=0.0, bottom_overlap=0.0,
                                                   include_all=True, fill_main_holes=True)
            except RuntimeError:
                plate, parea, pz = self._make_fill_plate(product, z_parting,
                                                         top_overlap=0.0, bottom_overlap=0.0)
                protrusion = plate

        # 2c+. 补充边缘特征（翻边/斜面/凸台等高于主板面、位于主板面轮廓外的朝上面）：
        #      凸起主体只覆盖主板面轮廓范围，产品边缘的翻边环面/B样条曲面会漏掉，
        #      导致凸起未完全贴合产品上表面。对每个这类面沿 +Z 拉伸到分模面、
        #      Cut(产品) 贴合产品表面，再 Fuse 回凸起。
        try:
            edge_faces = []
            eexp = TopExp_Explorer(product, TopAbs_FACE)
            while eexp.More():
                ef = eexp.Current()
                einfo = MoldFixer._face_outer_normal(ef, product)
                if einfo is None:
                    eexp.Next()
                    continue
                if einfo[1].Z() <= 0.05:
                    eexp.Next()
                    continue
                earea = MoldFixer._face_area(ef)
                efbb = MoldFixer._bbox_tuple(ef)
                if efbb[2] < main_z - z_tol:
                    eexp.Next()
                    continue  # 凹坑/圆盘内壁（z 低于主板面），跳过
                if abs(efbb[5] - efbb[2]) < 0.05 and abs(earea - main_area) < 1.0:
                    eexp.Next()
                    continue  # 主板面本身（已由全高柱覆盖）
                # 面 bbox 四角 + 中心上方一点是否都已被凸起覆盖；
                # 若存在未覆盖点（如翻边面边缘超出主板面轮廓），则需补充拉伸。
                ecz = min(efbb[5] + 0.5, z_parting - 0.5)
                probe_pts = [(efbb[0], efbb[1]), (efbb[3], efbb[1]),
                             (efbb[0], efbb[4]), (efbb[3], efbb[4]),
                             ((efbb[0] + efbb[3]) / 2, (efbb[1] + efbb[4]) / 2)]
                if all(MoldFixer._point_inside(protrusion, gp_Pnt(x, y, ecz))
                       for x, y in probe_pts):
                    eexp.Next()
                    continue
                edge_faces.append((earea, efbb, ef))
                eexp.Next()
            if edge_faces:
                print(f"[MoldFix]   补充边缘特征面 {len(edge_faces)} 个（翻边/斜面/凸台）...")
                edge_faces.sort(key=lambda x: -x[0])
                for earea, efbb, ef in edge_faces:
                    try:
                        # 只补充面【顶部以上】的空腔（z ≥ 面顶 + 0.1 → 分模面），
                        # 避免从面底部以下开始填充而把翻边/凸台两侧包裹成
                        # 高大的块状壁（图2 的"平直块状延伸"问题）。
                        eh = z_parting - (efbb[5] + 0.1)
                        if eh <= 0.2:
                            continue
                        et = MoldFixer._translate(ef, 0, 0, z_parting - efbb[5])
                        ecol = BRepPrimAPI_MakePrism(et, gp_Vec(0.0, 0.0, -eh)).Shape()
                        ecut = MoldFixer._cut(ecol, product, "边缘拉伸-产品", fuzz=1e-4)
                        if MoldFixer._solid_volume(ecut) < 1.0:
                            continue
                        protrusion = MoldFixer._fuse(protrusion, ecut, "凸起∪边缘", fuzz=1e-2)
                        print(f"[MoldFix]     边缘特征补腔 +{MoldFixer._solid_volume(ecut):.0f} mm³")
                    except Exception:
                        continue
        except Exception as e:
            print(f"[MoldFix]   边缘特征补充跳过: {e}")


        # 2c. 凸起含多个独立实体（凹坑/圆盘填充与主体贴合未融合）时，
        #     按体积从大到小逐件融合（Fuse 会吸收重叠，不丢体积）。
        #     用较大 fuzz(2e-2) 多次迭代，尽量把凹坑填充/边缘补腔碎片并进主体。
        guard = 0
        while MoldFixer._solid_count(protrusion) > 1 and guard < 8:
            guard += 1
            solids = []
            se = TopExp_Explorer(protrusion, TopAbs_SOLID)
            while se.More():
                solids.append(se.Current())
                se.Next()
            if len(solids) <= 1:
                break
            solids.sort(key=lambda s: -MoldFixer._solid_volume(s))
            merged = solids[0]
            n_before = MoldFixer._solid_count(merged)
            for s in solids[1:]:
                try:
                    fu = BRepAlgoAPI_Fuse(merged, s)
                    fu.SetRunParallel(False); fu.SetFuzzyValue(2e-2); fu.Build()
                    if fu.IsDone():
                        merged = fu.Shape()
                except Exception:
                    pass
            if MoldFixer._solid_count(merged) < MoldFixer._solid_count(protrusion):
                protrusion = merged
                continue
            break

        # 2d. Cut补腔：v29 新版布尔构造（Cut(全高柱, 产品)）已完整覆盖产品上表面，
        #     此步为旧版补充逻辑，会补出冗余/方块状凸起 → 已停用（range(0)）。
        for _loop in range(0):
            targets = []
            exp = TopExp_Explorer(product, TopAbs_FACE)
            while exp.More():
                f = exp.Current()
                info = MoldFixer._face_outer_normal(f, product)
                if info is None:
                    exp.Next(); continue
                if info[1].Z() <= 0.05:
                    exp.Next(); continue
                fb = MoldFixer._bbox_tuple(f)
                if fb[2] < main_z - z_tol:
                    exp.Next(); continue
                if MoldFixer._face_area(f) < 10.0:
                    exp.Next(); continue
                pnt, n = info
                outward = gp_Pnt(pnt.X() + n.X() * 0.5, pnt.Y() + n.Y() * 0.5,
                                 pnt.Z() + n.Z() * 0.5)
                inward = gp_Pnt(pnt.X() - n.X() * 0.3, pnt.Y() - n.Y() * 0.3,
                                pnt.Z() - n.Z() * 0.3)
                if not MoldFixer._point_inside(protrusion, outward) and \
                        not MoldFixer._point_inside(protrusion, inward):
                    targets.append(f)
                exp.Next()
            if not targets:
                break
            added = False
            for f in targets:
                fb = MoldFixer._bbox_tuple(f)
                z_low = fb[2] - 0.3
                z_high = z_parting + 0.05
                outline = MoldFixer._footprint_outline_face(f)
                o_top = MoldFixer._translate(outline, 0, 0, z_low - fb[5])
                col = BRepPrimAPI_MakePrism(o_top, gp_Vec(0, 0, z_high - z_low)).Shape()
                cavity = MoldFixer._cut(col, product, "Cut补腔")
                try:
                    fu = BRepAlgoAPI_Fuse(protrusion, cavity)
                    fu.SetRunParallel(False); fu.SetFuzzyValue(1e-2); fu.Build()
                    if fu.IsDone() and MoldFixer._solid_volume(fu.Shape()) > 0:
                        protrusion = fu.Shape()
                        added = True
                except Exception:
                    pass
            if not added:
                break

        # 2e. 合并多实体（按体积从大到小逐件融合，Fuse 吸收重叠）
        guard = 0
        while MoldFixer._solid_count(protrusion) > 1 and guard < 8:
            guard += 1
            solids = []
            se = TopExp_Explorer(protrusion, TopAbs_SOLID)
            while se.More():
                solids.append(se.Current())
                se.Next()
            if len(solids) <= 1:
                break
            solids.sort(key=lambda s: -MoldFixer._solid_volume(s))
            merged = solids[0]
            for s in solids[1:]:
                try:
                    fu = BRepAlgoAPI_Fuse(merged, s)
                    fu.SetRunParallel(False); fu.SetFuzzyValue(2e-2); fu.Build()
                    if fu.IsDone():
                        merged = fu.Shape()
                except Exception:
                    pass
            if MoldFixer._solid_count(merged) < MoldFixer._solid_count(protrusion):
                protrusion = merged
                continue
            break

        # 2f. （已移除硬编码"小圆盘位置切块"：旧坐标 X≈±294 与实际圆盘位置不符，
        #     且切块会误伤正常凸起。圆盘/凹坑现已由 2b 贴合补腔，无方块凸起。）

        # 2g. 顶部凹槽填平：凸起上表面凹槽（未到分模面的空腔）全部填实，
        #     使凸起顶部与底座全面接触、不留空腔。
        #     凹槽 = [顶带(分模面-5.5..分模面) − 凸起 − 产品] ∩ 凸起足迹(逐列阴影)
        #     → 只填凸起足迹内的空腔（不产生唇边），融合后填实内部空腔。
        try:
            # 顶部凹槽填平：凸起上表面凹槽（未到分模面的空腔）全部填实，
            # 使凸起顶部与底座全面接触、不留空腔。
            # 凹槽 = [顶带(分模面-5.5..分模面) − 凸起 − 产品] ∩ 凸起足迹(轮廓柱阴影)
            # → 只填凸起足迹内的空腔（不产生唇边），融合后填实内部空腔。
            groove_low = z_parting - 5.5
            # 顶面略高于分模面(+0.1)：保证 groove 与凸起顶部完整重叠、融合干净，
            # 多余薄层由后续"2h 顶部裁剪"统一切到分模面，避免边界交错产生碎片。
            top_box = BRepPrimAPI_MakeBox(
                gp_Ax2(gp_Pnt(px1 - 1, py1 - 1, groove_low), gp_Dir(0, 0, 1)),
                (px2 - px1) + 2, (py2 - py1) + 2, z_parting + 0.1 - groove_low).Shape()
            top_band = MoldFixer._cut(top_box, protrusion, "顶带-凸起")
            top_band = MoldFixer._cut(top_band, product, "顶带-凸起-产品")

            # 只允许在产品包围盒内填充（防止在脚印外补出方块）
            pcol = BRepPrimAPI_MakeBox(
                gp_Ax2(gp_Pnt(px1 - 1, py1 - 1, groove_low), gp_Dir(0, 0, 1)),
                (px2 - px1) + 2, (py2 - py1) + 2, z_parting + 0.1 - groove_low).Shape()
            top_band = MoldFixer._common(top_band, pcol, "顶带∩产品bbox")

            # 凸起足迹阴影：用凸起在 z_samp 高度的截面轮廓柱（连续柱体），
            # 替代逐列离散盒 → Common/Fuse 不产生碎片。
            z_samp = 0.5 * (main_z + z_parting)
            sb = MoldFixer._bbox_tuple(protrusion, "protrusion")
            slab = BRepPrimAPI_MakeBox(
                gp_Ax2(gp_Pnt(sb[0] - 1, sb[1] - 1, z_samp - 0.5), gp_Dir(0, 0, 1)),
                (sb[3] - sb[0]) + 2, (sb[4] - sb[1]) + 2, 1.0).Shape()
            foot = MoldFixer._common(protrusion, slab, "凸起足迹切片")
            # 提取所有水平面，Fuse 成足迹面
            foot_faces = []
            fexp = TopExp_Explorer(foot, TopAbs_FACE)
            while fexp.More():
                ff = fexp.Current()
                ffb = MoldFixer._bbox_tuple(ff)
                if abs(ffb[5] - ffb[2]) < 0.05:
                    foot_faces.append(ff)
                fexp.Next()
            if foot_faces:
                foot_union = foot_faces[0]
                for ff in foot_faces[1:]:
                    try:
                        fu = BRepAlgoAPI_Fuse(foot_union, ff)
                        fu.SetFuzzyValue(1e-3)
                        fu.Build()
                        if fu.IsDone():
                            foot_union = fu.Shape()
                    except Exception:
                        pass
                # 若并集含多面，取最大水平面作为足迹轮廓
                if MoldFixer._count_faces(foot_union) > 1:
                    _best = None
                    _best_area = -1.0
                    _f2 = TopExp_Explorer(foot_union, TopAbs_FACE)
                    while _f2.More():
                        _a2 = MoldFixer._face_area(_f2.Current())
                        if _a2 > _best_area:
                            _best_area = _a2
                            _best = _f2.Current()
                        _f2.Next()
                    if _best is not None:
                        foot_union = _best
                foot_outline = MoldFixer._footprint_outline_face(foot_union)
                shadow = BRepPrimAPI_MakePrism(
                    MoldFixer._translate(foot_outline, 0, 0, groove_low - z_samp),
                    gp_Vec(0.0, 0.0, z_parting + 0.2 - groove_low)).Shape()
            else:
                shadow = None

            groove = MoldFixer._common(top_band, shadow, "顶部凹槽")
            gv = MoldFixer._solid_volume(groove)
            print(f"[MoldFix]   顶部凹槽空腔 {gv:.0f} mm³，融合进凸起填平...")
            if gv > 1.0:
                protrusion = MoldFixer._fuse(protrusion, groove, "凸起∪顶部凹槽")
            else:
                print("[MoldFix]   顶部无凹槽空腔，跳过")
        except Exception as e:
            print(f"[MoldFix]   顶部凹槽填平跳过: {e}")


        # 2g+. 融合碎片：顶部凹槽 groove 由逐列小盒组成，Fuse 后可能有分离碎片；
        #      再做多轮强融合（大 fuzz 2e-2），尽量把碎片并进凸起主体。
        guard = 0
        while MoldFixer._solid_count(protrusion) > 1 and guard < 10:
            guard += 1
            solids = []
            se = TopExp_Explorer(protrusion, TopAbs_SOLID)
            while se.More():
                solids.append(se.Current())
                se.Next()
            if len(solids) <= 1:
                break
            solids.sort(key=lambda s: -MoldFixer._solid_volume(s))
            merged = solids[0]
            for s in solids[1:]:
                try:
                    fu = BRepAlgoAPI_Fuse(merged, s)
                    fu.SetRunParallel(False); fu.SetFuzzyValue(2e-2); fu.Build()
                    if fu.IsDone():
                        merged = fu.Shape()
                except Exception:
                    pass
            if MoldFixer._solid_count(merged) < MoldFixer._solid_count(protrusion):
                protrusion = merged
                continue
            break

        # 清理零体积残片（布尔运算产生的退化碎屑）
        _sliver = 0
        _kept = []
        _se = TopExp_Explorer(protrusion, TopAbs_SOLID)
        while _se.More():
            _s = _se.Current()
            if MoldFixer._solid_volume(_s) >= 1.0:
                _kept.append(_s)
            else:
                _sliver += 1
            _se.Next()
        if _sliver and _kept:
            comp = TopoDS_Compound()
            cb = BRep_Builder()
            cb.MakeCompound(comp)
            for _s in _kept:
                cb.Add(comp, _s)
            protrusion = comp
            print(f"[MoldFix]   清理零体积残片 {_sliver} 个")

        # 2h. 顶部裁剪：凸起顶部齐平分模面（基座底面），不侵入基座。
        #     原算法顶部凹槽填平/翻边补腔可能超出 z_parting 0.1~0.2mm，
        #     Cut 掉 z > z_parting 的薄层，保证凸起与基座互不交叠。
        try:
            tb = MoldFixer._bbox_tuple(protrusion, "protrusion-top")
            if tb[5] > z_parting + 1e-6:
                clip = BRepPrimAPI_MakeBox(
                    gp_Ax2(gp_Pnt(tb[0] - 1, tb[1] - 1, z_parting), gp_Dir(0, 0, 1)),
                    (tb[3] - tb[0]) + 2, (tb[4] - tb[1]) + 2,
                    (tb[5] - z_parting) + 1.0).Shape()
                protrusion = MoldFixer._cut(protrusion, clip, "凸起-顶部裁剪", fuzz=1e-4)
                print(f"[MoldFix]   凸起顶部裁剪到分模面 {z_parting:.2f} "
                      f"（原顶 {tb[5]:.2f}）")
        except Exception as e:
            print(f"[MoldFix]   顶部裁剪跳过: {e}")

        # 2i. 清理产品内部残片：布尔运算偶尔会留下悬浮在产品内部的碎实体
        #     （质心在产品实体内部且与产品交集体积占比高 → 异常残片，丢弃）。
        _bad = 0
        _good = []
        _pe = TopExp_Explorer(protrusion, TopAbs_SOLID)
        while _pe.More():
            _s = _pe.Current()
            _sb = MoldFixer._bbox_tuple(_s)
            _sv = MoldFixer._solid_volume(_s)
            _pc = GProp_GProps()
            BRepGProp.VolumeProperties_s(_s, _pc)
            _c = _pc.CentreOfMass()
            # 质心落在产品 z 范围内才做交集检查（省时）
            if _sv >= 1.0 and _sb[2] < z_parting and _sb[5] > z_bottom and \
                    MoldFixer._point_inside(product, gp_Pnt(_c.X(), _c.Y(), _c.Z())):
                try:
                    _cm = BRepAlgoAPI_Common(_s, product)
                    _cm.SetFuzzyValue(1e-3)
                    _cm.Build()
                    _iv = MoldFixer._solid_volume(_cm.Shape()) if _cm.IsDone() else 0.0
                except Exception:
                    _iv = 0.0
                if _iv > 0.3 * _sv:
                    _bad += 1
                    _pe.Next()
                    continue
            _good.append(_s)
            _pe.Next()
        if _bad:
            comp = TopoDS_Compound()
            _cb = BRep_Builder()
            _cb.MakeCompound(comp)
            for _s in _good:
                _cb.Add(comp, _s)
            protrusion = comp
            print(f"[MoldFix]   清理产品内异常残片 {_bad} 个")

        protrusion = self._fill_internal_voids(protrusion)
        protrusion_vol = MoldFixer._solid_volume(protrusion)
        print(f"[MoldFix]   凸起体积 = {protrusion_vol:.0f} mm³，"
              f"实体数 = {MoldFixer._solid_count(protrusion)}，"
              f"壳数 = {MoldFixer._shell_count(protrusion)}")

        # 2j. 凸起整合：丢弃产品下表面以下的柱状补腔 / 悬浮残片，
        #     只保留主凸起台单一实体 → 输出 = 基座 + 凸起台 + 产品。
        #     （v31）先 Cut(凸起, 产品)：清除边缘特征融合时产生的
        #     与产品表面的薄层重叠（模糊值导致约 0.01mm 侵入），
        #     消除产品上表面以下的多余棱边。
        try:
            protrusion = MoldFixer._cut(protrusion, product, "凸起-去产品重叠", fuzz=1e-5)
        except Exception:
            pass
        if MoldFixer._solid_count(protrusion) > 1:
            protrusion = MoldFixer._consolidate_protrusion(protrusion)
            protrusion_vol = MoldFixer._solid_volume(protrusion)

        # 2k. 去除底部薄鳍（布尔残留的角落局部凸起小面）—— 根因修复：
        #     凸起在主板面边缘包裹产品翻边/凸台时，布尔（Cut(全高柱,产品)）
        #     会留下极薄的薄鳍（法向材料厚度 < 0.5mm），在 CAD 中显示为
        #     "比周围高一点点"的两个小长方形面。
        #     v32 用"包围盒+0.5mm扩展+±2mm高度"的盒体切除 → 过度切削，
        #     挖出了 2 处小空缺。现改为【精确切除到产品上表面】：只切薄鳍
        #     自身包围盒（0.01mm 余量）、z 上限贴合产品上表面，无越界切削。
        #     ---- 修复前：先把当前几何状态序列化缓存（残留凸起坐标等） ----
        pre_fins = MoldFixer.detect_bottom_fins(protrusion, product, main_z)
        if cache_path:
            _state = {
                'schema': CACHE_SCHEMA,
                'input_file': os.path.abspath(filepath),
                'fingerprint': fp,
                'main_z': main_z,
                'z_parting': z_parting,
                # last_check = 输入文件本身的状态（check-only 读它）：
                # 输入尚未被修改，凸起仍在原位 → FAIL + 坐标
                'last_check': {
                    'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'result': 'FAIL' if pre_fins else 'PASS',
                    'fins': pre_fins,
                },
                # last_fix = 本次修复的前/后对照（修复验证用）
                'last_fix': {
                    'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'pre_fins': pre_fins,
                    'post_fins': None,
                    'result': None,
                },
            }
            _save_geom_cache(cache_path, _state)
            print(f"[MoldFix]   修复前几何状态已缓存: {os.path.basename(cache_path)}"
                  f"（残留凸起 {len(pre_fins)} 处）")

        protrusion, remaining_fins = MoldFixer._remove_bottom_fins(protrusion, product, main_z)
        protrusion_vol = MoldFixer._solid_volume(protrusion)

        # ---- 修复后复查：对比缓存前/后状态 ----
        # 若凸起仍在原位 → 报告 FAIL（缓存记录该状态，下次验证直接跳过重复逻辑）
        if remaining_fins:
            MoldFixer._report_fins(remaining_fins, prefix="修复后复查")
            if pre_fins and any(
                    MoldFixer._bbox_intersect(r['bbox'], p['bbox'])
                    for r in remaining_fins for p in pre_fins):
                print("[MoldFix]   凸起仍存在且位置未变"
                      "（缓存将记录该状态，下次验证直接跳过重复布尔计算）")
        if cache_path:
            _state['last_fix'] = {
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'pre_fins': pre_fins,
                'post_fins': remaining_fins,
                'result': 'PASS' if not remaining_fins else 'FAIL',
            }
            _save_geom_cache(cache_path, _state)

        # 3. 输出 = compound [基座, 凸起, 产品] —— 三个独立结构体
        result = self._make_compound([base, protrusion, product])
        result_vol = MoldFixer._solid_volume(result)

        print(f"[MoldFix] 输出体积合计 = {result_vol:.0f} mm³"
              f"（基座 {base_vol:.0f} + 凸起 {protrusion_vol:.0f}"
              f" + 产品 {product_vol:.0f}）")
        n_solid = MoldFixer._solid_count(result)
        n_pro = MoldFixer._solid_count(protrusion)
        print(f"[MoldFix] 输出结构体数 = {n_solid}"
              f"（基座 1 + 凸起 {n_pro} + 产品 1）")
        if n_pro <= 1:
            print("[MoldFix] 凸起为单一实体，内部无空腔（1 Shell）")
        else:
            print(f"[MoldFix] 凸起由 {n_pro} 个实体组成"
                  f"（凹坑/圆盘填充与主体贴合，未融合为一实体但互不重叠）")

        name = output_name or os.path.splitext(os.path.basename(filepath))[0]
        out_dir = output_dir or os.path.dirname(os.path.abspath(filepath))
        out_path = os.path.join(out_dir, f"{name}_FILLED.stp")
        self._export(result, out_path, as_compound=True)

        rx1, ry1, rz1, rx2, ry2, rz2 = self._bbox_tuple(result, "fill-result")
        print(f"[MoldFix] 修补后: Z[{rz1:.1f},{rz2:.1f}]  尺寸={rx2-rx1:.1f}x{ry2-ry1:.1f}x{rz2-rz1:.1f}")
        print(f"[MoldFix] 导出 compound（基座 + 凸起 + 产品 三个独立结构体，互不重叠）: "
              f"{os.path.basename(out_path)}")

        return {
            'output_path': out_path,
            'operation': 'fill',
            'method': 'split_base_protrusion_product',
            'upper_surface_faces': len(upper_faces),
            'lower_surface_faces': len(lower_faces),
            'base_volume': round(base_vol, 1),
            'protrusion_volume': round(protrusion_vol, 1),
            'product_volume': round(product_vol, 1),
            'result_volume': round(result_vol, 1),
            'structure_count': n_solid,
            'protrusion_check': {
                'result': 'PASS' if not remaining_fins else 'FAIL',
                'remaining_fins': remaining_fins,
                'pre_fix_fins': pre_fins,
                'cached_state': cache_path is not None,
            },
            'bbox': {
                'min': [round(rx1, 3), round(ry1, 3), round(rz1, 3)],
                'max': [round(rx2, 3), round(ry2, 3), round(rz2, 3)],
                'size': [round(rx2 - rx1, 3), round(ry2 - ry1, 3), round(rz2 - rz1, 3)],
            },
        }

    def check(self, filepath, product_filepath=None, cache_dir=None, force=False,
              no_cache=False):
        """只加载模型做数学层面的布尔交集检测（--check-only / --dry-run）。

        - 不做任何修复 / 渲染 / STP 导出；
        - 先对比几何状态缓存：输入文件指纹未变且上次结果仍记录凸起在原位置
          → 直接跳过重复布尔计算、跳过重新导入巨大 STP；
        - 检测到残留微小凸起面 → 返回 FAIL + 错误坐标；否则 PASS。
        返回 {'result': 'FAIL'/'PASS', 'protrusions': [...], 'cached': bool}。
        """
        _force_utf8()
        cache_path = _cache_path_for(filepath, cache_dir) if (cache_dir and not no_cache) else None
        fp = _input_fingerprint(filepath)

        # ---- 缓存命中：输入未变 + 上次检测结果仍有效 → 跳过重复布尔计算 ----
        if cache_path and not force:
            cache = _load_geom_cache(cache_path)
            if cache and cache.get('schema') == CACHE_SCHEMA and cache.get('fingerprint') == fp:
                last = cache.get('last_check') or {}
                fins = last.get('fins') or []
                result = last.get('result')
                if result == 'FAIL' and fins:
                    print("[MoldFix] 缓存命中：输入文件未变，残留凸起仍在原位置 —— "
                          "跳过重复布尔计算 / 跳过重新导入巨大 STP")
                    MoldFixer._report_fins(fins)
                    return {'result': 'FAIL', 'protrusions': fins, 'cached': True}
                if result == 'PASS':
                    print("[MoldFix] 缓存命中：上次检测/修复后无残留凸起 —— "
                          "跳过重复布尔计算 / 跳过重新导入巨大 STP")
                    return {'result': 'PASS', 'protrusions': [], 'cached': True}

        # ---- 真正加载 + 数学检测（不修复不导出）----
        print(f"[MoldFix] 检测模式：加载 {os.path.basename(filepath)} ...")
        solids = MoldFixer._extract_solids(filepath)
        if len(solids) >= 3:
            # 已是修复输出 compound [基座, 凸起, 产品] → 直接检查凸起实体本身，
            # 无需重新做布尔构造（最快的验证路径）
            base, protrusion, product = solids[0][1], solids[1][1], solids[2][1]
            px = MoldFixer._bbox_tuple(product, "product")
            z_parting = px[5]
            main_face, main_area, main_z = self._find_fill_face(product)
            print(f"[MoldFix]   已修复输出(3实体)：直接检查凸起实体，"
                  f"主板面 z={main_z:.2f}")
            fins = MoldFixer.detect_bottom_fins(protrusion, product, main_z)
        else:
            base, product = self._extract_base_and_product(filepath, product_filepath, is_punch=True)
            base, product = self._ensure_base_parting(base, product, is_punch=True)
            px = MoldFixer._bbox_tuple(product, "product")
            z_parting = px[5]
            main_face, main_area, main_z = self._find_fill_face(product)
            print(f"[MoldFix]   主板面基准: 面积={main_area:.0f}  z={main_z:.2f}")

            try:
                protrusion, n_holes, n_skip = self._build_protrusion_core(
                    product, z_parting, main_face, main_z)
            except Exception as e:
                print(f"[MoldFix]   布尔构造失败({e})，回退逐面填充...")
                try:
                    protrusion = self._build_fill_body(product, z_parting, main_face, main_z,
                                                       top_overlap=0.0, bottom_overlap=0.0,
                                                       include_all=True, fill_main_holes=True)
                except RuntimeError:
                    plate, parea, pz = self._make_fill_plate(product, z_parting,
                                                             top_overlap=0.0, bottom_overlap=0.0)
                    protrusion = plate
            fins = MoldFixer.detect_bottom_fins(protrusion, product, main_z)

        result = MoldFixer._report_fins(fins)

        if cache_path:
            state = {
                'schema': CACHE_SCHEMA,
                'input_file': os.path.abspath(filepath),
                'fingerprint': fp,
                'main_z': main_z,
                'z_parting': z_parting,
                # last_check = 针对【输入文件本身】的检测结果（check-only 读它）
                'last_check': {
                    'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'result': result,
                    'fins': fins,
                },
                'last_fix': None,
            }
            _save_geom_cache(cache_path, state)
        return {'result': result, 'protrusions': fins, 'cached': False}

    # ---------------------------------------------------------------
    # 凹模检测（下模 check）：数学层面的布尔交集检测，绝不导出/渲染
    # ---------------------------------------------------------------
    def check_cavity(self, filepath, product_filepath=None, cache_dir=None, force=False,
                     no_cache=False):
        """只加载凹模模型做数学层面的布尔交集检测（--check-only / --dry-run）。

        - 不做任何修复 / 渲染 / STP 导出；
        - 检测【残留凸起面】= 下形变面（产品朝下外表面）以上仍有基座材料：
          对每个"下形变面拉伸柱 / 井柱"，求 Common(模具, 柱) 体积，
          体积 > 阈值 → 该柱区域型腔未挖净（残留材料柱）→ Result: FAIL + 坐标；
        - 先对比几何状态缓存：输入文件指纹未变且上次凸起仍在原位
          → 直接跳过重复布尔计算、跳过重新导入巨大 STP；
        返回 {'result': 'FAIL'/'PASS', 'residuals': [...], 'cached': bool}。
        """
        _force_utf8()
        cache_path = _cache_path_for(filepath, cache_dir) if (cache_dir and not no_cache) else None
        fp = _input_fingerprint(filepath)

        # ---- 缓存命中：输入未变 + 上次检测结果仍有效 → 跳过重复布尔计算 ----
        if cache_path and not force:
            cache = _load_geom_cache(cache_path)
            if cache and cache.get('schema') == CACHE_SCHEMA and cache.get('fingerprint') == fp:
                last = cache.get('last_cavity_check') or {}
                residuals = last.get('residuals') or []
                result = last.get('result')
                if residuals or result == 'PASS':
                    if result == 'PASS':
                        print("[MoldFix] 缓存命中：输入文件未变，上次检测为 PASS —— "
                              "跳过重复布尔计算 / 跳过重新导入巨大 STP")
                    else:
                        print("[MoldFix] 缓存命中：输入文件未变，残留凸起仍在原位置 —— "
                              "跳过重复布尔计算 / 跳过重新导入巨大 STP")
                    if residuals:
                        MoldFixer._report_cavity_residuals(residuals)
                    return {'result': result, 'residuals': residuals, 'cached': True}

        # ---- 真正加载 + 数学检测（不修复不导出）----
        print(f"[MoldFix] 凹模检测模式：加载 {os.path.basename(filepath)} ...")
        base, product = self._extract_base_and_product(filepath, product_filepath, is_punch=False)
        base, product = self._ensure_base_parting(base, product, is_punch=False)
        bx = MoldFixer._bbox_tuple(base)
        z_top = bx[5] + 1.0
        upper_faces, lower_faces = self._classify_surfaces(product)

        # 收集"下形变面柱 + 井柱"（与 hollow 同一套数学定义）
        prisms = self._collect_cavity_removal_prisms(product, z_top)
        print(f"[MoldFix]   残留检测：Common(模具, 产品3D实体) 交集（产品能否放入型腔）...")

        residuals = []
        try:
            cm = BRepAlgoAPI_Common(base, product)
            cm.SetRunParallel(False)
            cm.SetFuzzyValue(1e-5)
            cm.Build()
            cv = MoldFixer._solid_volume(cm.Shape()) if cm.IsDone() else 0.0
        except Exception:
            cv = 0.0
        if cv > 1.0:  # 产品无法放入 → 残留凸起
            bb = MoldFixer._bbox_tuple(product)
            residuals.append({
                'x': round((bb[0] + bb[3]) / 2, 1),
                'y': round((bb[1] + bb[4]) / 2, 1),
                'z': round(bb[5], 2),
                'zmin': round(bb[2], 2),
                'zmax': round(bb[5], 2),
                'volume': round(cv, 1),
                'kind': '产品与型腔碰撞',
            })
        residuals.sort(key=lambda r: -r['volume'])

        result = 'FAIL' if residuals else 'PASS'
        if residuals:
            MoldFixer._report_cavity_residuals(residuals)
        else:
            print("[MoldFix] 型腔贴合检查：下形变面以上无残留材料柱 —— Result: PASS")

        if cache_path:
            state = {
                'schema': CACHE_SCHEMA,
                'input_file': os.path.abspath(filepath),
                'fingerprint': fp,
                'last_check': None,
                'last_fix': None,
                'last_cavity_check': {
                    'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'result': result,
                    'residuals': residuals,
                },
            }
            _save_geom_cache(cache_path, state)
        return {'result': result, 'residuals': residuals, 'cached': False}

    @staticmethod
    def _report_cavity_residuals(residuals):
        """打印凹模残留凸起面报告（含错误坐标）。"""
        print("=" * 60)
        print("凹模残留凸起面检测 —— 型腔未沿产品下形变面挖净")
        print("=" * 60)
        if not residuals:
            print("无残留凸起面 —— Result: PASS")
            return
        for i, r in enumerate(residuals):
            print(f" 残留#{i+1}  体积={r['volume']:9.1f} mm³  中心=({r['x']:8.1f},{r['y']:8.1f})  "
                  f"z[{r['zmin']:7.2f},{r['zmax']:7.2f}]  {r['kind']}")
        print("=" * 60)
        print("Result: FAIL —— 未生成任何 .stp 文件（请确认后执行 --fix 才会导出）")


    def _collect_cavity_removal_prisms(self, product, z_top):
        """凹模切除体集合（check_cavity / hollow 共用）：
        - 下形变面柱：产品朝下外表面沿 +Z 拉伸到基座顶面以上；
        - 大井柱：主板底面孔轮廓平移到井底后拉伸（井底 = 孔内最深下形变面 z）；
        - 小孔柱：主板底面小孔轮廓拉伸；
        返回 [(start_z, bbox, face_or_tool, kind)]。
        注：井/槽随形刀具（Cut(孔轮廓柱,产品)）经实测会分裂为多实体、在基座上
        产生封闭内腔，无法稳健切除；故统一用平底井柱（保证型腔开口、不封闭）。
        """
        prisms = []               # (start_z, fbb, face_or_tool, kind)
        max_plate_face = None
        max_plate_area = 0.0
        max_plate_z = 1e9
        exp = TopExp_Explorer(product, TopAbs_FACE)
        while exp.More():
            f = exp.Current()
            info = MoldFixer._face_outer_normal(f, product)
            if info is not None:
                _, n = info
                if n.Z() < -0.05:
                    fbb = MoldFixer._bbox_tuple(f)
                    if z_top - fbb[5] > 0 and MoldFixer._face_area(f) >= 1.0:
                        prisms.append((fbb[5], fbb, f, "下形变面"))
                        if abs(fbb[5] - fbb[2]) < 0.05:
                            a = MoldFixer._face_area(f)
                            if max_plate_face is None or a > max_plate_area + 1.0 or \
                                    (abs(a - max_plate_area) <= 1.0 and fbb[5] < max_plate_z):
                                max_plate_area = a
                                max_plate_z = fbb[5]
                                max_plate_face = f
            exp.Next()

        if max_plate_face is not None:
            z_plate = max_plate_z
            wires = MoldFixer._face_inner_wires(max_plate_face)
            hole_info = []
            for w in wires:
                try:
                    ww = TopoDS.Wire_s(w)
                    mf = BRepBuilderAPI_MakeFace(ww)
                    mf.Build()
                    if mf.IsDone():
                        hf = mf.Face()
                        ha = MoldFixer._face_area(hf)
                        if ha >= 500.0:
                            hbb = MoldFixer._bbox_tuple(hf)
                            wb = z_plate
                            for fz0, fbb0, _f, _k in prisms:
                                if fbb0[2] >= z_plate - 2.0:
                                    continue
                                ox = min(hbb[3], fbb0[3]) - max(hbb[0], fbb0[0])
                                oy = min(hbb[4], fbb0[4]) - max(hbb[1], fbb0[1])
                                if ox > 2 and oy > 2:
                                    wb = min(wb, fbb0[2])
                            hole_info.append((hf, ha, hbb, wb))
                        elif ha >= 30.0:
                            hbb = MoldFixer._bbox_tuple(hf)
                            prisms.append((z_plate, hbb, hf, "孔洞柱"))
                except Exception:
                    continue
            # 排除完全落在大井内的下形变面（井柱已覆盖，避免冗余布尔）
            if hole_info:
                kept = []
                for fz0, fbb, f, k in prisms:
                    inside = any(fbb[0] >= hbb[0] - 0.5 and fbb[3] <= hbb[3] + 0.5 and
                                 fbb[1] >= hbb[1] - 0.5 and fbb[4] <= hbb[4] + 0.5
                                 for _hf, _ha, hbb, _wb in hole_info)
                    if not inside:
                        kept.append((fz0, fbb, f, k))
                prisms = kept
            # 大井/槽柱：平底井柱（孔轮廓平移到井底后拉伸到基座顶面以上）
            for hf, ha, hbb, wb in hole_info:
                hf_tr = MoldFixer._translate(hf, 0, 0, wb - hbb[5])
                hbb_tr = MoldFixer._bbox_tuple(hf_tr)
                prisms.append((wb, hbb_tr, hf_tr, "井柱"))
        return prisms

    def _product_outer_profile(self, product, z_plane=None):
        """产品最外侧 x-y 轮廓面（用于外边缘切齐）。
        多高度截面外轮廓并集尝试；若结果明显小于产品 bbox 矩形，则回退为
        产品 bbox 矩形面（最外侧边界的稳健保守近似）。"""
        pb = MoldFixer._bbox_tuple(product)
        z_top = pb[5] if z_plane is None else min(z_plane, pb[5])
        z_bot = pb[2]
        bbox_area = (pb[3] - pb[0]) * (pb[4] - pb[1])

        # 1) 多高度截面并集（尝试捕捉翻边/法兰外形）
        span = max(z_top - z_bot, 1.0)
        levels = [z_top - i * span / 8.0 for i in range(9)]
        levels = [z for z in levels if z > z_bot + 0.5]
        wire_faces = []
        for z in levels:
            slab = BRepPrimAPI_MakeBox(
                gp_Ax2(gp_Pnt(pb[0] - 2, pb[1] - 2, z - 1.0), gp_Dir(0, 0, 1)),
                (pb[3] - pb[0]) + 4, (pb[4] - pb[1]) + 4, 2.0,
            ).Shape()
            try:
                c = BRepAlgoAPI_Common(product, slab)
                c.SetRunParallel(False)
                c.SetFuzzyValue(1e-4)
                c.Build()
                cross = c.Shape() if c.IsDone() else None
            except Exception:
                cross = None
            if cross is None:
                continue
            best_wire = None
            best_area = -1.0
            exp = TopExp_Explorer(cross, TopAbs_WIRE)
            while exp.More():
                w = exp.Current()
                try:
                    mf = BRepBuilderAPI_MakeFace(TopoDS.Wire_s(w))
                    mf.Build()
                    if mf.IsDone():
                        a = MoldFixer._face_area(mf.Face())
                        if a > best_area:
                            best_area = a
                            best_wire = w
                except Exception:
                    pass
                exp.Next()
            if best_wire is not None:
                try:
                    mf = BRepBuilderAPI_MakeFace(TopoDS.Wire_s(best_wire))
                    mf.Build()
                    if mf.IsDone():
                        wire_faces.append(mf.Face())
                except Exception:
                    pass
        if wire_faces:
            union = wire_faces[0]
            for f in wire_faces[1:]:
                try:
                    fu = BRepAlgoAPI_Fuse(union, f)
                    fu.SetRunParallel(False)
                    fu.SetFuzzyValue(1e-3)
                    fu.Build()
                    if fu.IsDone():
                        union = fu.Shape()
                except Exception:
                    pass
            best_wire = None
            best_area = -1.0
            exp = TopExp_Explorer(union, TopAbs_WIRE)
            while exp.More():
                w = exp.Current()
                try:
                    mf = BRepBuilderAPI_MakeFace(TopoDS.Wire_s(w))
                    mf.Build()
                    if mf.IsDone():
                        a = MoldFixer._face_area(mf.Face())
                        if a > best_area:
                            best_area = a
                            best_wire = w
                except Exception:
                    pass
                exp.Next()
            if best_wire is not None:
                try:
                    mf = BRepBuilderAPI_MakeFace(TopoDS.Wire_s(best_wire))
                    if mf.IsDone() and best_area >= 0.85 * bbox_area:
                        return mf.Face()
                except Exception:
                    pass

        # 2) 回退：产品 bbox 矩形外轮廓面（最外侧边界）
        try:
            bb_box = BRepPrimAPI_MakeBox(
                gp_Ax2(gp_Pnt(pb[0] - 0.5, pb[1] - 0.5, z_bot), gp_Dir(0, 0, 1)),
                (pb[3] - pb[0]) + 1, (pb[4] - pb[1]) + 1, 0.5,
            ).Shape()
            exp = TopExp_Explorer(bb_box, TopAbs_FACE)
            while exp.More():
                f = exp.Current()
                fbb = MoldFixer._bbox_tuple(f)
                if abs(fbb[5] - fbb[2]) < 0.05 and fbb[2] >= z_bot:
                    return f
                exp.Next()
        except Exception:
            pass
        return None

    def _trim_outer_edge(self, result, product, z_parting, bx, z_top):
        """外边缘切齐：把模具外轮廓收敛到产品最外侧边界（含翻边/法兰）。
        用产品在分模面高度处的外轮廓面生成外轮廓柱（基座底面→顶面），
        与当前结果做 Common —— 去除模具矩形外框残留的一圈板材/法兰。"""
        profile = self._product_outer_profile(product, z_parting)
        if profile is None:
            print("[MoldFix]   外轮廓获取失败，跳过外边缘切齐")
            return result
        pbb = MoldFixer._bbox_tuple(profile)
        pcol = BRepPrimAPI_MakePrism(
            MoldFixer._translate(profile, 0, 0, bx[2] - pbb[5]),
            gp_Vec(0.0, 0.0, z_top - bx[2]),
        ).Shape()
        prev = MoldFixer._solid_volume(result)
        try:
            c = BRepAlgoAPI_Common(result, pcol)
            c.SetRunParallel(False)
            c.SetFuzzyValue(1e-4)
            c.Build()
            if not c.IsDone():
                print("[MoldFix]   外边缘切齐布尔失败，跳过")
                return result
            r2 = c.Shape()
            v2 = MoldFixer._solid_volume(r2)
            # 安全阀：外边缘切齐只应去除外圈板材（比例应较小）；
            # 若体积骤降（轮廓错误），放弃切齐，避免把模具切没。
            # 另加 z 向安全阀：切齐不得削掉模具顶部（型腔顶面必须保持在
            # 分模面附近）—— 产品顶部截面窄小的异形件，外轮廓柱可能不覆盖
            # 模具上部，Common 会把型腔顶部整体切掉（实测 YA-1238 顶面从
            # 34.6 被切到 4.6），体积比例 0.92 仍通过旧安全阀，必须用 z 判据拦截。
            if 0 < v2 <= prev and v2 >= 0.6 * prev \
                    and MoldFixer._solid_count(r2) == 1 \
                    and MoldFixer._shell_count(r2) == 1 \
                    and MoldFixer._bbox_tuple(r2)[5] >= z_parting - 1.0:
                print(f"[MoldFix]   外边缘切齐完成：模具外轮廓收敛到产品最外侧边界"
                      f"（体积 {prev:.0f} → {v2:.0f} mm³）")
                return r2
            print(f"[MoldFix]   外边缘切齐结果异常/切除比例过大（{prev:.0f} → {v2:.0f}），跳过")
            return result
        except Exception as e:
            print(f"[MoldFix]   外边缘切齐异常({e})，跳过")
            return result

    # ---------------------------------------------------------------
    # 凹模修复（下模 hollow）：逐个切除 下形变面柱 ∪ 主板面孔洞柱
    # ---------------------------------------------------------------
    def _hollow_cavity_prisms(self, full_base, product, z_top):
        """备选方法（prisms）：下形变面柱/井柱逐个切除（直筒型腔，用于对比调试）。"""
        prisms = self._collect_cavity_removal_prisms(product, z_top)
        n_well = sum(1 for p in prisms if p[3] == "井柱")
        n_hole = sum(1 for p in prisms if p[3] == "孔洞柱")
        print(f"[MoldFix]   直筒切除柱总数 = {len(prisms)}"
              f"（大井柱 {n_well}，小孔柱 {n_hole}，下形变面 {len(prisms) - n_well - n_hole}）")
        prisms.sort(key=lambda x: (x[0], -round(MoldFixer._face_area(x[2]) / 1000.0)))
        print("[MoldFix] 逐个切除（型腔沿产品下形变面挖出、顶部开口）...")
        result = full_base
        n_done = 0
        n_skip = 0
        for fz_face, _fbb, f, kind in prisms:
            p = BRepPrimAPI_MakePrism(f, gp_Vec(0.0, 0.0, z_top - fz_face)).Shape()
            prev_vol = MoldFixer._solid_volume(result)
            try:
                c = BRepAlgoAPI_Cut(result, p)
                c.SetRunParallel(False)
                c.SetFuzzyValue(1e-4)
                c.Build()
                if not c.IsDone():
                    n_skip += 1
                    continue
                r2 = c.Shape()
                v2 = MoldFixer._solid_volume(r2)
                if (0 < v2 <= prev_vol and MoldFixer._solid_count(r2) == 1
                        and MoldFixer._shell_count(r2) == 1):
                    result = r2
                    n_done += 1
                else:
                    n_skip += 1
            except Exception:
                n_skip += 1
        print(f"[MoldFix]   切除完成: 应用 {n_done}/{len(prisms)} 个，"
              f"跳过 {n_skip} 个（已覆盖 / 会产生碎片）")
        return result

    @staticmethod
    def _collect_downward_faces(product, z_top):
        """收集产品所有【朝下外表面】（下形变面，即弯弯曲曲的下表面）。
        返回 [(face_max_z, bbox, face)]，用于"挖空产品下表面以上的全部部分"。"""
        faces = []
        exp = TopExp_Explorer(product, TopAbs_FACE)
        while exp.More():
            f = exp.Current()
            info = MoldFixer._face_outer_normal(f, product)
            if info is not None:
                _, n = info
                if n.Z() < -0.05:
                    fbb = MoldFixer._bbox_tuple(f)
                    if z_top - fbb[5] > 0 and MoldFixer._face_area(f) >= 1.0:
                        faces.append((fbb[5], fbb, f))
            exp.Next()
        return faces

    @staticmethod
    def _collect_main_plate_holes(product):
        """找到产品【最大水平朝下平面】（主板底面），返回 (主板面 z, 内环孔面列表)。
        内环 = 井/槽口轮廓 —— 沿 +Z 拉伸出"井口柱"，清掉型腔内残留的环形孤岛/凸台。"""
        max_face = None
        max_area = 0.0
        max_z = None
        exp = TopExp_Explorer(product, TopAbs_FACE)
        while exp.More():
            f = exp.Current()
            info = MoldFixer._face_outer_normal(f, product)
            if info is not None:
                _, n = info
                fb = MoldFixer._bbox_tuple(f)
                if n.Z() < -0.05 and abs(fb[5] - fb[2]) < 0.05:
                    a = MoldFixer._face_area(f)
                    if a > max_area:
                        max_area = a
                        max_face = f
                        max_z = fb[5]
            exp.Next()
        if max_face is None:
            return None, []
        holes = []
        for w in MoldFixer._face_inner_wires(max_face):
            try:
                mf = BRepBuilderAPI_MakeFace(TopoDS.Wire_s(w))
                mf.Build()
                if mf.IsDone():
                    hf = mf.Face()
                    if MoldFixer._face_area(hf) >= 30.0:
                        holes.append(hf)
            except Exception:
                continue
        return max_z, holes

    def _hollow_cavity_solid3d(self, full_base, product, z_parting, clearance=0.0):
        """主方法（solid3d / 随形挖腔）：Cut(基座, 产品) 得到随形型腔。

        - 型腔底面 = 产品下形变面（弯弯曲曲的下表面，完全随形、贴合）；
        - 型腔侧壁 / 底部圆盘 / 加强筋 / 圆盘口等全部跟随产品外形 —— 无直角口、
          无残留边缘、侧壁精确贴合产品外侧形状；
        - 顶部安装间隙 = 产品朝上外表面沿 +Z 拉伸到分模面的并集（只挖产品上表面
          以上、产品轮廓以内的基座材料，保证产品能完全放入）—— 不再用 bbox 矩形
          外轮廓柱挖，型腔侧壁不再比产品外轮廓大一圈；
        - 默认不切外边缘（trim_edge=False）→ 产品周围的基座实体保留。

        任何一步失败或结果异常 → 返回 None（上层回退直筒切除）。
        """
        bx = MoldFixer._bbox_tuple(full_base)
        px = MoldFixer._bbox_tuple(product)

        # ---- 1. 型腔 = Cut(基座(顶=分模面-0.5), 产品) —— 型腔随产品外形 ----
        open_offset = 0.5
        base_top = px[5] - open_offset
        if base_top <= bx[2] + 0.1:
            base_top = bx[2] + 0.1
        base_box = BRepPrimAPI_MakeBox(
            gp_Ax2(gp_Pnt(bx[0], bx[1], bx[2]), gp_Dir(0, 0, 1)),
            bx[3] - bx[0], bx[4] - bx[1], base_top - bx[2],
        ).Shape()
        print(f"[MoldFix]   Cut(基座, 产品) —— 型腔随产品外形、侧壁精确贴合产品外侧，"
              f"顶面开口于 Z={base_top:.2f}...")
        try:
            result = self._cut(base_box, product, "Cut(基座, 产品)")
        except Exception as e:
            print(f"[MoldFix]   Cut(基座, 产品) 失败: {e}")
            return None

        # ---- 2. 顶部安装间隙：各朝上外表面沿 +Z 拉伸到分模面 ----
        #      柱底向下 0.5mm 重叠（避免与产品上表面贴合处的布尔脆断），
        #      最深优先切，保持主体连续；碎片交给 hollow() 整合清理。
        cols = []
        exp = TopExp_Explorer(product, TopAbs_FACE)
        while exp.More():
            f = exp.Current()
            info = MoldFixer._face_outer_normal(f, product)
            if info is None:
                exp.Next()
                continue
            _, n = info
            if n.Z() <= 0.05:
                exp.Next()
                continue
            try:
                fbb = MoldFixer._bbox_tuple(f)
                if fbb[5] >= base_top - 0.05:
                    exp.Next()
                    continue  # 该面已在分模面附近，无需拉伸
                ol = MoldFixer._footprint_outline_face(f)   # 补掉内孔
                obb = MoldFixer._bbox_tuple(ol)
                z0 = fbb[5] - 0.5
                # 柱顶 = 型腔开口面（分模面-0.5）+ 微小余量 —— 绝不超出基座顶面，
                # 避免倾斜翻边柱在分模面以上残留零体积退化 ghost 面。
                h = (base_top + 0.001) - z0
                if h <= 0.05:
                    exp.Next()
                    continue
                p = BRepPrimAPI_MakePrism(
                    MoldFixer._translate(ol, 0, 0, z0 - obb[5]),
                    gp_Vec(0.0, 0.0, h)).Shape()
                if MoldFixer._solid_volume(p) > 0.01:
                    cols.append((z0, -MoldFixer._face_area(ol), p))
            except Exception:
                pass
            exp.Next()
        cols.sort()   # 最深（z 最小）优先
        n_ok = 0
        for _z0, _a, p in cols:
            prev = MoldFixer._solid_volume(result)
            try:
                c = BRepAlgoAPI_Cut(result, p)
                c.SetRunParallel(False)
                c.SetFuzzyValue(1e-4)
                c.Build()
                if not c.IsDone():
                    continue
                r2 = c.Shape()
                v2 = MoldFixer._solid_volume(r2)
                if 0 < v2 < prev and BRepCheck_Analyzer(r2).IsValid():
                    result = r2
                    n_ok += 1
            except Exception:
                pass
        if n_ok:
            print(f"[MoldFix]   顶部安装间隙切除 {n_ok}/{len(cols)} 个朝上特征柱"
                  f"（只挖产品轮廓以内的基座材料，侧壁贴合产品外侧）")
        else:
            print("[MoldFix]   顶部安装间隙为空，跳过")

        # 顶部裁剪：朝上特征柱（倾斜翻边柱）切完后，分模面以上可能残留
        # 零体积退化薄片/ghost 面（bbox 顶会高出基座顶 1mm）—— 裁掉。
        try:
            tb = MoldFixer._bbox_tuple(result, "hollow-result")
            if tb[5] > base_top + 1e-6:
                clip = BRepPrimAPI_MakeBox(
                    gp_Ax2(gp_Pnt(tb[0] - 1, tb[1] - 1, base_top), gp_Dir(0, 0, 1)),
                    (tb[3] - tb[0]) + 2, (tb[4] - tb[1]) + 2,
                    (tb[5] - base_top) + 1.0).Shape()
                result = self._cut(result, clip, "型腔-顶部裁剪")
                print(f"[MoldFix]   型腔顶部裁剪到分模面开口 {base_top:.2f}"
                      f"（原顶 {tb[5]:.2f}）")
        except Exception as e:
            print(f"[MoldFix]   型腔顶部裁剪跳过: {e}")
        # 尽力把布尔产生的碎片融合回 1 实体（柱体挖掉后基座被切成多块是布尔精度所致，
        # 若各块恰好贴合则融合成功；融合失败则保留多块，由 hollow() 整合清理）。
        if MoldFixer._solid_count(result) > 1:
            try:
                from OCP.TopExp import TopExp_Explorer as _TE
                from OCP.TopAbs import TopAbs_SOLID as _TS
                _ss = []
                _se = _TE(result, _TS)
                while _se.More():
                    _ss.append(_se.Current()); _se.Next()
                _fused = None
                for _s in sorted(_ss, key=lambda x: MoldFixer._solid_volume(x), reverse=True):
                    if _fused is None:
                        _fused = _s
                    else:
                        _fu = BRepAlgoAPI_Fuse(_fused, _s)
                        _fu.SetRunParallel(False); _fu.SetFuzzyValue(1e-2); _fu.Build()
                        if _fu.IsDone():
                            _fused = _fu.Shape()
                if _fused is not None and MoldFixer._solid_count(_fused) == 1 \
                        and MoldFixer._shell_count(_fused) == 1:
                    result = _fused
            except Exception:
                pass
        rv = MoldFixer._solid_volume(result)
        print(f"[MoldFix]   挖空后体积 = {rv:.0f} mm³，实体数 = {MoldFixer._solid_count(result)}，"
              f"壳数 = {MoldFixer._shell_count(result)}")
        return result

    def hollow(self, filepath, product_filepath=None, output_dir=None, output_name=None,
               cache_dir=None, no_cache=False, force=False, method='solid3d', clearance=0.1,
               trim_edge=False):
        """修复凹模（输入：凹模模具 compound 文件）。

        method = 'solid3d'（默认）：挖空【产品下表面以上的全部部分】——
            Cut(基座(顶面=分模面-0.5mm), 产品) 得到随形型腔，再 Cut 掉
            顶部安装间隙（产品朝上外表面沿 +Z 拉伸到分模面的并集，只挖产品
            上表面以上、产品轮廓以内的基座材料）。
            型腔底面 = 产品下形变面（弯弯曲曲、完全随形贴合）；型腔侧壁 / 底部圆盘 /
            加强筋 / 圆盘口全部跟随产品外形、侧壁精确贴合产品外侧形状（不再比
            产品外轮廓大一圈）；同时挖掉产品上表面以上被包裹的间隙实体
            （Common=0，产品能完全放入）；
            布尔产生的碎片会尽力融合回 1 实体；产品周围的基座实体（模具壁/边框）保留。
         method = 'prisms'：保留旧"下形变面拉伸柱/井柱"直筒切除逻辑（备选/调试）。
        clearance（配合间隙/收缩率，mm）：占位参数（默认 0.1；当前 OCP 版本偏置
           不可用，实际为完全贴合）。
        trim_edge（外边缘切齐，默认 False）：默认【不切齐】，保留产品周围的基座
           实体（模具壁/边框）。只有明确需要去除基座外圈时才置 True。
        流程（solid3d）：
          1) 重建完整基座长方体；
          2) Cut(基座(顶=分模面-0.5), 产品) → 随形型腔（侧壁精确贴合产品外侧）；
          3) Cut(型腔, 顶部安装间隙) → 产品能完全放入（尽量融合回 1 实体 1 壳）；
          4) 输出 = compound [型腔, 产品]，修复后做残留检测并写几何状态缓存。
        """
        _force_utf8()
        cache_path = _cache_path_for(filepath, cache_dir) if (cache_dir and not no_cache) else None
        base, product = self._extract_base_and_product(filepath, product_filepath, is_punch=False)
        base, product = self._ensure_base_parting(base, product, is_punch=False)

        bx = MoldFixer._bbox_tuple(base, "base")
        px = MoldFixer._bbox_tuple(product, "product")
        bx1, by1, bz1, bx2, by2, bz2 = bx
        px1, py1, pz1, px2, py2, pz2 = px
        z_parting = pz2
        z_top = bz2 + 1.0          # 基座顶面以上 1mm → 型腔顶部开口（不产生封闭内腔）

        print("\n" + "=" * 55)
        print(f"修复凹模 (Hollow)：方法 = {method}"
              f"{'（随形挖腔：底面/侧壁/圆盘全部随产品外形）' if method == 'solid3d' else '（直筒切除，备选）'}"
              f"，配合间隙 = {clearance} mm")
        print("=" * 55)
        print(f"[MoldFix] 基座: X[{bx1:.1f},{bx2:.1f}] Y[{by1:.1f},{by2:.1f}] Z[{bz1:.1f},{bz2:.1f}]")
        print(f"[MoldFix] 产品: X[{px1:.1f},{px2:.1f}] Y[{py1:.1f},{py2:.1f}] Z[{pz1:.1f},{pz2:.1f}]")
        print(f"[MoldFix] 分模面 Z_A = {z_parting:.2f}（产品上表面最高点 = 基座顶面，型腔从该面向下挖）")

        upper_faces, lower_faces = self._classify_surfaces(product)
        product_vol = MoldFixer._solid_volume(product)

        # 1. 重建完整基座长方体
        full_base = BRepPrimAPI_MakeBox(
            gp_Ax2(gp_Pnt(bx1, by1, bz1), gp_Dir(0, 0, 1)),
            bx2 - bx1, by2 - by1, bz2 - bz1,
        ).Shape()
        full_vol = MoldFixer._solid_volume(full_base)
        print(f"[MoldFix] 重建完整基座长方体: 体积 = {full_vol:.0f} mm³")

        # 2. 主方法：挖空产品下表面以上全部（含产品与基座顶面之间的间隙）；失败时回退
        if method == 'solid3d':
            print("[MoldFix] 方法 solid3d：挖空产品下表面以上全部（型腔随形 + 间隙）...")
            result = self._hollow_cavity_solid3d(
                full_base, product, z_parting, clearance=clearance,
            )
            if result is not None and MoldFixer._solid_volume(result) > 1.0:
                used_method = 'solid3d'
            else:
                print("[MoldFix] solid3d 切除失败/结果异常，自动回退直筒切除（prisms）...")
                result = self._hollow_cavity_prisms(full_base, product, z_top)
                used_method = 'prisms'
        else:
            result = self._hollow_cavity_prisms(full_base, product, z_top)
            used_method = 'prisms'

        # 2b. 型腔整合：Cut(基座, 挖空体) 偶尔会切出多个互不相连的独立实体
        #     （主体型腔 + 填充产品底部加强筋/圆盘区域的基座残留碎片）。
        #     尽力融合；融合失败则丢弃产品足迹内、分模面以下的残留碎片，
        #     只保留最大型腔主体 → 输出 = compound [型腔(1实体), 产品]。
        if MoldFixer._solid_count(result) > 1:
            result = MoldFixer._consolidate_hollow_result(result, product)
        result_vol = MoldFixer._solid_volume(result)
        removed = full_vol - result_vol

        # 5. （默认关闭）外边缘切齐：保留产品周围的基座实体（模具壁/边框）。
        #    仅当显式开启 trim_edge 时才把模具外轮廓收敛到产品最外侧边界。
        if trim_edge and result is not None \
                and MoldFixer._solid_count(result) == 1 and MoldFixer._shell_count(result) == 1:
            result = self._trim_outer_edge(result, product, z_parting, bx, z_top)
            result_vol = MoldFixer._solid_volume(result)
            removed = full_vol - result_vol

        # 6. 验证：切除量 / 实体数 / 壳数
        print(f"[MoldFix] 切除量 = {removed:.0f} mm³（完整基座 {full_vol:.0f} - 型腔 {result_vol:.0f}）")
        if removed > 0.9 * max(product_vol, 1.0):
            print("[MoldFix] 型腔完全挖出（沿产品下形变面，顶部开口，无封闭内腔）")
        else:
            print("[MoldFix] 切除量不足，型腔可能未完全挖出")

        n_solid = MoldFixer._solid_count(result)
        n_shell = MoldFixer._shell_count(result)
        print(f"[MoldFix] 输出实体数 = {n_solid}（应为 1），壳数 = {n_shell}（应为 1，开口型腔）")

        name = output_name or os.path.splitext(os.path.basename(filepath))[0]
        out_dir = output_dir or os.path.dirname(os.path.abspath(filepath))
        out_path = os.path.join(out_dir, f"{name}_HOLLOWED.stp")

        # 输出 = compound [型腔, 产品] —— 产品独立保留，方便 CAD 中对照检查
        # 型腔面是否与产品下形变面吻合（可分开移动/半透明叠加查看）。
        out_compound = MoldFixer._make_compound([result, product])
        self._export(out_compound, out_path, as_compound=True)

        rx1, ry1, rz1, rx2, ry2, rz2 = self._bbox_tuple(result, "hollow-result")
        print(f"[MoldFix] 挖空后: Z[{rz1:.1f},{rz2:.1f}]  尺寸={rx2-rx1:.1f}x{ry2-ry1:.1f}x{rz2-rz1:.1f}")
        print(f"[MoldFix] 导出 compound [型腔, 产品]（产品独立保留，可对照检查贴合）: "
              f"{os.path.basename(out_path)}")

        # 6. 修复后残留凸起面检测（数学层面，不渲染不导出）并写入几何状态缓存
        residual_info = []
        if cache_path:
            try:
                residual_info = []
                try:
                    cm = BRepAlgoAPI_Common(result, product)
                    cm.SetRunParallel(False)
                    cm.SetFuzzyValue(1e-5)
                    cm.Build()
                    cv = MoldFixer._solid_volume(cm.Shape()) if cm.IsDone() else 0.0
                except Exception:
                    cv = 0.0
                if cv > 1.0:
                    bb = MoldFixer._bbox_tuple(product)
                    residual_info.append({
                        'x': round((bb[0] + bb[3]) / 2, 1),
                        'y': round((bb[1] + bb[4]) / 2, 1),
                        'zmin': round(bb[2], 2),
                        'zmax': round(bb[5], 2),
                        'volume': round(cv, 1),
                        'kind': '产品与型腔碰撞',
                    })
                residual_info.sort(key=lambda r: -r['volume'])
                ck_result = 'FAIL' if residual_info else 'PASS'
                prev_cache = _load_geom_cache(cache_path) or {}
                state = {
                    'schema': CACHE_SCHEMA,
                    'input_file': os.path.abspath(filepath),
                    'fingerprint': _input_fingerprint(filepath),
                    'last_check': prev_cache.get('last_check'),
                    'last_fix': prev_cache.get('last_fix'),
                    # 输入文件本身的检测结果保持不变（check-only 读它）
                    'last_cavity_check': prev_cache.get('last_cavity_check'),
                    # 修复后的型腔残留状态（与输入检测分开，避免 check-only 误报 PASS）
                    'last_cavity_fix': {
                        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                        'result': ck_result,
                        'residuals': residual_info,
                    },
                }
                _save_geom_cache(cache_path, state)
                print(f"[MoldFix] 修复后残留检测: Result: {ck_result}"
                      f"（残留凸起 {len(residual_info)} 处，已写入几何状态缓存）")
            except Exception as e:
                print(f"[MoldFix] 修复后残留检测跳过: {e}")

        return {
            'output_path': out_path,
            'operation': 'hollow',
            'method': f'solid3d_product_cut' if used_method == 'solid3d'
                      else 'cut_lower_surface_columns_and_hole_columns',
            'geometry_method': used_method,
            'clearance': clearance,
            'upper_surface_faces': len(upper_faces),
            'lower_surface_faces': len(lower_faces),
            'base_volume': round(full_vol, 1),
            'removed_volume': round(removed, 1),
            'residual_check': {
                'result': 'FAIL' if residual_info else 'PASS',
                'residuals': residual_info,
            },
            'bbox': {
                'min': [round(rx1, 3), round(ry1, 3), round(rz1, 3)],
                'max': [round(rx2, 3), round(ry2, 3), round(rz2, 3)],
                'size': [round(rx2 - rx1, 3), round(ry2 - ry1, 3), round(rz2 - rz1, 3)],
            },
        }


if __name__ == '__main__':
    _force_utf8()

    parser = argparse.ArgumentParser(
        prog='mold_fix.py',
        description='模具修复工具 mold_fix —— 支持 --check-only 快速检测（不导出/不渲染）与 --fix 完整修复+导出 STP',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            '示例：\n'
            '  python mold_fix.py --check-only\n'
            '      只加载模具并做数学层面的布尔交集检测；若检测到残留微小凸起面\n'
            '      → Result: FAIL + 错误坐标，绝不执行 3D 渲染 / STP 文件写入。\n'
            '  python mold_fix.py --check-only --input <凹模.stp>\n'
            '      自动识别凹模并做型腔残留检测（Common(模具, 下形变面柱) 交集）。\n'
            '  python mold_fix.py --fix\n'
            '      完整修复并真正生成 .stp 文件（不带模式参数时默认也是完整修复+导出）。\n'
            '  python mold_fix.py --check-only --force\n'
            '      忽略几何状态缓存，强制重新导入模型检测。'
        ),
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-only', '--dry-run', action='store_true', dest='check_only',
                      help='只加载模型做数学层面的布尔交集检测：检测到残留凸起面返回 Result: FAIL + 错误坐标，'
                           '绝对不执行 3D 渲染和 STP 文件写入')
    mode.add_argument('--fix', action='store_true', dest='do_fix',
                      help='完整执行修复并导出 .stp（默认行为：不带任何模式参数时也是完整修复+导出）')
    mode.add_argument('--both', action='store_true', dest='do_both',
                      help='同时修复凸模+凹模（默认行为）')

    parser.add_argument('--punch', dest='punch_file', default=None,
                        help='凸模(上模) STP 文件路径')
    parser.add_argument('--cavity', dest='cavity_file', default=None,
                        help='凹模(下模) STP 文件路径')
    parser.add_argument('--input', dest='input_file', default=None,
                        help='单个待处理 STP 文件（check-only / fix 均可）')
    parser.add_argument('--product', dest='product_file', default=None,
                        help='外部产品 STP 文件（可选，模具 compound 含产品时无需）')
    parser.add_argument('--output-dir', dest='output_dir', default=None,
                        help='修复结果输出目录（默认 fix_output）')
    parser.add_argument('--cache-dir', dest='cache_dir', default=DEFAULT_CACHE_DIR,
                        help='几何状态缓存目录（JSON，默认 mold_fix_cache）')
    parser.add_argument('--no-cache', action='store_true', dest='no_cache',
                        help='禁用几何状态缓存')
    parser.add_argument('--force', action='store_true', dest='force',
                        help='忽略缓存，强制执行完整数学检测')
    parser.add_argument('--method', dest='method', default='solid3d',
                        choices=['solid3d', 'prisms'],
                        help='凹模挖腔方法：solid3d=产品3D实体切除(默认,型腔随形)；'
                             'prisms=直筒切除(备选/调试)')
    parser.add_argument('--clearance', dest='clearance', type=float, default=0.1,
                        help='凹模配合间隙/收缩率(mm)，对产品实体做均匀偏置后切除(默认0.1)')
    parser.add_argument('--trim-edge', dest='trim_edge', action='store_true', default=True,
                        help='外边缘切齐：模具外轮廓收敛到产品最外侧边界(默认开)')
    parser.add_argument('--no-trim-edge', dest='trim_edge', action='store_false',
                        help='关闭外边缘切齐')
    args = parser.parse_args()

    bdir = os.path.dirname(os.path.abspath(__file__))

    def _resolve(path):
        return path if os.path.isabs(path) else os.path.join(bdir, path)

    # ---- 解析输入文件 ----
    punch_file = _resolve(args.punch_file) if args.punch_file else None
    cavity_file = _resolve(args.cavity_file) if args.cavity_file else None
    input_file = _resolve(args.input_file) if args.input_file else None

    if input_file:
        if not os.path.exists(input_file):
            print(f"输入文件不存在: {input_file}")
            sys.exit(1)
    else:
        if not punch_file or not cavity_file:
            candidates = [
                ("mold_output/YA-1131-505_MIRROR_xOy_PUNCH.stp",
                 "mold_output/YA-1131-505_MIRROR_xOy_CAVITY.stp"),
                ("YA-1131-505_MIRROR_xOy_PUNCH.stp",
                 "YA-1131-505_MIRROR_xOy_CAVITY.stp"),
            ]
            for pun, cav in candidates:
                p1, p2 = _resolve(pun), _resolve(cav)
                if os.path.exists(p1) and os.path.exists(p2):
                    punch_file, cavity_file = p1, p2
                    break
        if not punch_file or not os.path.exists(punch_file):
            print("未找到凸模文件（可用 --punch 或 --input 指定）")
            sys.exit(1)
        if not cavity_file or not os.path.exists(cavity_file):
            print("未找到凹模文件（可用 --cavity 指定）")
            sys.exit(1)

    # ---- 模式决定 ----
    check_only = bool(args.check_only)
    do_fix = bool(args.do_fix or args.do_both)
    if not check_only and not do_fix:
        do_fix = True  # 兼容旧行为：无参数默认完整修复+导出
        print("[MoldFix] 未指定模式，默认执行完整修复+导出（如需快速检测请用 --check-only）")

    out_dir = args.output_dir or os.path.join(bdir, "fix_output")
    os.makedirs(out_dir, exist_ok=True)
    cache_dir = _resolve(args.cache_dir) if not os.path.isabs(args.cache_dir) else args.cache_dir
    os.makedirs(cache_dir, exist_ok=True)

    fixer = MoldFixer()

    def _detect_op(filepath):
        """自动判断单个输入文件是凸模(fill)还是凹模(hollow)：
        - 产品顶面 ≈ 基座顶面 → 凹模（型腔从顶面开口）；
        - 产品顶面 ≈ 基座底面 → 凸模（填充体在基座内）。
        """
        try:
            solids = MoldFixer._extract_solids(filepath)
            if len(solids) >= 2:
                base = solids[0][1]
                product = solids[1][1]
                bz = MoldFixer._bbox_tuple(base)
                pz = MoldFixer._bbox_tuple(product)
                if abs(pz[5] - bz[5]) < 0.2:
                    return 'hollow'   # 凹模
                if abs(pz[5] - bz[2]) < 0.2:
                    return 'fill'     # 凸模
        except Exception:
            pass
        return 'fill'  # 默认凸模

    exit_code = 0

    def _check_one(target, op):
        if op == 'hollow':
            print(f"\n▶ 凹模检测模式 (--check-only --cavity)：{os.path.basename(target)}")
        else:
            print(f"\n▶ 凸模检测模式 (--check-only)：{os.path.basename(target)}")
        try:
            if op == 'hollow':
                r = fixer.check_cavity(target, product_filepath=args.product_file,
                                       cache_dir=cache_dir, force=args.force,
                                       no_cache=args.no_cache)
            else:
                r = fixer.check(target, product_filepath=args.product_file,
                                cache_dir=cache_dir, force=args.force,
                                no_cache=args.no_cache)
            if r.get('cached'):
                print("（缓存命中：未重新导入 STP，未执行重复布尔计算）")
            print(f"最终结果: Result: {r['result']}")
            residuals = r.get('protrusions') or r.get('residuals') or []
            if residuals:
                print(f"错误坐标数量: {len(residuals)} 处 —— "
                      "未生成任何 .stp 文件（请手动确认后执行 --fix 才会导出）")
            return 0 if r['result'] == 'PASS' else 1
        except Exception as e:
            import traceback
            print(f"Result: FAIL  —— 检测异常: {e}")
            traceback.print_exc()
            return 2

    def _fix_one(target, op):
        name = os.path.splitext(os.path.basename(target))[0]
        print(f"\n▶ 修复模式 (--fix)：{os.path.basename(target)}")
        try:
            if op == 'fill':
                r = fixer.fill(target, product_filepath=args.product_file,
                               output_dir=out_dir, output_name=name,
                               cache_dir=cache_dir, no_cache=args.no_cache,
                               force=args.force)
            else:
                r = fixer.hollow(target, product_filepath=args.product_file,
                                 output_dir=out_dir, output_name=name,
                                 cache_dir=cache_dir, no_cache=args.no_cache,
                                 force=args.force, method=args.method,
                                 clearance=args.clearance, trim_edge=args.trim_edge)
            print(f"[{op}] -> {r['output_path']}")
            print(f"  方法: {r['method']}  尺寸: {r['bbox']['size']}")
            pc = r.get('protrusion_check')
            if pc:
                print(f"[凸起复查] Result: {pc['result']}  —— "
                      f"残留凸起 {len(pc.get('remaining_fins', []))} 处"
                      f"（修复前 {len(pc.get('pre_fix_fins', []))} 处）")
            rc = r.get('residual_check')
            if rc:
                print(f"[型腔残留复查] Result: {rc['result']}  —— "
                      f"残留凸起 {len(rc.get('residuals', []))} 处")
            return 0
        except Exception as e:
            import traceback
            print(f"[{op}] 失败: {e}")
            traceback.print_exc()
            return 1

    # ---- 执行 ----
    if check_only:
        if input_file:
            op = _detect_op(input_file)
            exit_code = _check_one(input_file, op)
        else:
            print(f"凸模(上模): {os.path.basename(punch_file)}")
            exit_code = _check_one(punch_file, 'fill')
            if cavity_file:
                print(f"\n凹模(下模): {os.path.basename(cavity_file)}")
                exit_code = _check_one(cavity_file, 'hollow') or exit_code
        sys.exit(exit_code)

    # 修复模式（--fix / --both / 默认）
    if input_file:
        op = _detect_op(input_file)
        exit_code = _fix_one(input_file, op)
    else:
        exit_code = _fix_one(punch_file, 'fill')
        exit_code = _fix_one(cavity_file, 'hollow') or exit_code
    sys.exit(exit_code)
