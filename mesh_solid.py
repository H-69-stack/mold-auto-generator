"""STL（三角网格）→ 实体 (mesh_solid.py)

STL 只有三角片、没有拓扑，布尔运算必须先缝合成闭合壳体再做成实体。
流程：``StlAPI_Reader`` 读成三角面片集合 → ``BRepBuilderAPI_Sewing`` 按容差缝合
→ ``ShapeFix_Solid`` 修一下 → ``BRepBuilderAPI_MakeSolid`` 得到实体
→ ``BRepCheck_Analyzer`` + 体积校验。

注意（会写进提示信息，也会在 Web 上提示用户）：
  * STL 是**多面体近似**：曲面会变成许多小平面，生成的模具型腔也是多面体面
    （不是光滑圆弧）。要光滑曲面请用 STEP。
  * 网格必须**闭合**（水密）。不闭合时缝合成壳体后做不出实体，这里会给出
    自由边数量，明确报错，而不是悄悄产出一个错模具。
"""

from typing import Optional, Tuple

from OCP.BRep import BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Sewing
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.ShapeFix import ShapeFix_Solid
from OCP.StlAPI import StlAPI_Reader
from OCP.TopAbs import TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape

from errors import ModelReadError
from geometry_utils import shape_bbox


def _first(shape, kind):
    exp = TopExp_Explorer(shape, kind)
    if exp.More():
        return exp.Current()
    return None


def read_stl_shape(path: str):
    """读取 STL（二进制/ASCII 均可），返回三角面片集合（TopoDS_Shape，通常是 compound）。"""
    shape = TopoDS_Shape()
    reader = StlAPI_Reader()
    ok = False
    try:
        ok = bool(reader.Read(shape, str(path)))
    except Exception as exc:  # noqa: BLE001
        raise ModelReadError(f"STL 读取失败: {exc}") from exc
    if not ok or shape.IsNull():
        raise ModelReadError(
            "STL 读取失败（文件可能损坏，或不是二进制/ASCII STL）")
    return shape


def mesh_to_solid(shape, tolerance: Optional[float] = None,
                  verbose: bool = True) -> Tuple[object, dict]:
    """把三角面片集合缝合成**实体**。返回 (solid, info)。

    tolerance 默认按模型尺寸取（对角线 × 1e-5，最小 1e-3 mm）——STL 顶点常带
    浮点误差，容差太小会缝不上。
    """
    info = {}
    bb = shape_bbox(shape)
    diag = float(((bb[3] - bb[0]) ** 2 + (bb[4] - bb[1]) ** 2
                  + (bb[5] - bb[2]) ** 2) ** 0.5)
    tol = float(tolerance) if tolerance else max(1e-3, diag * 1e-5)
    info["sew_tolerance"] = tol

    sewed = None
    for t in (tol, tol * 10.0, tol * 100.0):
        sew = BRepBuilderAPI_Sewing(t, True, True, True, False)
        sew.Add(shape)
        sew.Perform()
        info["free_edges"] = int(sew.NbFreeEdges())
        info["multiple_edges"] = int(sew.NbMultipleEdges())
        sewed = sew.SewedShape()
        if sewed is not None and not sewed.IsNull() and info["free_edges"] == 0:
            info["sew_tolerance"] = t
            break
    if sewed is None or sewed.IsNull():
        raise ModelReadError("STL 缝合失败：三角面片无法组成壳体")
    if info.get("free_edges"):
        raise ModelReadError(
            "STL 网格**不闭合**（缝合成壳体后仍有 %d 条自由边）："
            "布尔运算做不出实体，请先在 CAD 里补洞/封闭后再导出 STL" % info["free_edges"])

    shell = _first(sewed, TopAbs_SHELL)
    if shell is None:
        if _first(sewed, TopAbs_SOLID) is not None:
            shell = _first(_first(sewed, TopAbs_SOLID), TopAbs_SHELL)
        if shell is None:
            raise ModelReadError("STL 缝合后没有找到封闭壳体")
    shell = TopoDS.Shell_s(shell)

    solid = None
    try:
        fx = ShapeFix_Solid()
        fx.Init(shell)
        fx.Perform()
        solid = fx.Solid()
    except Exception:  # noqa: BLE001
        solid = None
    if solid is None or solid.IsNull():
        try:
            solid = BRepBuilderAPI_MakeSolid(shell).Solid()
        except Exception as exc:  # noqa: BLE001
            raise ModelReadError(f"STL 壳体制成实体失败: {exc}") from exc
    if solid is None or solid.IsNull():
        raise ModelReadError("STL 壳体制成实体失败（几何退化）")

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(solid, props)
    info["volume"] = float(props.Mass())
    if info["volume"] <= 0.0:
        raise ModelReadError(
            "STL 实体体积为 0：网格可能整体外翻或自交，请检查后重新导出")
    try:
        info["valid"] = bool(BRepCheck_Analyzer(solid).IsValid())
    except Exception:  # noqa: BLE001
        info["valid"] = None
    n_faces = 0
    from OCP.TopAbs import TopAbs_FACE
    exp = TopExp_Explorer(solid, TopAbs_FACE)
    while exp.More():
        n_faces += 1
        exp.Next()
    info["faces"] = n_faces
    if verbose:
        print("  [STL] 缝合容差 %.4g mm，自由边 0，面数 %d，体积 %.1f mm³，有效=%s"
              % (info["sew_tolerance"], n_faces, info["volume"], info["valid"]))
        if n_faces > 400:
            print("  [STL 提示] 该网格被还原成 %d 个平面小面（多面体近似）："
                  "生成的模具型腔会是多面体面、不是光滑曲面；"
                  "且面数多会明显变慢，建议尽量用 STEP 原始模型。" % n_faces)
    return solid, info
