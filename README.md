# 模具自动生成系统（S4）

> **2026-09 本轮 7 项修改（详见 [`PIPE_MOLD_README.md`](PIPE_MOLD_README.md)）**
>
> 1. **输出前缀按管件编号自动命名**：`24TK_1324-31_ROTATE_x_CW_90deg_ROTATE_y_CW_4deg.stp`
>    → 前缀 `24TK_1324-31`（`naming_utils.part_number_from_filename`，前后端同一套规则；
>    上传后前端自动填入"输出前缀"，留空时后端也兜底）。
> 2. **镜像 → 镜像 + 旋转**：「🪞 镜像 / 旋转模型」面板可镜像（xOy/xOz/yOz，过质心或不过）
>    和/或绕 X/Y/Z 轴顺时针/逆时针旋转任意角度（0~360），两者可叠加（先镜像后旋转），
>    输出 `编号_MIRROR_xOy_ROT_X_CW_30deg.stp` 并可直接下载。
> 3. **STL 导入可用**：三角网格 → 缝合 → 实体（`mesh_solid.py`）；**不闭合网格明确报错**
>    （给出自由边数）。STL 是多面体近似：型腔会是平面小面，要光滑曲面请用 STEP。
> 4. 管件 **型腔端口被模料堵住** 修复（= 用户说的"端口小台阶 / 尖角 / 管件和模具重叠"）。
>    根因有两个：截断面切在**斜切端口内侧**（芯棒短 0.5~2.9 mm），以及"端面补齐"那一步
>    因 `bool_fuse` 传了 list 而**静默失效**（从写下起就没生效过）。现在截断主路径改为
>    **沿端口端面平面截断**，芯棒长度 = 内孔全长；实测端口内孔里的模料 2 700 → **0.000 mm³**，
>    四种重叠检查全部 0.0000 mm³，整条管线 103.8 → **64.4 s**。
> 5. 管件 **水平分模面永远保留**（取消上一版的"自动切侧影随形"——用户要的就是水平面、
>    高度自己填）；侧影随形分模面剖面改用**插值样条**（更贴合）；安全区间颠倒时只提示、
>    并报出侧影高度范围，不擅自换分模面。
> 6. 管件 **导出件搬回输入数模的原始坐标系**（上模/下模/芯棒/四件套直接和原产品装配就对得上，
>    实测重叠 0.000 mm³）；**盲端单端截断** + **侧向开口检测**（有侧洞时明确提示"需要侧向抽芯"）。
> 7. **`24TK_1442-1` 型腔完全不对** 的根因修复：`_wire_center()` 用"顶点平均"求内孔中心
>    （BSpline 环顶点又少又不均）→ 端口平面的**局部轴向算歪** → 射线打到产品顶面 → 端口平面
>    被拟合成水平面 → 截断把模芯横着切掉一半，"芯棒"变成一块 765×93×22 的**顶板**。
>    现在：`_wire_center` 用**曲线形心**（`BRepGProp.LinearProperties`）、端口平面**逐个命中面
>    校核**（必须是平面 + 法向沿长轴 + 支持平面）、芯棒再加一道**包围盒校验**。
>    实测 1442-1：芯棒 1 444 401（顶板）→ **1 440 609 mm³（内孔全长）**，端口内孔模料 **0.000 mm³**，
>    四种重叠检查全部 0.0000 mm³，耗时 14.2 → **8.9 s**；顺带 2052-2 也 194.9 → **14.5 s**。


## 一、系统概述

本系统用于 **液压成型（折弯成型）金属产品的自动开模**：输入产品三维数模（STEP/IGES/STL），自动生成 **上下模具（凹模/凸模）STEP 文件**。基于 **OpenCASCADE（OCCT 7.8）** 内核，通过 `cadquery-ocp` 提供的 `OCP`（pythonocc 风格）绑定实现几何建模、布尔运算与导出。

项目包含 **两套并存的模具生成逻辑**、一个 **独立的模芯设计模块** 与一个 **Web 前端应用**：

1. **模块化智能模具管线（推荐/主力）** —— `main.py` 串联 6 个模块，从产品数模到上模/下模 STEP。
2. **液压成型模具生成器（旧版）** —— `mold_generator.py` 的 `MoldGenerator`，加上 `mold_fix.py` 的 `MoldFixer`（后续修补）。
3. **模芯设计（芯棒）** —— `core_design.py`：面向薄壁中空件，沿产品长轴两端端口面把内孔**完整填满**，
   生成芯棒（内芯 / mandrel）+ 贴合率/啃料/抽芯检验，详见 [`CORE_DESIGN_README.md`](CORE_DESIGN_README.md)。
4. **Web 三维查看与生成系统** —— `web_box_generator/app.py`（Flask），前端用 Three.js。
   **2026-09 按需求精简**：界面只保留 **上传 / 三维查看 → 镜像 → 智能模具**（管件 → 上模 + 下模 +
   芯棒 + 四件套装配体）这条主线；测量、剖切、旧版模具/修复、产品优化、模芯设计等面板已从界面移除
   （对应的后端路由与模块文件仍保留，需要时可随时接回）。

---

## 二、运行方式

- **Python 环境**：`venv/`（Windows 虚拟环境）。需 `cadquery-ocp`（提供 `OCP` 模块）、`cadquery`、`flask`、`numpy` 等。
- **命令行（模块化管线）**：
  ```bash
  venv\Scripts\python main.py --part <产品.stp> [--core L W H] [--margin MX MY MZ]
      [--base-thickness 20] [--parting-mode max_z|mid|custom|contour|silhouette]
      [--parting-z Z] [--z-align bottom|center] [--output ./output] [--prefix mold]
      [--mold-type sheet|pipe] [--parting-surface plane|silhouette]
      [--no-verify] [--no-core] [--parallel]
  # 模芯设计（芯棒）：沿产品两端端口面把内孔填满（独立出图，可留间隙/逐站位校验）
  venv\Scripts\python main.py --mode core --part <产品.stp> --output ./output --prefix core
  ```
  无参数运行即用内置测试产品（带凸台长方体）跑通全流程。
- **启动 Web**：
  ```bash
  venv\Scripts\python web_box_generator\app.py --port 5001
  # 浏览器访问 http://127.0.0.1:5001
  ```

> **重要**：中文用户名/路径（如本机 `C:\Users\何雨卓\...`）下，OCC 底层 C++ 文件 API 不支持非 ASCII 路径。所有读写在 `io_utils.py` 中统一处理：读前复制到 ASCII 临时路径，写后由 Python 移回。**新增任何直接调用 OCC 读写文件的代码都应复用 `io_utils`，不要直接传中文路径给 OCC。**

---

## 三、模块化智能模具管线（核心，六步）

入口 `main.py::generate_mold()`，返回 dict（`product`/`core`/`aligned`/`cavity_core`/`parting`/`paths`）。
**`mold_type` 选择工艺路线**：

* `mold_type="sheet"`（默认）—— **板件折弯/液压成型**，下面 6 步（每步一个模块文件）；
* `mold_type="pipe"` —— **管件硬模成型**：产品居中 + 外形包络型腔 + 分模面
  （`parting_surface="plane"` 水平面（默认）/ `"silhouette"` 侧影随形）
  + 开模/顶出干涉闭环校验 + 三件套装配体导出，
  由 `main.py::_generate_pipe_mold` + `pipe_mold.py` 实现，
  详见 [`PIPE_MOLD_README.md`](PIPE_MOLD_README.md)。

### 三件套 / 四件套装配体（管件模式默认都导出）

`main.build_mold_assembly()` 把 **[上模, 产品, 下模]**；`main.build_mold4_assembly()` 把
**[上模, 产品, 芯棒, 下模]** 放进 compound（同一坐标系、按实际配合位置摆好，合模状态互不重叠），
导出为 `{prefix}_mold_assembly.step` / `{prefix}_mold4_assembly.step`，可直接打开核对装配关系。

> **管件模式的芯棒是白捡的**：挖"外形包络"型腔时内孔里那块"内芯条"本来就要被切断剔除，
> 它就是产品**内孔腔 = 芯棒**（`pipe_mold.build_envelope_die(keep_pieces=True)`）。
> 所以四件套不额外花布尔时间，也比"再跑一次 `--mode core`"快 ~5 min；
> 四块是同一次布尔切出的互补实体，天然互不重叠。
> 实测：`--mold-type pipe --no-verify --parallel` 一次跑出上下模+芯棒+四件套 **452 s**。

| # | 模块文件 | 函数/类 | 职责 |
|---|----------|---------|------|
| 1 | `core_block_generator.py` | `compute_core_size` / `generate_core_block` | 生成长方体模芯实体 `CoreBlock`（底面中心在 XY 原点，Z∈[0,H]）。尺寸=产品包围盒+双边余量，Z 向为非对称“下基座 + 上余量” |
| 2 | `model_reader.py` | `read_product_model` / `product_model_from_shape` | 读取 STEP/IGES，返回 `ProductModel`（shape、bbox、volume、surface_area、file_path、file_format） |
| 3 | `part_aligner.py` | `align_product_to_core` | 产品平移居中。XY 对齐模芯中心；Z 向 `bottom`（底面落基座顶面）或 `center`（居中）。返回 `AlignmentResult`，并校验产品不越界 |
| 4 | `cavity_cutter.py` | `cut_cavity` | `BRepAlgoAPI_Cut(模芯, 产品)` 挖出型腔。带 FuzzyValue 容差重试（0, 1e-6, 1e-5, 1e-4）与体积校验 |
| 5 | `parting_splitter.py` | `split_mold` / `split_mold_by_contour` / `detect_undercuts` / `find_max_projection_z` | 沿 `z=z_parting` 平面或产品投影轮廓把带型腔模芯切成上模/下模，并检测倒扣区域；`silhouette`/`contour` 默认取产品投影面积最大处为分模面。返回 `PartingResult` |
| 6 | `result_exporter.py` | `export_molds` | 导出 `{prefix}_upper_mold.step` / `_lower_mold.step` / `_metadata.json`，支持额外形状 |

### 各模块关键细节

- **模芯尺寸**（`core_block_generator.py:47`）：`L=产品L+2*mx`, `W=产品W+2*my`, `H=base_thickness+产品H+mz`。必须 ≥ 产品尺寸+双边余量，否则抛 `MoldGenerationError`。
- **分模高度**（`parting_splitter.py:98`）：`max_z`=产品最高点 z_max（上模为平板）；`mid`=高度中点；`custom`=用户指定；`contour` / `silhouette`=沿产品在分模高度处的截面轮廓（外轮廓柱体，上模取轮廓内），未指定分模高度时默认取**产品投影面积最大处**（`find_max_projection_z`）。
  * ⚠ **2026-09 修正**：`find_max_projection_z` 的判据由"截面**材料面积**"改成截面点云的
    **凸包面积**（真正的投影轮廓面积），并在面积平台段取中位高度。
    旧判据对**薄壁中空件**会把分模面选到"平面与曲面相切"的高度（产品最顶/最底的切点），
    实测管件会把分模面选到产品顶面之外。
- **倒扣检测**（仅警告不阻塞）：
  * 朝上（nz>0）的面位于分模面**以下** → 产品顶出会被挡（倒扣）；
  * 朝下（nz<0）的面位于分模面**以上** → 上模抬起会被钩住（倒扣）；
  * 与分模面共面的面给 1e-6 容差。
  * ⚠ **2026-09 修正**：旧判据写反（"朝下且低于分模面"当倒扣），把正常型腔底面全部误报。

### 模芯设计（芯棒 / 内芯）—— `core_design.py`（独立模块）

面向 **薄壁中空件**：硬模成型时产品内孔是空的，需要一根芯棒把内孔撑住。
入口 `main.py --mode core`（或 `core_design.py` 单独运行，`--selftest` 跑内置空心管自测）。

做法：**取芯块 − 产品 − 端口封盖** → 取出**内孔腔**（与管件模具挖腔同一套"封盖切断"机制，
但封盖从端口面**往外**拉伸），再用两个端口面之间的切片把端面切平 → 芯棒端面与端口面**严格齐平**。
生成后自动做三项检验：**内孔贴合率**（贴芯面面积 ÷ 芯棒侧面面积，应为 1.00）、
**是否啃料**（芯棒 ∩ 产品 ≈ 0）、**抽芯干涉**（沿 ±长轴分级平移求干涉 → 能否整体直抽）。

导出 `{prefix}_core_pin.step`（芯棒）与 `{prefix}_core_assembly.step`（产品 + 芯棒装配体）。
详见 [`CORE_DESIGN_README.md`](CORE_DESIGN_README.md)。

### 共享工具层
- `geometry_utils.py`：`shape_bbox`、`shape_volume`、`shape_surface_area`、`is_valid_shape`（BRepCheck）。
- `errors.py`：统一异常体系。基类 `MoldGenerationError`（带 `stage` 标识），子类 `ModelReadError`/`GeometryError`/`BooleanOpError`/`PartingError`/`ExportError`。**上层统一 catch `MoldGenerationError`。**
- `io_utils.py`：非 ASCII 路径处理（见上文）、`ensure_utf8_stdout` 处理 Windows 控制台 GBK 编码问题。

---

## 四、Web 三维查看与生成系统（web_box_generator/app.py）

Flask 后端，`app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)`。上传文件存 `uploads/`，文件名带 `{12位hex}_` 前缀，展示时用 `_strip_uid_prefix` 剥离。

### 页面功能（前端 `templates/index.html`，Three.js 渲染）

上传模型、3D 查看（旋转/缩放/平移、视图立方体、半透明、线框、面拾取属性）、剖切、镜像、模具、智能模具、模芯设计。

> 「🤖 智能模具」面板可选 **模具工艺类型**：`板件折弯/液压成型`（原管线）或
> `管件硬模成型`（产品居中 + 外形包络型腔 + 分模面 + 开模/顶出干涉校验）。
>
> 「🧩 模芯设计（芯棒）」面板面向**中空管件**：沿产品长轴两个端口面把内孔完整填满生成芯棒，
> 结果区给出贴合率/啃料/抽芯结论，并提供芯棒 STEP、产品+芯棒装配体（含半透明与剖视预览）。

### 主要 API 与对应后端模块

| API | 功能 | 调用的模块/类 |
|-----|------|--------------|
| `POST /upload` | 上传 .stp/.step/.stl 并解析 | `step_reader.StepReader`（STEP）/ 内置 STL 解析 |
| `POST /generate` | 生成长方体 .stp/.stl | `BoxGenerator.StepBoxWriter` / `StlBoxWriter` |
| `GET /api/slice_plane` | 平面与模型相交轮廓 | `slice_analyzer.MeshSlicer` |
| `GET /api/slice_plane_batch` | 批量切片（展开长度） | `slice_analyzer.MeshSlicer` |
| `POST /api/generate_mirror` | 关于 xOy/xOz/yOz 面镜像 | `mirror_generator.MirrorGenerator` |
| `POST /api/fix_mold` | 模具修复：`fill`补齐/`hollow`挖空 | `mold_fix.MoldFixer` |
| `POST /api/generate_mold` | 生成上下模具（旧版） | `mold_generator.MoldGenerator` |
| `POST /api/mold_generate` | **模块化管线**（`mold_type=sheet` 板件 / `pipe` 管件硬模成型；可带 `verify` / `with_core` / `run_parallel` / `parting_z`）**异步任务** | `main.generate_mold` |
| `GET /api/task_status` | 查询异步任务状态（running / done / error，附带日志尾部与已用时间） | — |
| `POST /api/pipe_plane_suggest` | **管件水平分模面高度推荐**（推荐值 + 安全区间 + 上下模高度，不跑布尔） | `main.suggest_pipe_plane` |
| `POST /api/core_generate` | **模芯设计**（独立出芯棒图）：沿产品两端端口面把内孔填满 + 贴合/抽芯检验 | `core_design.design_core` |
| `GET /api/core_download` | 下载芯棒 / 产品+芯棒装配体 STEP、元数据 JSON | — |
| `GET /api/model_parse` | 解析 STEP 为前端网格 | `step_reader.StepReader` |
| `GET /api/model_section` | 半剖实体看内部型腔 | `model_reader` + 布尔 Cut |
| `GET /api/model_features` | 折弯/圆孔特征检测 | `brep_topology.BrepTopologyExtractor` + `bend_detector.BendDetector` |

### STEP 网格解析要点（`app.py:_parse_step`）

- **子进程解析**：OCC 网格化是阻塞的原生 C++ 代码，会卡死主进程。用 `multiprocessing` 单进程池，超时（60s）强制终止，页面不无限加载。
- **自适应精度**：先用粗精度(2.0mm)估模型尺寸，再 `linear=size/分段数`（小零件密、大零件粗），并限三角面 ≤20 万（超限降精度重试）。
- **网格清理**：`_clean_mesh` 按 1e-7 容差合并顶点、去退化三角形（前端直接 `computeVertexNormals()` 得平滑法线）。

---

## 五、液压成型模具生成（旧版，mold_generator.py）

`MoldGenerator`（基于 cadquery）：

- 设定产品顶面 `Z_A`（分模面）、底面 `Z_B`。
- **下模/凹模**：`Cut(毛坯[Z_A-100..Z_A], 产品)` —— 顶面与 Z_A 平齐，产品沉入。
- **上模/凸模**：`Cut(毛坯[Z_A..Z_A+100], 产品-0.1mm)` —— 底面贴合产品顶面。

输出为 compound（基座 + 凸起/产品等独立结构体）。Web 的 `/api/generate_mold` 调用它生成 `cavity`（凹模）与 `punch`（凸模）。

---

## 六、模具修复模块（mold_fix.py，约 15 万行，v34）

`MoldFixer` 对 `MoldGenerator` 的输出做精细修补，**输入只需模具 compound 文件（内部已含对齐的产品），通常不需要外部产品文件**（可选 `product_filepath`）。

- `fill()` —— 凸模修复：输出 `compound [基座, 凸起台, 产品]` 三个独立结构体。核心难点：凸起底部要 **精确贴合产品弯曲形变面**，需去除布尔产生的残片、清除与产品重叠薄层、切除底部薄鳍（`detect_bottom_fins`/`_remove_bottom_fins`）。
- `hollow()` —— 凹模修复：沿产品下表面挖空型腔，型腔底面完全随形，顶部在分模面开口。
- `check()` / `check_cavity()` —— 数学层面的残留检测（`--check-only`），不做 3D 渲染/写文件。
- **几何状态缓存**：修复前把残留凸起坐标/面积/厚度等序列化为 `.geom_cache.json`，输入指纹未变时跳过重复布尔计算（实测 52s → 1.9s）。

> **注意**：`mold_fix.py` 包含大量迭代修补的历史逻辑与 `# noqa`，改动需谨慎；建议优先在模块化管线中修改，而非此文件。

---

## 七、几何分析与特征检测模块

- `step_reader.py`：`StepReader` —— 将 STEP 三角网格化并导出顶点/三角面/边/颜色/物理属性/特征；`StepFileScanner` 批量扫描。`export_mesh_json` / `export_features_json` 可导出 JSON。
- `slice_analyzer.py`：`MeshSlicer` —— 沿方向对三角网格求等距截面，计算轮廓弧长（展开尺寸）。
- `brep_topology.py`：`BrepTopologyExtractor` —— 从 STEP 的 BREP 提取 Face/Edge/Vertex、邻接关系、板厚 t、面分类（平面/圆柱/圆角）、基准底板。
- `bend_detector.py`：`BendDetector` —— 基于 BREP 拓扑识别折弯（平面共享边+圆角 = 直角折弯；圆柱面两侧法兰 = 滚弯弧），计算二面角与内半径。
- `BoxGenerator.py`：`StepBoxWriter`（生成 STEP）/ `StlBoxWriter`（生成 STL）/ `BoxGeneratorApp`（Tkinter GUI）。

---

## 八、目录与数据

| 路径 | 说明 |
|------|------|
| `output/` `fix_output/` `mold_output/` `mold_fix_cache/` | 命令行运行的输出/缓存 |
| `mold_fix_backup_*.py` | `mold_fix.py` 的历史备份（可删） |
| `web_box_generator/uploads/` | Web 上传与输出：`mirror_output/`、`fix_output/`、`mold_output/`、`mold_system_output/<task_id>/`、`core_output/<task_id>/` |
| `venv/` | 虚拟环境（勿改） |
| `_repro3_fixed/` | 复现/测试输出 |

---

## 九、给 AI 助手的关键提示

1. **识别调用入口**：`main.py:generate_mold()`（模块化管线）是主力；Web 的 `/api/mold_generate` 就是调它。`MoldGenerator`/`MoldFixer` 是另一套逻辑。`mold_type="pipe"` 走 `_generate_pipe_mold` + `pipe_mold.py`（管件硬模成型，见 `PIPE_MOLD_README.md`）。**模芯设计（芯棒）** 是独立模块：`core_design.py::design_core()`（CLI `main.py --mode core`，Web `/api/core_generate`，见 `CORE_DESIGN_README.md`）。
2. **异常体系**：所有业务错误抛 `MoldGenerationError` 子类，带 `stage`；catch 它即可获得清晰错误信息。
3. **中文路径陷阱**：新增 OCC 读写必须走 `io_utils`（`make_ascii_read_path` / `make_ascii_write_path` / `move_into_place`），否则中文目录下会失败。
4. **控制台编码**：Windows 下 print 中文/单位符号（mm³、✓ 等）会报 UnicodeEncodeError；用 `ensure_utf8_stdout()` 或 reconfigure UTF-8。
5. **布尔运算稳健性**：布尔操作统一带 FuzzyValue 容差重试（`cavity_cutter.bool_cut`/`bool_common`），`run_parallel=False` 更稳定。
6. **分模模式**：`silhouette` / `contour`（沿产品投影轮廓，自动取最大投影截面为分模面）是推荐模式，Web 前端默认；`max_z` 一刀切最简单，但只适合顶部即最宽处的简单件。
7. **体积校验**：挖腔后校验“模芯-型腔模芯”是否等于产品体积（偏差>5% 报错），排查产品是否在模芯内。
8. **OCC 7.8 已知坑**（2026-09 实测，管件开模时踩到）：
   * `BRepPrimAPI_MakePrism` **不接受 solid**（抛 `Standard_NoSuchObject: Solids are not Processed`），只支持 face/shell/wire/edge；
   * 圆柱/BSpline **面**拉伸出的棱柱体积与“面积×距离”不符（比值 0.10~0.98），部分结果 `BRepCheck` 无效；
     多个这种棱柱做布尔求并/求差会大面积失败（`BOP 未完成`）——**不要**用“各面棱柱求并”构造扫掠体；
   * `shape_bbox`（`BRepBndLib`）会被 BSpline 控制点撑大（实测 757.49 → 实际几何 ~756.3；下模真实
     z∈[60.00,106.37]，`BRepBndLib` 却报 [33.00,106.39]）→ **要量尺寸就用三角化包围盒**；
   * 模型里可能存在**零面积面**，参与布尔会产生病态几何，需要过滤；
   * 模具型腔与产品表面**零间隙重合**时，布尔求交要用“平移后的实体 vs 实体”避开共面退化；
     点采样法在这种场合会因网格弦差产生假干涉（实测假报 2.5×10⁵ mm³），不能当主判据；
   * **薄壁件里“点在壁厚内部”常被判成 `ON` 而不是 `IN`**（2 mm 壁厚管件实测）→ 判“是否在材料里”时
     `IN` 与 `ON` 都要算；关键判定优先用射线求交（`IntCurvesFace_ShapeIntersector`）；
   * **面上内环（曲面上的孔口）不要用 `MakeFace(wire)+Prism` 做封盖**（2026-09 实测）：
     会留下病态面，导致布尔结果**写 STEP 时实体丢失**（降级成 shell，但 `BRepCheck` 仍报“有效”，
     `ShapeFix`/缝合/`MakeSolid`/换 fuzzy/换 AP203 与 ManifoldSolidBrep 全都救不回来）
     → 改成“孔口包围盒 → **平面矩形** → 拉棱柱”（`pipe_mold._planar_hole_plate`）。
     **导出后把 STEP 读回来数实体**（`_pipe_diag/verify_mold4.py`）是发现这类问题的可靠手段。

9. **长任务**：Web 端 `/api/mold_generate`、`/api/core_generate`、`/api/pipe_plane_suggest`
   都是**异步任务**（POST 立刻返回 `task_id` → 轮询 `GET /api/task_status`），
   因为大件要跑十几分钟，同步 HTTP 会被浏览器掐断（曾误报“生成超时（4分钟）”而服务器还在算）。
   前端用 `submitTask()` 统一处理，不要再给这些接口加 `AbortController` 超时。
