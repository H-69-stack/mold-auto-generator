# 模具自动生成系统（S4）

## 一、系统概述

本系统用于 **液压成型（折弯成型）金属产品的自动开模**：输入产品三维数模（STEP/IGES/STL），自动生成 **上下模具（凹模/凸模）STEP 文件**。基于 **OpenCASCADE（OCCT 7.8）** 内核，通过 `cadquery-ocp` 提供的 `OCP`（pythonocc 风格）绑定实现几何建模、布尔运算与导出。

项目包含 **两套并存的模具生成逻辑** 与一个 **Web 前端应用**：

1. **模块化智能模具管线（推荐/主力）** —— `main.py` 串联 6 个模块，从产品数模到上模/下模 STEP。
2. **液压成型模具生成器（旧版）** —— `mold_generator.py` 的 `MoldGenerator`，加上 `mold_fix.py` 的 `MoldFixer`（后续修补）。
3. **Web 三维查看与生成系统** —— `web_box_generator/app.py`（Flask），提供上传、3D 查看、剖切分析、镜像、模具生成/修复等功能，前端用 Three.js。

---

## 二、运行方式

- **Python 环境**：`venv/`（Windows 虚拟环境）。需 `cadquery-ocp`（提供 `OCP` 模块）、`cadquery`、`flask`、`numpy` 等。
- **命令行（模块化管线）**：
  ```bash
  venv\Scripts\python main.py --part <产品.stp> [--core L W H] [--margin MX MY MZ]
      [--base-thickness 20] [--parting-mode max_z|mid|custom|contour]
      [--parting-z Z] [--z-align bottom|center] [--output ./output] [--prefix mold]
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

入口 `main.py::generate_mold()`，返回 dict（`product`/`core`/`aligned`/`cavity_core`/`parting`/`paths`）。每步对应一个模块文件：

| # | 模块文件 | 函数/类 | 职责 |
|---|----------|---------|------|
| 1 | `core_block_generator.py` | `compute_core_size` / `generate_core_block` | 生成长方体模芯实体 `CoreBlock`（底面中心在 XY 原点，Z∈[0,H]）。尺寸=产品包围盒+双边余量，Z 向为非对称"下基座 + 上余量" |
| 2 | `model_reader.py` | `read_product_model` / `product_model_from_shape` | 读取 STEP/IGES，返回 `ProductModel`（shape、bbox、volume、surface_area、file_path、file_format） |
| 3 | `part_aligner.py` | `align_product_to_core` | 产品平移居中。XY 对齐模芯中心；Z 向 `bottom`（底面落基座顶面）或 `center`（居中）。返回 `AlignmentResult`，并校验产品不越界 |
| 4 | `cavity_cutter.py` | `cut_cavity` | `BRepAlgoAPI_Cut(模芯, 产品)` 挖出型腔。带 FuzzyValue 容差重试（0, 1e-6, 1e-5, 1e-4）与体积校验 |
| 5 | `parting_splitter.py` | `split_mold` / `split_mold_by_contour` / `detect_undercuts` | 沿 `z=z_parting` 平面或产品投影轮廓把带型腔模芯切成上模/下模，并检测倒扣区域。返回 `PartingResult` |
| 6 | `result_exporter.py` | `export_molds` | 导出 `{prefix}_upper_mold.step` / `_lower_mold.step` / `_metadata.json`，支持额外形状 |

### 各模块关键细节

- **模芯尺寸**（`core_block_generator.py:47`）：`L=产品L+2*mx`, `W=产品W+2*my`, `H=base_thickness+产品H+mz`。必须 ≥ 产品尺寸+双边余量，否则抛 `MoldGenerationError`。
- **分模高度**（`parting_splitter.py:98`）：`max_z`=产品最高点 z_max（上模为平板）；`mid`=高度中点；`custom`=用户指定；`contour`=沿产品在分模高度处的截面轮廓（外轮廓柱体，上模取轮廓内）。
- **倒扣检测**（仅警告不阻塞）：面法向与 +Z 夹角 >90° 且面中心 Z < z_parting 标记为倒扣候选；水平底面单独标记 `is_horizontal`。

### 共享工具层

- `geometry_utils.py`：`shape_bbox`、`shape_volume`、`shape_surface_area`、`is_valid_shape`（BRepCheck）。
- `errors.py`：统一异常体系。基类 `MoldGenerationError`（带 `stage` 标识），子类 `ModelReadError`/`GeometryError`/`BooleanOpError`/`PartingError`/`ExportError`。**上层统一 catch `MoldGenerationError`。**
- `io_utils.py`：非 ASCII 路径处理（见上文）、`ensure_utf8_stdout` 处理 Windows 控制台 GBK 编码问题。

---

## 四、Web 三维查看与生成系统（web_box_generator/app.py）

Flask 后端，`app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)`。上传文件存 `uploads/`，文件名带 `{12位hex}_` 前缀，展示时用 `_strip_uid_prefix` 剥离。

### 页面功能（前端 `templates/index.html`，Three.js 渲染）

上传模型、3D 查看（旋转/缩放/平移、视图立方体、半透明、线框、面拾取属性）、剖切、镜像、模具、智能模具。

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
| `POST /api/mold_generate` | **模块化管线** | `main.generate_mold` |
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

> **注意**：`mold_fix.py` 包含大量迭代修补的历史逻辑，改动需谨慎；建议优先在模块化管线中修改，而非此文件。

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
| `web_box_generator/uploads/` | Web 上传与输出：`mirror_output/`、`fix_output/`、`mold_output/`、`mold_system_output/<task_id>/` |
| `venv/` | 虚拟环境（勿改） |
| `_repro3_fixed/` | 复现/测试输出 |

---

## 九、给 AI 助手的关键提示

1. **识别调用入口**：`main.py:generate_mold()`（模块化管线）是主力；Web 的 `/api/mold_generate` 就是调它。`MoldGenerator`/`MoldFixer` 是另一套逻辑。
2. **异常体系**：所有业务错误抛 `MoldGenerationError` 子类，带 `stage`；catch 它即可获得清晰错误信息。
3. **中文路径陷阱**：新增 OCC 读写必须走 `io_utils`（`make_ascii_read_path` / `make_ascii_write_path` / `move_into_place`），否则中文目录下会失败。
4. **控制台编码**：Windows 下 print 中文/单位符号（mm³、✓ 等）会报 UnicodeEncodeError；用 `ensure_utf8_stdout()` 或 reconfigure UTF-8。
5. **布尔运算稳健性**：布尔操作统一带 FuzzyValue 容差重试（`cavity_cutter.bool_cut`/`bool_common`），`run_parallel=False` 更稳定。
6. **分模模式**：`contour`（沿产品投影轮廓）是推荐模式，Web 前端默认；`max_z` 一刀切最简单。
7. **体积校验**：挖腔后校验"模芯-型腔模芯"是否等于产品体积（偏差>5% 报错），排查产品是否在模芯内。
