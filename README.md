# 管件硬模成型 · 模具自动生成系统

输入一个**管件产品数模**（STEP / IGES / STL），自动生成硬模成型模具的四个零件，并可下载：

| 输出 | 说明 |
|---|---|
| `{编号}_upper_mold.step` | **上模**：型腔内壁 = 管件外壁 |
| `{编号}_lower_mold.step` | **下模**：与上模在同一分模面处切开 |
| `{编号}_core_pin.step` | **芯棒**：内孔全长，端面与管件端口齐平 |
| `{编号}_mold4_assembly.step` | **四件套装配体**：上模 + 产品 + 芯棒 + 下模（合模状态） |
| `{编号}_metadata.json` | 分模高度 / 各件体积 / 耗时 / 警告（供核对） |

另外提供 **镜像 / 旋转**（模型方向不合适时先变换再开模）。

> 本系统已按需求**精简**：只保留管件硬模成型这一条管线，**没有三维查看**（不加载 three.js、
> 没有剖切/测量/特征识别等面板）。板件折弯管线、旧版修复器、独立芯棒设计模块已从仓库移除
> （本机留在 `_legacy/`，历史版本仍在 git 里）。
> 管件模具的技术细节、历次优化与实测数据见 [`PIPE_MOLD_README.md`](PIPE_MOLD_README.md)。

---

## 一、运行方式

### Web（推荐）
```bat
:: 双击项目根目录的 启动Web服务.bat，或：
venv\Scripts\python.exe web_box_generator\app.py --port 5002
```
浏览器打开 <http://127.0.0.1:5002>，页面上三步：
1. **上传产品数模** → 自动识别管件编号（用作输出前缀）
2. **生成管件模具** → 填分模面高度（可点「取推荐值」），点「开始生成」→ 进度日志 → 四个下载按钮
3. **镜像 / 旋转** → 生成一个新的 STEP 供下载

### 命令行
```bat
venv\Scripts\python.exe main.py --part <产品.stp> [--parting-z 40] [--prefix 24TK_1324-31] ^
    [--parting-surface plane|silhouette] [--margin 20 20 20] [--no-verify] [--no-core] [--parallel]
```
`--part` 必填（不再有内置测试产品）。`--no-verify` 跳过开模/顶出干涉校验，快几分钟。

### 依赖
`venv/`（Windows 虚拟环境）：`cadquery-ocp`（提供 `OCP`，OCCT 7.8）、`cadquery`、`flask`、`numpy`。

> **中文路径**：OCC 底层文件 API 不支持非 ASCII 路径。所有读写统一走 `io_utils.py`
> （读前复制到 ASCII 临时目录、写后由 Python 移回），**新增直接调用 OCC 读写文件的代码必须复用它**。

---

## 二、系统怎么生成的（一句话版）

1. 读产品 → 摆正到**模芯块**正中（管件长轴沿 X）；
2. 型腔 = 产品**外形包络**（把内孔填实），所以型腔内壁与管件外壁严丝合缝；
3. 填实内孔的那块料正好就是**芯棒**（挖包络顺带切出来，不额外花时间）；
4. 按**分模面**把型腔模切成上模 / 下模：
   * **水平面**（默认，最好加工）：高度可由用户指定，也可用自动推荐值；
   * **侧影随形面**：沿产品侧影线（曲面，更随形，加工麻烦）；
5. 导出上模 / 下模 / 芯棒 / 四件套（都已在**输入数模的原始坐标系**里，可直接和原产品装配）。

**分模面高度怎么定**：开模方向是 +Z，分型线要落在产品表面法向水平处。程序用表面采样
（不跑布尔，几十秒）算出"安全区间"和推荐高度；区间上下限颠倒时说明该件**任何单一水平面**
都会在部分站位留台阶（斜切端面 / 局部倒扣），此时可以自己填一个高度，或改用侧影随形面。

---

## 三、Web 接口

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/` | 页面 |
| POST | `/api/upload` | 上传数模（`file`），返回 `{file, part_number, size_mb}` |
| POST | `/api/mold_generate` | 生成模具（异步）→ `{task_id}` |
| POST | `/api/pipe_plane_suggest` | 只算推荐分模面高度（异步）→ `{task_id}` |
| GET | `/api/task_status?id=` | 轮询：`{state, log, result, error}` |
| GET | `/api/download?task=&file=` | 下载生成件（只允许该任务自己的输出目录） |
| POST | `/api/generate_mirror` | 镜像 / 旋转（同步）→ `{name, url, ...}` |
| GET | `/api/download_mirror?name=` | 下载镜像 / 旋转结果 |

生成参数（`/api/mold_generate` 的 JSON）：

```json
{ "file": "上传后的存储名",
  "prefix": "24TK_1324-31",
  "parting_surface": "plane",        // plane=水平面 / silhouette=侧影随形
  "parting_z": 40.0,                 // 留空或 null = 自动推荐
  "margin": [20, 20, 20],
  "verify": false,                   // true = 做开模/顶出干涉校验（慢几分钟）
  "with_core": true }                // 是否出芯棒 + 四件套
```

---

## 四、文件清单

| 文件 | 作用 |
|---|---|
| `main.py` | 管线入口：`generate_mold()` / `suggest_pipe_plane()` / 命令行 |
| `pipe_mold.py` | **核心**：型腔包络、分模（水平面/侧影随形）、芯棒提取、开模校验 |
| `core_block_generator.py` | 按产品包围盒 + 余量算模芯块尺寸 |
| `part_aligner.py` | 产品在模芯里的居中定位 |
| `cavity_cutter.py` | 布尔运算封装（带 FuzzyValue 容错重试） |
| `result_exporter.py` | 导出 STEP（AP214，含 schema 降级）+ JSON 元数据 |
| `model_reader.py` / `mesh_solid.py` | 读 STEP / IGES / STL（STL 先缝合网格成实体） |
| `mirror_generator.py` | 镜像（xOy/xOz/yOz，可过质心）/ 绕轴旋转（顺逆时针） |
| `naming_utils.py` | 从文件名提取管件编号作为输出前缀 |
| `geometry_utils.py` / `errors.py` / `io_utils.py` | 体积/包围盒、异常类型、中文路径与 OCC 的安全读写 |
| `web_box_generator/app.py` | 极简 Flask 服务（上传 / 生成 / 下载 / 镜像旋转） |
| `web_box_generator/templates/index.html` | 单文件页面（无三方前端库） |
| `启动Web服务.bat` | 一键启动（端口 5002） |
| `PIPE_MOLD_README.md` | 管件模具的技术说明：算法、OCCT 坑、历次优化与实测数据 |
| `24TK_*.stp` / `r0.STL` | 测试用管件数模 |

---

## 五、已知限制（如实说明）

1. **管壁有侧向开口**（三通口、侧窗）时，内孔腔从洞里与主体相连，两端截断在原理上切不断 —
   这种结构需要**侧向抽芯（斜顶/滑块）**，本程序不生成侧抽芯，只会在 `metadata.mold_warnings`
   里明确提示。
2. **斜切端面 / 局部倒扣 + 水平面**：任何单一水平面都会在部分站位留下台阶或钩料（几何决定的），
   要完全随形请用「侧影随形」分模面。
3. **包围盒虚胖**：`Bnd_Box` 对 BSpline 面会撑大（`24TK_1324-31` 实测虚胖 2×25 mm），
   影响模芯尺寸与上报的"产品尺寸"（多约 7% 材料）；型腔贴合、分模、芯棒都不受影响。
4. **OCC 的 STEP 写出器**对个别 BSpline 件会轻微重逼近（0.001 mm 量级），交付文件自洽。
