# Frequency Response Tool

用于以黑盒方式测量 voloop 控制模块的离散频率响应，并生成原始数据、处理后数据和 Bode 图。

工具通过公共初始化、复位和计算接口驱动被测模块，不读取模块内部状态，也不修改核心算法。

## 工作流

```text
generate_frequencies.py
    生成频点并写入测试配置
            ↓
config.json
    保存测试频点、模块参数和测量参数
            ↓
run_c_runner.py
    物化 frequencies.txt 并调用 C runner
            ↓
raw CSV
            ↓
plot_response.py
    生成 processed CSV、PNG 和 SVG
```

单片机扫频输出使用一条额外的转换路径：

```text
MCU CSV（同步分量、正交分量、数字状态码）
            ↓
convert_mcu_response.py
    补充 module、mode、gain_linear、gain_db 和 phase_deg
            ↓
response CSV
            ↓
plot_response.py
    生成 processed CSV、PNG 和 SVG
```

`config.json` 是一次测试的源规格。`frequencies.txt`、CSV 和图片均为可重新生成的产物。

## 环境准备

在仓库根目录构建 C runner：

```bash
cmake --preset host-tools
cmake --build --preset host-tools
```

安装 Python 依赖：

```bash
cd tools/freq_response
uv sync
```

## 使用方法

以下命令均在 `tools/freq_response` 目录执行。

### 1. 配置频点

生成对数分布频点，并合并到已有配置：

```bash
uv run python generate_frequencies.py \
  --config examples/pid_one_zero_pi.json \
  --start-hz 10 \
  --stop-hz 200 \
  --points-per-decade 20
```

默认保留已有频点，合并后统一去重并升序排列。

整体替换原频点：

```bash
uv run python generate_frequencies.py \
  --config examples/pid_one_zero_pi.json \
  --start-hz 10 \
  --stop-hz 200 \
  --points-per-decade 20 \
  --replace
```

添加指定频点：

```bash
uv run python generate_frequencies.py \
  --config examples/qpr_non_ideal_50hz.json \
  --start-hz 10 \
  --stop-hz 200 \
  --points-per-decade 10 \
  --include-hz 45 \
  --include-hz 50 \
  --include-hz 55
```

使用 `--dry-run` 可预览结果而不修改配置。

### 2. 运行频响测量

```bash
uv run python run_c_runner.py \
  --config examples/pid_one_zero_pi.json
```

该命令会：

1. 读取 `frequencies.values_hz`；
2. 生成配置指定的 `frequencies.txt`；
3. 调用已经构建的 C runner；
4. 输出原始 CSV。

`run_c_runner.py` 不负责配置或构建 CMake 项目。

预览 C runner 命令：

```bash
uv run python run_c_runner.py \
  --config examples/pid_one_zero_pi.json \
  --dry-run
```

### 3. 转换单片机扫频 CSV

单片机输出的 CSV 需要包含以下列：

```text
frequency_hz,synchronous_component,quadrature_component,status
```

例如，将 `cache/1.csv` 转换为可供绘图工具读取的响应 CSV：

```bash
uv run python convert_mcu_response.py \
  --input ../../../cache/1.csv
```

默认不会覆盖输入文件，而是在输入文件旁生成 `1_response.csv`。输出列为：

```text
module,mode,frequency_hz,synchronous_component,quadrature_component,gain_linear,gain_db,phase_deg,status
```

其中 `module` 固定为 `power_path`，`mode` 使用输入文件名且不含扩展名。计算规则与当前单片机固件一致：

```text
amplitude   = hypot(synchronous_component, quadrature_component)
gain_linear = max(amplitude / input_amplitude, gain_floor)
gain_db     = 20 * log10(gain_linear)
phase_deg   = degrees(atan2(quadrature_component, synchronous_component))
```

默认 `input_amplitude=0.1`、`gain_floor=1e-12`；扫描配置不同时可覆盖：

```bash
uv run python convert_mcu_response.py \
  --input ../../../cache/1.csv \
  --output ../../../cache/power_path_1.csv \
  --input-amplitude 0.05 \
  --gain-floor 1e-10
```

状态码 `0` 和 `1` 分别转换为 `ok` 和 `ok_with_gain_floor`。其他状态行仍会保留，但派生列留空，因此不会进入 Bode 图。

### 4. 生成 Bode 图

```bash
uv run python plot_response.py \
  --input ../../../build/tools/freq_response/pid_one_zero_pi_raw.csv \
  --out-dir ../../../build/tools/freq_response/plots \
  --name pid_one_zero_pi
```

输出：

```text
pid_one_zero_pi_processed.csv
pid_one_zero_pi_bode.png
pid_one_zero_pi_bode.svg
```

图中的标记为真实测量点，连线仅用于辅助阅读。需要观察窄带峰值时，应在配置中增加实际测试频点。

## 配置要点

主要字段：

```text
frequencies.values_hz
    最终测试频点列表

runner.executable
    C runner 可执行文件

artifacts.frequencies_txt
    物化后的频点文件

artifacts.raw_csv
    原始测量结果

measurement
    通用采样和测量参数

params
    被测模块参数
```

配置内的相对路径均相对于配置文件所在目录解析。

## Python API

四个 Python 文件均可直接执行，也可作为模块导入：

```python
from convert_mcu_response import convert_mcu_response
from generate_frequencies import generate_log_frequencies
from run_c_runner import run_from_config
from plot_response import render_response
```

模块接口通过返回值和异常报告结果，不依赖 CLI 输出。

## 边界

* C runner 只接受频点文本文件，JSON 到文本文件的转换由 Python 完成。
* 工具只测量已实现的公共接口行为，不推导连续域理论模型。
* 单片机原始 CSV 不会被转换器覆盖；无效测量行会保留以便追溯。
* 无效状态行保留在原始和处理后 CSV 中，但不进入 Bode 图。
* 频点必须为有限正数，并低于当前测量配置允许的有效范围。
* 工具不修改 `core`，也不负责自动设计控制器参数。
