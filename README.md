# voloop

`voloop` 是一个面向微控制器的轻量级数字电源控制算法库。项目以源码形式发布，用户应将 `core/Inc` 和 `core/Src` 集成到自己的工程中，并随应用一起编译。核心算法使用 C99 编写，不依赖具体 MCU、HAL、板级外设或操作系统，适合嵌入到 STM32CubeMX、裸机工程、RTOS 工程或其他 C/CMake 项目中。

## 特性

- 平台无关：核心代码只依赖 C 标准库和 `math.h`。
- 源码集成：直接把头文件和 `.c` 源文件加入用户工程，不要求使用预编译库。
- 模块化 API：可包含聚合头文件 `voloop.h`，也可按需包含单个模块头文件。
- 固定采样周期设计：PID、QPR、FOF、NCO、PLL 等运行函数适合放在定时器中断或控制环任务中周期调用。
- 电源控制场景支持：提供 Buck 级联环控制和离网逆变器电压控制模块。
- 可生成 Doxygen 文档：仓库包含 `Doxyfile` 和基础文档入口。

## 模块概览

| 模块 | 头文件 | 说明 |
| --- | --- | --- |
| Common Definitions | `voloop_def.h` | 状态码、数学常量、限幅函数、Q1.31 相位转换、正弦/余弦查表接口和 PWM 状态定义。 |
| PID Controller | `voloop_pid.h` | PID 控制器，支持离散增益、连续参数、单零点和双零点形式初始化，并提供条件积分和反算抗积分饱和。 |
| First-Order Filter | `voloop_fof.h` | 一阶离散滤波器，支持离散系数、连续一阶形式、低通、高通和超前滞后补偿器初始化。 |
| QPR Controller | `voloop_qpr.h` | 准比例谐振/比例谐振控制器，适用于周期参考或周期扰动抑制。 |
| NCO | `voloop_nco.h` | 数控振荡器，维护 Q1.31 相位累加器并输出相位、正弦和余弦。 |
| PLL | `voloop_pll.h` | 单相锁相环，组合 PID 环路滤波器和 NCO，用于估计输入信号相位与频率。 |
| Buck Converter | `voloop_buck.h` | Buck 变换器控制，组合输出电压外环和电感电流内环，输出 PWM 占空比建议。 |
| Off-Grid Inverter | `voloop_offinv.h` | 离网逆变器控制，组合 QPR 电压环和 NCO，输出全桥两桥臂 PWM 指令。 |

## 目录结构

```text
.
├── core
│   ├── Inc        # 公共头文件
│   └── Src        # 模块实现
├── tests
│   └── link_smoke # CI 链接 smoke 测试
├── docs           # 文档入口
├── CMakeLists.txt
├── Doxyfile
└── README.md
```

## 快速开始

### 作为 CMake 源码子项目使用

将本仓库加入你的工程后，在上层 `CMakeLists.txt` 中添加：

```cmake
add_subdirectory(path/to/voloop)

target_link_libraries(your_target PRIVATE voloop)
```

这里的 `voloop` 是由源码在你的工程构建过程中生成的 CMake target，不是需要单独下载的预编译库。

然后在 C 代码中包含聚合头文件：

```c
#include "voloop.h"
```

如果只需要某个模块，也可以直接包含单独头文件，例如：

```c
#include "voloop_pid.h"
#include "voloop_qpr.h"
```

### 手动集成到非 CMake 工程

对于 STM32CubeMX、Keil、IAR 或其他非 CMake 工程，可以按源码方式集成：

1. 将 `core/Inc` 添加到编译器 include path。
2. 将 `core/Src` 下需要的 `.c` 文件加入工程源文件列表。
3. 在应用代码中包含 `voloop.h` 或具体模块头文件。
4. 如果使用默认正弦/余弦查表实现，需要同时加入 `core/Src/voloop_def.c` 和 `core/Src/voloop_sin_table_1024.inc`。

当前源码文件包括：

```text
core/Src/voloop_buck.c
core/Src/voloop_def.c
core/Src/voloop_fof.c
core/Src/voloop_nco.c
core/Src/voloop_offinv.c
core/Src/voloop_pid.c
core/Src/voloop_pll.c
core/Src/voloop_qpr.c
```

### 验证源码构建

仓库提供 CMake 配置，主要用于验证源码能正常编译。默认不构建测试；需要运行测试时开启 `VOLOOP_BUILD_TESTS`：

```sh
cmake -S . -B build -DVOLOOP_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

当前测试 `tests/link_smoke` 用于验证头文件和源码能正常编译链接。

## 基本使用示例

### PID 控制器

```c
#include "voloop_pid.h"

PID_HandleTypeDef pid;
PID_InitTypeDef init = {0};

init.mode = PID_Discrete;
init.init.Discrete.KpDiscrete = 1.0f;
init.init.Discrete.KiDiscrete = 0.01f;
init.init.Discrete.KdDiscrete = 0.0f;

if (VOLOOP_PID_Init(&pid, &init) == VOLOOP_OK) {
    float duty = VOLOOP_PID_ComputeConditional(&pid,
                                               12.0f,  /* setpoint */
                                               10.5f,  /* measurement */
                                               0.0f,
                                               0.9f);
    (void)duty;
}
```

### NCO 相位发生器

```c
#include "voloop_nco.h"

NCO_HandleTypeDef nco;
NCO_InitTypeDef init = {
    .triggerFrequency = 10000U,
    .initialFrequency = 50.0f,
    .initialRad = 0.0f,
};

if (VOLOOP_NCO_Init(&nco, &init) == VOLOOP_OK) {
    VOLOOP_NCO_Start(&nco);

    /* 在固定频率中断或控制任务中周期调用 */
    VOLOOP_NCO_Sync(&nco);

    float sine = VOLOOP_NCO_GetSine(&nco);
    float cosine = VOLOOP_NCO_GetCosine(&nco);
    (void)sine;
    (void)cosine;
}
```

## 嵌入式集成建议

- 控制器的 `Sync` / `Compute` 函数应在固定采样周期内调用。
- 初始化参数中的 `triggerFrequency` 使用 Hz。
- PID 零点配置、FOF 超前滞后配置、QPR 谐振频率和截止频率均使用 Hz，而不是 rad/s。
- Buck 和 OffInv 模块只生成控制建议，不直接访问 ADC 或 PWM 外设；采样、保护硬件动作和 PWM 写寄存器应由板级代码完成。
- `voloop_def.h` 中的 `VOLOOP_DEF_SIN`、`VOLOOP_DEF_COS`、`VOLOOP_DEF_PRINTF` 可以在包含头文件前重定义，以接入项目自己的数学库、CORDIC、DSP 库或日志后端。

## 文档

仓库包含 Doxygen 配置文件：

```sh
doxygen Doxyfile
```

生成的 API 文档可从 `docs/index.html` 进入。

## 许可证

本项目使用 Apache License 2.0，详见 `LICENSE`。
