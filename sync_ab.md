# ICM 六轴 IMU 驱动 —— 使用指南

> 面向使用者的完整手册：从文件结构、移植接线，到 API 使用与调试排查。
> 驱动当前默认目标器件为 **InvenSense / TDK ICM-42607-C**，前缀统一为 `icm_` / `ICM_`，为将来兼容 ICM-42688 等同宗器件预留。

---

## 目录

- [一、这个驱动能做什么](#一这个驱动能做什么)
- [二、文件结构与分层](#二文件结构与分层)
- [三、三步跑通（快速上手）](#三三步跑通快速上手)
- [四、移植：实现 HAL 的 5 个函数](#四移植实现-hal-的-5-个函数)
- [五、配置：改哪些参数](#五配置改哪些参数)
- [六、API 详解](#六api-详解)
- [七、典型使用流程](#七典型使用流程)
- [八、SPI 硬件约定](#八spi-硬件约定)
- [九、常见问题排查](#九常见问题排查)
- [十、FAQ](#十faq)

---

## 一、这个驱动能做什么

| 能力 | 说明 |
|---|---|
| **器件控制** | 初始化、软复位、WHO_AM_I 校验、量程/ODR 配置、低噪声模式使能 |
| **数据采集** | 一次 SPI 突发读回 温度 + 三轴加速度 + 三轴角速度（14 字节，时间戳对齐） |
| **单位换算** | 原始 16bit 值 → 物理量（°C / g / °·s⁻¹），分辨率按量程自动推导 |
| **姿态解算** | 四元数积分 + Mahony 倾角校正，输出 roll / pitch / yaw |
| **惯性导航** | 重力补偿 → 线加速度 → 积分出速度、位置，带 ZUPT 静止归零抑制漂移 |
| **零偏标定** | 静止采样求平均，自动扣除陀螺零偏 |

**设计约束**：C99、无动态内存、无 RTOS 强依赖、驱动核心与平台完全解耦。

**一个重要的前提认知**：本驱动只用加速度计 + 陀螺仪，**没有磁力计**。因此：

- `roll` / `pitch` 有加速度计做绝对参考，长时间稳定不漂移
- `yaw` 只能靠陀螺积分，**会缓慢漂移**，这是六轴 IMU 的物理限制，不是 bug

---

## 二、文件结构与分层

```
icm.h              统一入口 —— 业务代码只需 #include 这一个
│
├── icm_cfg.h      配置层：用户可改参数 / 错误码 / 芯片选择宏
├── icm_regs.h     规格层：寄存器地址 / 位域 / 器件常数（通常不改）
├── icm_hal.h      HAL 接口声明
├── icm_ctrl.h     控制层：状态机 / 数据结构 / 器件控制 API
└── icm_integ.h    算法层：姿态与惯性导航 API

对应实现文件：
icm_hal.c          【移植时唯一需要修改的文件】HAL 模板 + User Code 区段
icm_ctrl.c         控制层实现（寄存器读写、状态机）
icm_integ.c        算法层实现（四元数、Mahony、ZUPT）
```

### 依赖关系（严格单向，无环）

```
icm_cfg.h    → 不依赖任何层
icm_regs.h   → 不依赖任何层
icm_hal.h    → icm_cfg.h
icm_ctrl.h   → icm_cfg.h, icm_regs.h, icm_hal.h
icm_integ.h  → icm_ctrl.h          （故意不依赖 icm_regs.h）
icm.h        → 以上全部
```

### 各层职责一句话

| 文件 | 职责 | 你需要改吗 |
|---|---|---|
| `icm_cfg.h` | 纯配置：量程、ODR、功能开关、时序、芯片型号 | **调参时改** |
| `icm_regs.h` | 器件规格：寄存器地址与位域 | 基本不改 |
| `icm_hal.h` | HAL 接口契约 | 不改，只读 |
| `icm_hal.c` | 平台实现（SPI、延时、滴答） | **移植时改** |
| `icm_ctrl.h/.c` | 器件控制逻辑 | 不改 |
| `icm_integ.h/.c` | 姿态解算逻辑 | 不改 |
| `icm.h` | 聚合入口 | 不改，只包含 |

---

## 三、三步跑通（快速上手）

### 第 1 步：包含头文件

```c
#include "icm.h"        /* 只需这一个，不要直接包含底层头文件 */
```

### 第 2 步：初始化 + 标定

```c
icm_raw_t        raw;      /* 原始数据 */
icm_scaled_t     s;        /* 物理量 */
icm_estimator_t  est;      /* 解算上下文（约 80 字节） */

if (icm_init() != ICM_OK) {
    /* 初始化失败：查 icm_get_state()，通常进 ERROR 状态 */
    return -1;
}

if (icm_enable_sensors() != ICM_OK) {
    /* 使能失败：同样是通信或状态问题 */
    return -1;
}

icm_est_init(&est);                     /* 复位估计器 */
/* 静止、水平放置，采集 256 个样本标定陀螺零偏 */
icm_est_capture_gyro_bias(&est, 256);
```

> **标定必须在器件完全静止且水平时进行**，否则零偏会标歪，导致姿态持续漂移。

### 第 3 步：循环采样

```c
while (1) {
    /* 推荐等待数据就绪（比固定延时更准、更省 CPU） */
    if (icm_wait_data_ready(ICM_DRDY_TIMEOUT_MS) != ICM_OK) {
        continue;                       /* 超时跳过本帧 */
    }

    icm_read_all(&raw);                 /* 一次 SPI 突发读 14 字节 */
    icm_convert(&raw, &s);              /* 原始值 → 物理量 */

    icm_est_update_auto(&est, &s);      /* 用系统滴答自动算 dt 并解算 */

    float eul[3], pos[3];
    icm_est_get_euler(&est, eul);       /* roll / pitch / yaw (rad) */
    icm_est_get_position(&est, pos);    /* 世界系位置 (m) */
    /* ... 使用 eul / pos ... */
}
```

**跑通判定**：静止水平放置时，`s.accel_g[2]` 应约等于 `1.0`（重力），`eul[0]` / `eul[1]` 应收敛到 `0` 附近。

---

## 四、移植：实现 HAL 的 5 个函数

驱动核心（`cfg` / `regs` / `ctrl` / `integ`）**完全平台无关，一行都不用改**。移植只需在 `icm_hal.c` 的 **User Code 区段**内填写平台代码。

### 4.1 User Code 区段机制

`icm_hal.c` 是模板文件，每个函数体内恰好有一个区段，区段名 == 函数名：

```c
icm_err_t icm_hal_init(void)
{
    /*< User Code >[icm_hal_init]*/
    /* 在这里填你的平台代码 */
    /*< Code end >[icm_hal_init]*/
}
```

| 区段名 | 用途 |
|---|---|
| `[includes]` | 文件级：放平台头文件、宏、静态变量 |
| `[icm_hal_init]` | 初始化 SPI 外设、CS 置高 |
| `[icm_hal_spi_xfer]` | 全双工 SPI 传输（自动管理 CS） |
| `[icm_hal_delay_us]` | 微秒延时 |
| `[icm_hal_delay_ms]` | 毫秒延时 |
| `[icm_hal_get_tick_ms]` | 系统毫秒滴答 |

> **区段外的内容由驱动维护者管理**。配合同步脚本升级驱动时，区段内的用户代码会按区段名一一保留，不会被覆盖。

### 4.2 五个函数的要求

#### `icm_hal_init()`

初始化 SPI 外设，并把 CS 置为无效（高电平，避免上电期间总线误判）。

```c
/* Renesas RA / FSP 示例 */
R_BSP_PinWrite(ICM_CS_PIN, BSP_IO_LEVEL_HIGH);
err = R_SPI_Open(&g_spi0_ctrl, &g_spi0_cfg);
if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err)) {
    return ICM_ERR_HAL;
}
return ICM_OK;
```

若外设已由别处打开，返回 `ICM_OK` 即可（视为已就绪）。

#### `icm_hal_spi_xfer()` —— 唯一有复杂度的函数

```c
icm_err_t icm_hal_spi_xfer(uint8_t *tx, uint8_t *rx, uint16_t len);
```

**核心契约：进入拉低 CS → 传输 len 字节 → 退出拉高 CS。**

| 要求 | 说明 |
|---|---|
| `len == 0` | 直接返回 `ICM_OK`，不碰硬件 |
| `tx == NULL` | 返回 `ICM_ERR_PARAM` |
| `rx` | 可为 NULL（只发不收） |
| CS 管理 | 由本函数**独占**，业务层完全不碰 CS |
| 全双工 | 发送 `tx[i]` 的同时把总线收到的字节存入 `rx[i]` |
| 错误路径 | **所有 return 路径都必须保证 CS 为高**（包括每个失败分支） |
| 超时 | 返回 `ICM_ERR_TIMEOUT`，且 CS 已拉高，不能挂死总线 |

**帧格式**（协议由 `icm_ctrl.c` 组装，你只需按字节搬运）：

```
tx[0] = bit7(读标志 0x80) | bit6:0(寄存器地址)
       bit7=1 → 读，rx[1..len-1] 为数据
       bit7=0 → 写，tx[1] 为待写值

rx[0] = dummy（发送命令期间收到的字节，丢弃）
rx[1..len-1] = 对应寄存器数据（大端，MSB 先）
```

**Renesas RA / FSP 参考实现**（`R_SPI_WriteRead` 是异步的，必须等回调）：

```c
/* [includes] 区段 */
static volatile bool s_icm_spi_done = false;

static void icm_spi_callback(spi_callback_args_t *p_args)
{
    if ((NULL != p_args) && (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)) {
        s_icm_spi_done = true;
    }
}

/* [icm_hal_spi_xfer] 区段 */
icm_err_t icm_hal_spi_xfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    fsp_err_t err;
    uint32_t  timeout_ms;

    if (0U == len) { return ICM_OK; }
    if (NULL == tx) { return ICM_ERR_PARAM; }

    R_BSP_PinWrite(ICM_CS_PIN, BSP_IO_LEVEL_LOW);
    s_icm_spi_done = false;              /* 必须在启动传输之前清标志 */

    err = R_SPI_WriteRead(&g_spi0_ctrl, (void const *)tx, (void *)rx,
                          (uint32_t)len, SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err) {
        R_BSP_PinWrite(ICM_CS_PIN, BSP_IO_LEVEL_HIGH);   /* 失败也要放 CS */
        return ICM_ERR_HAL;
    }

    timeout_ms = ICM_SPI_TIMEOUT_MS;
    while ((!s_icm_spi_done) && (0U != timeout_ms)) {
        icm_hal_delay_ms(1U);
        timeout_ms--;
    }
    R_BSP_PinWrite(ICM_CS_PIN, BSP_IO_LEVEL_HIGH);

    return (s_icm_spi_done) ? ICM_OK : ICM_ERR_TIMEOUT;
}
```

**Renesas RX 骨架**（多为「发一个收一个」，用循环）：

```c
CS_LOW();
for (uint16_t i = 0U; i < len; i++) {
    /* TODO: 发送 tx[i]，等待发送完成 */
    /* TODO: 等待接收完成，若 rx != NULL 则 rx[i] = 收到的字节 */
}
while (SPI_BUSY) { }        /* 等最后一位时钟走完 */
CS_HIGH();
return ICM_OK;
```

#### `icm_hal_delay_us()` / `icm_hal_delay_ms()`

用于 `OFF→LN` 切换后的 200 µs 禁写窗口、软复位后等待、陀螺稳定等待等。
**微秒延时的精度要求在 ±50% 以内**（过短会违反手册时序）。

#### `icm_hal_get_tick_ms()`

返回系统毫秒滴答，供 `icm_est_update_auto()` 自动计算 `dt`。

> **无滴答平台**：返回 `0` 即可。此时 `icm_est_update_auto()` 会返回 `ICM_ERR_HAL`，请改用 `icm_est_update(e, s, dt)` 由调用方显式传入 `dt`。

### 4.3 编译期不再需要任何实现宏

重构后的 `icm_hal.c` **不含** `#if defined(ICM_HAL_IMPL_RA_FSP)` 之类的条件编译分支。平台差异全部在 User Code 区段内消化，一份文件对应一个平台。

---

## 五、配置：改哪些参数

全部集中在 **`icm_cfg.h`**，与平台无关。所有参数都可用 `#ifndef` 保护，因此既能在此文件改，也能从编译命令行 `-D` 覆盖。

### 5.1 芯片选择

```c
#define ICM_CHIP_ID            (ICM_CHIP_42607)  /* 目标芯片型号 */
#define ICM_WHO_AM_I_VALUE     (0x61U)           /* -C 版 0x61；非 -C 版 0x60 */
```

> 换用非 -C 料号（ICM-42607 非 C 版）时，把 `ICM_WHO_AM_I_VALUE` 改为 `0x60`。

### 5.2 量程（最常改）

```c
#define ICM_ACCEL_FS_G         (16)     /* 加速度量程：2 / 4 / 8 / 16 (g) */
#define ICM_GYRO_FS_DPS        (2000)   /* 陀螺量程：250 / 500 / 1000 / 2000 (dps) */
```

分辨率自动推导，无需手填：

```c
ICM_ACCEL_RES_G   =  ICM_ACCEL_FS_G / 32768.0
ICM_GYRO_RES_DPS  =  ICM_GYRO_FS_DPS / 32768.0
```

**量程选择权衡**：量程越大，抗饱和能力越强，但分辨率越低。静态应用选小量程（2g / 250dps）精度更高；运动剧烈选大量程。

### 5.3 采样率（ODR）

```c
#define ICM_GYRO_ODR           (ICM_ODR_800HZ)   /* 陀螺 ODR */
#define ICM_ACCEL_ODR          (ICM_ODR_800HZ)   /* 加速度 ODR，可与陀螺不同 */
```

可选值：`1.6k / 800 / 400 / 200 / 100 / 50 / 25 / 12.5` Hz。

### 5.4 时序常数

```c
ICM_SOFT_RESET_MS          软复位后等待（默认 10 ms）
ICM_PWR_ON_SETTLE_US       OFF→LN 后禁写窗口（默认 200 µs，手册要求）
ICM_GYRO_RDY_MS            陀螺上电稳定等待（默认 50 ms）
ICM_MCLK_RDY_TIMEOUT_MS    轮询 MCLK_RDY 超时（默认 100 ms）
ICM_DRDY_TIMEOUT_MS        轮询数据就绪超时（默认 100 ms）
ICM_SPI_TIMEOUT_MS         SPI 传输超时（默认 100 ms，HAL 内部使用）
```

### 5.5 解算功能开关（体积/性能裁剪）

```c
ICM_INTEG_ENABLE         (1)   总开关，0 = 裁掉整个解算模块（最省 Flash）
ICM_EST_TILT_CORRECT     (1)   加速度计倾角校正（Mahony），抑制 roll/pitch 漂移
ICM_EST_GRAVITY_COMP     (1)   扣除重力，得到线加速度
ICM_EST_VEL_ENABLE       (1)   积分出速度
ICM_EST_POS_ENABLE       (1)   积分出位置（纯惯性易漂移，可单独关）
ICM_EST_ZUPT_ENABLE      (1)   静止检测归零速度，显著抑制漂移
```

### 5.6 算法增益与阈值

```c
ICM_EST_KP              (0.2f)    倾角校正比例增益，0.1~0.3 常用
ICM_EST_KI              (0.0f)    积分增益，>0 可消除缓慢零偏（耗 RAM/CPU）
ICM_GRAVITY             (9.80665f)
ICM_ZUPT_ACC_DEV        (0.08f)   静止判定：|a|-1g 偏差阈值 (g)
ICM_ZUPT_GYRO_DEV       (0.05f)   静止判定：角速度阈值 (rad/s)
ICM_VEL_MAX             (20.0f)   速度限幅 (m/s)，防积分爆炸
ICM_DT_MAX              (0.2f)    单帧最大 dt (s)，超时/卡顿丢弃本帧防跳变
ICM_BIAS_SAMPLES        (256U)    零偏标定默认采样数
```

### 5.7 行为裁剪

```c
ICM_USE_BURST_READ      (1)   单次 14 字节突发读（推荐，保证时间戳对齐）
ICM_KEEP_SEPARATE_READ  (1)   保留分轴读取接口（兼容旧代码，新工程不推荐）
```

### 5.8 命令行覆盖示例

```bash
gcc -std=c99 -I. \
    -DICM_ACCEL_FS_G=8 -DICM_GYRO_FS_DPS=1000 \
    -DICM_EST_POS_ENABLE=0 \
    main.c icm_hal.c icm_ctrl.c icm_integ.c -lm -o app
```

---

## 六、API 详解

### 6.1 器件控制（`icm_ctrl.h`）

#### `icm_err_t icm_init(void)`

初始化 HAL → 软复位 → 校验 WHO_AM_I → (可选)等待 MCLK 就绪。

- 成功后状态机进入 `RESET`，**必须再调用 `icm_enable_sensors()` 才能采样**
- 失败会进入 `ERROR` 状态，需重新 `icm_init()`

#### `icm_err_t icm_enable_sensors(void)`

使能加速度计 + 陀螺仪（低噪声 / LN 模式），量程与 ODR 取自 `icm_cfg.h`。

内部顺序遵循手册：先配 `CONFIG0` → 写 `PWR_MGMT0` 切 LN → 等 200 µs（禁写窗口）→ 等陀螺稳定。成功后进入 `ACTIVE`。

#### `icm_err_t icm_soft_reset(void)`
#### `icm_err_t icm_check_who_am_i(void)`

软复位 / 单独校验器件 ID（调试用）。

#### `icm_err_t icm_read_reg(uint8_t reg, uint8_t *p_val)`
#### `icm_err_t icm_write_reg(uint8_t reg, uint8_t val)`

单寄存器读写。**有意不做状态机校验**，便于在任何状态下调试。

#### `icm_err_t icm_wait_data_ready(uint32_t timeout_ms)`

轮询 `INT_STATUS_DRDY` 的 `DATA_RDY_INT`(bit0)，超时返回 `ICM_ERR_TIMEOUT`。
比固定延时更准、更省 CPU。无中断线时也可直接调用 `icm_read_all()`。

#### `icm_err_t icm_read_all(icm_raw_t *raw)`

**一次 SPI 突发读取** 温度(2) + 加速度(6) + 陀螺(6) = 14 字节，保证三轴时间戳一致（对积分至关重要）。

```c
typedef struct {
    int16_t temp;        /* 温度原始值 */
    int16_t accel[3];    /* 加速度原始值 X/Y/Z */
    int16_t gyro[3];     /* 陀螺原始值 X/Y/Z */
} icm_raw_t;
```

#### `icm_err_t icm_convert(const icm_raw_t *raw, icm_scaled_t *out)`

原始值 → 物理量。分辨率按 `icm_cfg.h` 中的量程自动取用。

```c
typedef struct {
    float temp_c;        /* 温度 (°C) */
    float accel_g[3];    /* 加速度 (g) */
    float gyro_dps[3];   /* 角速度 (°/s) */
} icm_scaled_t;
```

#### 零偏标定三件套

```c
void         icm_set_gyro_bias(float x, float y, float z);   /* 单位 dps */
void         icm_get_gyro_bias(float *x, float *y, float *z);
icm_err_t    icm_capture_gyro_bias(uint16_t n);              /* 静止采样求平均 */
```

#### `icm_state_t icm_get_state(void)`

读状态机当前状态。

```c
typedef enum {
    ICM_STATE_UNINIT = 0,   /* 未初始化 */
    ICM_STATE_RESET,        /* 已完成软复位 + WHO_AM_I */
    ICM_STATE_IDLE,         /* 已上电未使能 */
    ICM_STATE_ACTIVE,       /* 已使能，可采样 */
    ICM_STATE_ERROR         /* 不可恢复错误，需重新 init */
} icm_state_t;
```

> 注意：`ERROR` 在枚举中排在最后，业务入口统一用「非 ERROR 且不低于某个状态」判定，请勿自行用 `<` 比较状态。

#### 分轴读取（`ICM_KEEP_SEPARATE_READ=1` 时可用）

```c
icm_err_t icm_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
icm_err_t icm_read_gyro (int16_t *gx, int16_t *gy, int16_t *gz);
icm_err_t icm_read_temp (int16_t *raw_temp);
```

⚠️ 不推荐新工程使用：三次独立读取会产生几毫秒相位差，对积分不利。用 `icm_read_all()` 替代。

### 6.2 姿态解算（`icm_integ.h`）

#### 初始化与复位

```c
void icm_est_init (icm_estimator_t *e);   /* q = 单位四元数，v/p = 0 */
void icm_est_reset(icm_estimator_t *e);
```

#### 零偏标定

```c
icm_err_t icm_est_capture_gyro_bias(icm_estimator_t *e, uint16_t n);
```

静止时采集 n 个样本求平均作为零偏，标定后无需再手动 `set_bias`。

#### 核心解算

```c
/* 显式传入 dt（用于无系统滴答的平台） */
icm_err_t icm_est_update(icm_estimator_t *e, const icm_scaled_t *s, float dt);

/* dt 由 HAL 系统滴答自动计算（推荐） */
icm_err_t icm_est_update_auto(icm_estimator_t *e, const icm_scaled_t *s);
```

`dt` 约束：`> 0` 且 `<= ICM_DT_MAX`。非法返回 `ICM_ERR_PARAM`，未初始化返回 `ICM_ERR_STATE`。

#### 取值接口（只读，不修改上下文）

```c
void icm_est_get_quat    (const icm_estimator_t *e, float q[4]);         /* 四元数 */
void icm_est_get_euler   (const icm_estimator_t *e, float euler_rad[3]); /* roll/pitch/yaw (rad) */
void icm_est_get_velocity(const icm_estimator_t *e, float v[3]);         /* 速度 (m/s) */
void icm_est_get_position(const icm_estimator_t *e, float p[3]);         /* 位置 (m) */
```

#### 解算上下文结构

```c
typedef struct {
    float    q[4];          /* 姿态四元数 [q0,q1,q2,q3]，标量在前 */
    float    euler[3];      /* roll / pitch / yaw (rad) */
    float    vel[3];        /* 世界系速度 (m/s) */
    float    pos[3];        /* 世界系位置 (m) */
    float    gyro_bias[3];  /* 陀螺零偏 (dps) */
    float    w_int[3];      /* Mahony 积分项 */
    uint32_t last_tick;     /* 上次更新滴答，用于自动 dt */
    uint8_t  inited;
    uint8_t  first_frame;
} icm_estimator_t;
```

每实例约 **80 字节 RAM**。

### 6.3 错误码（`icm_cfg.h`）

```c
typedef uint8_t icm_err_t;

ICM_OK            (0)   成功
ICM_ERR_HAL       (1)   底层 HAL 失败
ICM_ERR_COMM      (2)   SPI 链路错误
ICM_ERR_WHOAMI    (3)   WHO_AM_I 不匹配
ICM_ERR_PARAM     (4)   参数非法
ICM_ERR_STATE     (5)   状态机非法
ICM_ERR_TIMEOUT   (6)   等待超时
```

---

## 七、典型使用流程

### 7.1 纯姿态（只关心 roll / pitch / yaw）

```c
#include "icm.h"

icm_estimator_t est;
icm_raw_t raw;
icm_scaled_t s;
float eul[3];

icm_init();
icm_enable_sensors();
icm_est_init(&est);
icm_est_capture_gyro_bias(&est, 256);   /* 静止水平时标定 */

while (1) {
    if (icm_wait_data_ready(ICM_DRDY_TIMEOUT_MS) != ICM_OK) continue;
    icm_read_all(&raw);
    icm_convert(&raw, &s);
    icm_est_update_auto(&est, &s);

    icm_est_get_euler(&est, eul);
    /* eul[0]=roll, eul[1]=pitch, eul[2]=yaw（rad，yaw 会缓慢漂移） */
}
```

**裁剪建议**：关掉速度/位置积分可省 Flash 与 CPU。

```c
-D ICM_EST_VEL_ENABLE=0 -DICM_EST_POS_ENABLE=0
```

### 7.2 只读原始数据（不做解算）

```c
#define ICM_INTEG_ENABLE 0      /* 整个解算模块被裁掉 */

icm_init();
icm_enable_sensors();

while (1) {
    if (icm_wait_data_ready(100) != ICM_OK) continue;
    icm_read_all(&raw);
    icm_convert(&raw, &s);
    printf("%.3f g, %.3f g, %.3f g | %.2f dps\n",
           s.accel_g[0], s.accel_g[1], s.accel_g[2], s.gyro_dps[0]);
}
```

### 7.3 惯性导航（速度 + 位置）

```c
/* 需要 ICM_EST_VEL_ENABLE / ICM_EST_POS_ENABLE / ICM_EST_ZUPT_ENABLE 全开 */
float vel[3], pos[3];

icm_est_update_auto(&est, &s);
icm_est_get_velocity(&est, vel);
icm_est_get_position(&est, pos);
```

> ⚠️ 纯惯性位置解算**必然漂移**（误差随时间的平方累积）。ZUPT 只能在「确实静止」时把速度归零，无法解决运动中的累积误差。长时定位建议加其他传感器融合。

### 7.4 手动控制采样节奏（无滴答平台）

```c
float dt = 0.01f;    /* 假设 100 Hz 采样 */

while (1) {
    icm_read_all(&raw);
    icm_convert(&raw, &s);
    icm_est_update(&est, &s, dt);   /* 显式传 dt */
    my_delay_ms(10);
}
```

---

## 八、SPI 硬件约定

| 项 | 值 | 说明 |
|---|---|---|
| 线制 | **4 线 SPI** | 不是 3 线 |
| 模式 | **Mode 0 或 Mode 3** | CPOL/CPHA 同极同相 |
| 帧宽 | **8 bit** | 手册要求 |
| 位序 | **MSB first** | 手册要求 |
| 速率 | 建议 ≤ 24 MHz | 调通前先用低速（如 1 MHz） |
| CS 管理 | 由 `icm_hal_spi_xfer` 内部管理 | 业务层不接触 CS |

**Mode 0 vs Mode 3 对应关系**：

| 模式 | CPOL | CPHA |
|---|---|---|
| Mode 0 | Low | Leading（第一个边沿采样） |
| Mode 3 | High | Trailing（第二个边沿采样） |

> 配成 Mode 1 或 Mode 2 会读到全 `0x00` 或全 `0xFF`。

**CS 引脚**：配成普通 GPIO 输出，初始电平高。不要复用给 SPI 外设。

**INT 引脚**：可以不接。驱动默认走轮询（`icm_wait_data_ready`），不依赖中断线。

---

## 九、常见问题排查

### 9.1 启动阶段

| 现象 | 可能原因 | 排查动作 |
|---|---|---|
| `icm_init()` 返回 `ICM_ERR_WHOAMI` | ID 不匹配 | 确认料号：-C 版 `0x61`，非 -C 版 `0x60`，改 `ICM_WHO_AM_I_VALUE` |
| `icm_init()` 返回 `ICM_ERR_TIMEOUT` | MCLK 未就绪 / SPI 无响应 | 检查 CS 引脚、SPI 模式、是否有片选复用冲突 |
| `icm_init()` 返回 `ICM_ERR_HAL` | SPI 外设启动失败 | 查 FSP 配置、`R_SPI_Open` 返回值、回调是否绑定 |
| `icm_enable_sensors()` 返回 `ICM_ERR_STATE` | 前置 `icm_init()` 未成功 | 先确认 `icm_get_state()` 是否 `RESET` |

### 9.2 通信阶段

| 现象 | 可能原因 | 排查动作 |
|---|---|---|
| 读回全 `0xFF` | CS 未拉低 / Mode 错 / MISO 未接 | 查 CS 引脚方向配置；CPOL/CPHA 是否为 Mode 0/3 |
| 读回全 `0x00` | 只发没收 / 器件未上电 | 确认 SPI 配置了接收；量 VDD 电压 |
| 突发读 14 字节只有前几字节对 | 中途 CS 被抬 / 地址自增断裂 | 全局搜索 CS 引脚，确认无其他代码操作 |
| 偶发数据错位 | CS 拉高早于最后一位时钟 | 必须等完成标志/BSY 清零，不能靠固定延时 |
| 一次失败后永久失败 | 错误路径漏了拉高 CS | 检查所有 `return` 前是否都放了 CS |
| 首次传输卡死 | 等完成标志但回调没接上 | 确认 SPI 实例的 callback 已绑定 |

### 9.3 解算阶段

| 现象 | 可能原因 | 排查动作 |
|---|---|---|
| `icm_est_update_auto()` 返回 `ICM_ERR_HAL` | 滴答恒为 0 | 实现 `icm_hal_get_tick_ms()`，或改用显式 `dt` 版本 |
| `icm_est_update()` 返回 `ICM_ERR_PARAM` | `dt <= 0` 或 `dt > ICM_DT_MAX` | 检查采样间隔；卡顿严重可调大 `ICM_DT_MAX` |
| roll/pitch 不收敛反而发散 | Mahony 叉积符号相反 | 见 `icm_integ.c` 内注释，把 `prv_cross` 两个参数对调或 `err` 取反 |
| yaw 缓慢漂移 | 六轴 IMU 固有特性（无磁力计） | 正常现象；需要绝对航向请加磁力计 |
| 静置时姿态缓慢偏转 | 陀螺零偏未标定或标定环境不静止 | 重新 `icm_est_capture_gyro_bias`，确保标定时完全静止水平 |
| 位置快速漂移 | 纯惯性积分的固有误差 | 正常现象；确保 ZUPT 开启且静止判定阈值合理 |
| 输出出现 NaN | 数值异常 | 驱动已对 `asinf` 入参限幅；检查输入物理量是否异常 |

### 9.4 上板推荐验证顺序

按这个顺序逐级验证，每步都能定位问题层次：

```
1. icm_check_who_am_i() 返回 ICM_OK
   → 证明 SPI 通路 + 模式 + CS 全部正确
        ↓
2. icm_wait_data_ready() 返回 ICM_OK
   → 证明控制寄存器读写正常
        ↓
3. icm_read_all() + icm_convert() 打印 accel
   → 静止水平时 accel_g[2] ≈ 1.0
        ↓
4. icm_est_update() 若干帧后打印 euler
   → roll / pitch 收敛到 0
```

> **技巧**：先用 STUB 版 `icm_hal.c` 在 PC 上编译跑通（无需硬件），再换成真实平台实现。这样能快速区分「算法问题」与「底层 SPI/时序问题」。

---

## 十、FAQ

**Q1：为什么 yaw 会漂移？**
本驱动只用加速度计 + 陀螺仪。加速度计只能观测重力方向（提供 roll/pitch 的绝对参考），无法观测航向。yaw 只能靠陀螺积分，零偏与噪声会持续累积。需要绝对航向必须增加磁力计。

**Q2：为什么必须做陀螺零偏标定？**
陀螺存在零点偏差（典型几 dps）。不标定的话，积分后姿态会以每秒数度的速度持续旋转。标定只需在**上电后、器件静止且水平**时采样一次。

**Q3：`icm_est_update_auto` 和 `icm_est_update` 该用哪个？**
有系统毫秒滴答就用 `_auto`（省心、带超时保护）；没有滴答（`icm_hal_get_tick_ms` 返回 0）就必须用显式 `dt` 版本。

**Q4：量程该选多少？**
静态/慢速应用选小量程（2g / 250dps），分辨率更高；剧烈运动选大量程（16g / 2000dps），抗饱和。不确定时先用默认（16g / 2000dps）跑通，再按实际需要下调。

**Q5：能不能同时用两片 ICM？**
当前驱动使用文件级静态状态（`icm_ctrl.c` 的 `g_state` / `g_gyro_bias_dps`），**不支持多实例**。解算层（`icm_estimator_t`）本身是多实例安全的。若需要多片，需改造控制层。

**Q6：`icm_hal.c` 里的 User Code 区段会被覆盖吗？**
不会。区段由 `/*< User Code >[name]*/ ... /*< Code end >[name]*/` 标记包裹，配合同步脚本升级驱动时按区段名一一保留。**只需保证区段名不变，且改动都写在区段内。**

**Q7：能不能用 3 线 SPI？**
器件支持，但本驱动默认按 4 线设计（`ICM_SPI_4WIRE=1`）。用 3 线需改 `icm_regs.h` 的 `ICM_SPI_4WIRE=0`，并确认平台 SPI 支持三线模式（MISO/MOSI 合并）。

**Q8：采样很慢或偶尔丢帧怎么办？**
先用 `icm_wait_data_ready()` 替代固定延时；检查 SPI 时钟是否过低；确认 `ICM_USE_BURST_READ=1`（单次事务读全部数据）。若积分中丢帧，`ICM_DT_MAX` 会丢弃异常帧防止跳变。

---

## 附：最小可用代码

```c
#include "icm.h"
#include <stdio.h>

int main(void)
{
    icm_raw_t       raw;
    icm_scaled_t    s;
    icm_estimator_t est;
    float           eul[3];

    if (icm_init() != ICM_OK)           { printf("init fail\n");   return 1; }
    if (icm_enable_sensors() != ICM_OK) { printf("enable fail\n"); return 1; }

    icm_est_init(&est);
    icm_est_capture_gyro_bias(&est, 256);      /* 需静止水平 */

    for (int i = 0; i < 1000; i++) {
        if (icm_wait_data_ready(ICM_DRDY_TIMEOUT_MS) != ICM_OK) continue;
        icm_read_all(&raw);
        icm_convert(&raw, &s);
        icm_est_update_auto(&est, &s);

        icm_est_get_euler(&est, eul);
        printf("acc=(%6.3f,%6.3f,%6.3f)g  rpy=(%6.1f,%6.1f,%6.1f)deg\n",
               s.accel_g[0], s.accel_g[1], s.accel_g[2],
               eul[0] * 57.29578f, eul[1] * 57.29578f, eul[2] * 57.29578f);
    }
    return 0;
}
```

编译：

```bash
gcc -std=c99 -Wall -Wextra -I. main.c icm_hal.c icm_ctrl.c icm_integ.c -lm -o icm_demo
```
