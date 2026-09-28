/**
 * =============================================================================
 *  icm.h  ——  ICM 系列六轴 IMU 驱动，统一对外头文件
 * =============================================================================
 *
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │  1. 文件说明                                                          │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 *  [器件]   InvenSense / TDK ICM-42607-C（当前默认，-P/-A 等同宗），6 轴
 *           （3 轴陀螺 + 3 轴加速度），SPI 4 线通信，量程/ODR 可配。
 *           前缀统一为 icm_ / ICM_，芯片型号由 icm_cfg.h 的 ICM_CHIP_ID 选择，
 *           为将来兼容 ICM-42688 等同类器件预留。
 *
 *  [用途]   本文件是驱动对外的「唯一入口」。业务代码**只需 #include "icm.h"**
 *           即可使用全部功能，不需要、也不应该直接包含其他底层头文件：
 *             - 器件控制：icm_init / icm_enable_sensors / icm_read_all / icm_convert ...
 *             - 六轴积分解算：icm_est_* （姿态 / 速度 / 位置）
 *
 *  [架构]   驱动按「配置 → 规格 → 底层 → 控制 → 解算」分层，依赖严格单向、无环：
 *
 *                        icm.h  (本文件，统一聚合)
 *                             │
 *        ┌──────────┬─────────┼──────────────────┬────────────────────┐
 *        │          │         │                  │                    │
 *   icm_cfg.h  icm_regs.h  icm_hal.h   ┌─────────┴──────────┐         │
 *  (配置层：    (规格层：   (HAL 接口    │  icm_ctrl.h        │  icm_integ.h
 *   用户可改     寄存器/     声明；        │  (控制层 API +     │  (算法层 API +
 *   参数、错误    位域/规格   icm_hal.c    │   状态机/数据结构)  │   估计器)
 *   码、芯片      常数，     中 User Code  └─────────┬──────────┘         │
 *   选择宏)      通常不改)   区段实现)              │                    │
 *                                                  ▼                    ▼
 *                                        icm_hal.c  (HAL 模板，用户填 User Code)
 *                                        icm_ctrl.c (寄存器/状态机)
 *                                        icm_integ.c(四元数/积分)
 *
 *           分层依赖（严格单向，仅向下）：
 *             icm_cfg.h    ：不依赖任何层（仅 stdint/stdbool/stddef）
 *             icm_regs.h   ：不依赖任何层
 *             icm_hal.h    ：依赖 icm_cfg.h
 *             icm_ctrl.h   ：依赖 icm_cfg.h / icm_regs.h / icm_hal.h
 *             icm_integ.h  ：依赖 icm_ctrl.h（**不依赖 icm_regs.h**）
 *
 *  [内存/性能]   - 控制层零静态缓冲（除状态机变量）；解算层每实例约 80 字节(RAM)。
 *                - 解算用 float + 一阶四元数更新，无 sin/cos/矩阵求逆，仅 1~2 次
 *                  sqrtf，CPU/Flash 占用极低；无 FPU 的 MCU（如 RA2L1/RX）可用软浮点。
 *                - 功能宏（见 icm_cfg.h）可裁剪：关闭位置/速度/ZUPT/重力补偿可进一步
 *                  省 Flash/RAM；ICM_INTEG_ENABLE=0 可整体裁掉积分解算。
 *
 *  [依赖]       C99、icm_cfg.h 中已自包含 <stdint.h>/<stdbool.h>/<stddef.h>。
 *               不依赖任何 MCU SDK/RTOS。
 *
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │  2. API 接口说明（按使用顺序）                                          │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 *  === 2.1 器件控制（来自 icm_ctrl.h）=====================================
 *
 *  -- 初始化与配置 --
 *    icm_err_t icm_init(void)
 *        初始化 HAL → 软复位 → 校验 WHO_AM_I → (可选)等待 MCLK 就绪。
 *        成功后状态机进入 RESET/IDLE，**需再调用 icm_enable_sensors() 才能采样**。
 *        返回 ICM_OK 表示成功，否则可查 icm_get_state()。
 *
 *    icm_err_t icm_enable_sensors(void)
 *        使能加速度计 + 陀螺仪（低噪声/LN 模式），量程与 ODR 取自 icm_cfg.h。
 *        内部顺序遵循手册：先配 CONFIG0，再写 PWR_MGMT0 切 LN，等 200µs
 *        （OFF→LN 写禁忌窗口），再按需等陀螺稳定(45ms)。成功后进入 ACTIVE。
 *
 *    icm_err_t icm_soft_reset(void)
 *    icm_err_t icm_check_who_am_i(void)
 *        软复位 / 单独校验器件 ID（调试用）。
 *
 *  -- 寄存器读写（一般业务无需调用，解算有更高层封装）--
 *    icm_err_t icm_read_reg (uint8_t reg, uint8_t *p_val);
 *    icm_err_t icm_write_reg(uint8_t reg, uint8_t val);
 *
 *  -- 采样流程（推荐）--
 *    icm_err_t icm_wait_data_ready(uint32_t timeout_ms)
 *        轮询 INT_STATUS_DRDY 的 DATA_RDY_INT(bit0)，超时返回 ICM_ERR_TIMEOUT。
 *        比固定延时更准、更省 CPU；无中断线时也可直接调用 read_all（略等即可）。
 *
 *    icm_err_t icm_read_all(icm_raw_t *raw)
 *        **一次 SPI 突发读取** 温度(2)+加速度(6)+陀螺(6)=14 字节，
 *        保证三轴时间戳一致（对积分至关重要）。返回原始 16bit 有符号值。
 *
 *    icm_err_t icm_convert(const icm_raw_t *raw, icm_scaled_t *out)
 *        原始 → 物理量：out->temp_c(°C) / accel_g[3](g) / gyro_dps[3](°/s)。
 *        换算分辨率按 icm_cfg.h 中的量程自动取用，无需手填系数。
 *
 *    （可选）分轴读取宏 ICM_KEEP_SEPARATE_READ=1 时提供
 *        icm_read_accel / icm_read_gyro / icm_read_temp，兼容旧代码，不推荐新工程用。
 *
 *  -- 陀螺零偏标定（静态误差补偿）--
 *    void    icm_set_gyro_bias(float x,float y,float z);  // 单位 dps
 *    void    icm_get_gyro_bias(float *x,*y,*z);
 *    icm_err_t icm_capture_gyro_bias(uint16_t n);
 *        静止水平放置时采 n 个样本取平均作为零偏；务必保持器件静止。
 *
 *  -- 状态/调试 --
 *    icm_state_t icm_get_state(void);   // 读状态机当前状态
 *
 *  === 2.2 六轴积分解算（来自 icm_integ.h，需 ICM_INTEG_ENABLE=1）========
 *
 *  解算链路：陀螺→四元数姿态 → Mahony 加速度倾角校正(抑制 roll/pitch 漂移)
 *           → 重力补偿得线加速度 → 积分速度/位置 → ZUPT 静止归零(抑制漂移)。
 *  注意：无磁力计，yaw 仅由陀螺积分、会缓慢漂移，属正常现象。
 *
 *    typedef struct icm_estimator_t { ... } // 解算上下文(约80B RAM)
 *
 *    void    icm_est_init (icm_estimator_t *e);  // 复位：q=单位, v/p=0
 *    void    icm_est_reset(icm_estimator_t *e);
 *
 *    icm_err_t icm_est_capture_gyro_bias(icm_estimator_t *e, uint16_t n);
 *            静止时采集 n 样本求零偏，标定后无需再手动 set_bias。
 *
 *    icm_err_t icm_est_update(icm_estimator_t *e,
 *                             const icm_scaled_t *s, float dt);
 *            核心：用一帧物理量做一步解算。dt 为本帧间隔(s)，需 >0 且
 *            <= ICM_DT_MAX；非法/未初始化会返回错误。
 *
 *    icm_err_t icm_est_update_auto(icm_estimator_t *e,
 *                                  const icm_scaled_t *s);
 *            同上，但 dt 由 HAL 系统滴答自动计算（**推荐**，最省心且带超时保护）。
 *
 *    // 取值接口（只读取出，不修改上下文）
 *    void icm_est_get_quat    (const e, float q[4]);        // 四元数
 *    void icm_est_get_euler   (const e, float euler[3]);    // roll/pitch/yaw (rad)
 *    void icm_est_get_velocity(const e, float v[3]);        // 世界系速度 (m/s)
 *    void icm_est_get_position(const e, float p[3]);        // 世界系位置 (m)
 *
 *  === 2.3 典型业务代码骨架 ===============================================
 *
 *      #include "icm.h"
 *      icm_estimator_t est;
 *      icm_raw_t raw;  icm_scaled_t s;
 *      float eul[3], vel[3], pos[3];
 *
 *      icm_init();
 *      icm_enable_sensors();
 *      icm_est_init(&est);
 *      icm_est_capture_gyro_bias(&est, 256);   // 静止时标定零偏
 *
 *      while (1) {
 *          icm_wait_data_ready(100);
 *          icm_read_all(&raw);
 *          icm_convert(&raw, &s);
 *          icm_est_update_auto(&est, &s);        // 用系统滴答自动算 dt
 *          icm_est_get_euler(&est, eul);
 *          icm_est_get_position(&est, pos);
 *      }
 *
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │  3. 移植 / 配置说明                                                     │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 *  三类改动彼此独立，请按需区分（不要混淆「移植」与「调参」）：
 *
 *  [A] 移植（换 MCU / 换平台）——**只改 icm_hal.c 中的 User Code 区段**
 *      驱动核心（cfg / regs / ctrl / integ）完全平台无关，一行都不用改。
 *      在 icm_hal.c 中，每个 HAL 函数体只包含一个 User Code 区段，区段名 == 函数名：
 *
 *          - 区段标记形如：  User Code 标记 + [icm_hal_init]  ...  Code end 标记 + [icm_hal_init]
 *            即在 `User Code` / `Code end` 标记后带方括号区段名，形如：
 *            [User Code] [icm_hal_init]  ...  [Code end] [icm_hal_init]
 *            本节中的 5 个函数与 1 个文件级区段依次为：
 *              icm_hal_init / icm_hal_spi_xfer / icm_hal_delay_us /
 *              icm_hal_delay_ms / icm_hal_get_tick_ms / includes(放平台头文件)
 *
 *      只需把区段内默认的 STUB 替换为你的平台实现：
 *        - icm_hal_init()        : 初始化 SPI 外设 + CS 置为无效(高)
 *        - icm_hal_spi_xfer()    : 单次全双工传输，**自动管理 CS**（起拉低/止拉高）
 *        - icm_hal_delay_us()    : 微秒延时（用于 200us 等时序，精度±50%内即可）
 *        - icm_hal_delay_ms()    : 毫秒延时
 *        - icm_hal_get_tick_ms() : 系统滴答(ms)，供积分 dt 计算（无则返 0，
 *                                  此时须用 icm_est_update 显式传 dt）
 *      SPI 约定：4 线、Mode0/3、8bit、MSB first；CS 在 xfer 内部管理，业务不关心。
 *      → 区段外内容由驱动维护者管理，**升级驱动时不会被覆盖**：
 *        配合带 User Code 标记的同步脚本，用户代码按区段名一一保留。
 *
 *  [B] 应用调参（与平台无关）——改 icm_cfg.h：
 *      - 量程：ICM_GYRO_FS_DPS / ICM_ACCEL_FS_G（dps / g）
 *      - ODR ：ICM_GYRO_ODR / ICM_ACCEL_ODR（Hz）
 *      - 时序：ICM_SOFT_RESET_MS、ICM_GYRO_RDY_MS、ICM_DT_MAX 等
 *      - 解算：是否需要位置/速度/ZUPT/倾角校正、KP/KI 增益、速度上限等
 *
 *  [C] 芯片切换 —— 改 icm_cfg.h：
 *      - 设 ICM_CHIP_ID（默认 ICM_CHIP_42607）为对应型号；
 *      - 设 ICM_WHO_AM_I_VALUE：-C 版 0x61（默认），非 -C 版 0x60；
 *      - 若该型号寄存器有差异，将来可新增 icm42688_regs.h 并在 icm_cfg.h 选入。
 *
 *  [常见坑]
 *      - 没接中断线：无需初始化 INT 引脚，直接轮询 icm_wait_data_ready 或 icm_read_all。
 *      - 滴答缺失：icm_hal_get_tick_ms 返回 0 时，必须用 icm_est_update(s, dt) 手动给 dt。
 *      - 软复位后必须等 MCLK 就绪（icm_init 已处理），否则 WHO_AM_I 读错。
 *      - WHO_AM_I 不匹配（ICM_ERR_WHOAMI）：检查 ICM_WHO_AM_I_VALUE 是否与料号一致。
 *      - RX 无 FPU：float 解算走软浮点，可接受；若要极致体积可后续改 Q 格式定点。
 *
 *  [校验]   icm_hal.c 默认 STUB 可在 PC 直接编译链接并通过自测，建议先跑通逻辑，
 *           再换到目标平台，可快速区分「算法问题」与「底层 SPI/时序问题」。
 *
 * =============================================================================
 */

#ifndef ICM_H
#define ICM_H

#include "icm_cfg.h"    /* 配置层：用户可改参数 / 错误码 / 芯片选择（平台无关） */
#include "icm_regs.h"   /* 规格层：寄存器地址 / 位域 / 规格常数（通常不改） */
#include "icm_hal.h"    /* HAL 接口声明（由用户在 icm_hal.c 的 User Code 区段实现） */
#include "icm_ctrl.h"   /* 控制层：状态机 / 数据结构 / 器件控制 API */
#include "icm_integ.h"  /* 算法层：六轴积分解算 API */

#endif /* ICM_H */
