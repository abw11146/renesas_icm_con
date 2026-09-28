/**
 * @file    icm_cfg.h
 * @brief   ICM 系列六轴 IMU 驱动 —— 纯配置层（唯一需要按需调整的参数集中在此）
 *
 * @note    【本层定位】本文件是纯配置层。**只放用户可改参数与由其推导的宏**。
 *          寄存器映射见 icm_regs.h，控制语义见 icm_ctrl.h。
 *
 * @note    【放什么】
 *          - 芯片选择宏 ICM_CHIP_ID（默认 ICM_CHIP_42607，为将来加 42688 预留）
 *          - 芯片相关固定常量：ICM_WHO_AM_I_VALUE（默认 0x61，非 -C 版 0x60）
 *          - 用户可改参数：量程、ODR、功能开关、时序常数
 *          - 由上述参数编译期推导出的宏（ICM_ACCEL_FS_SEL / ICM_ACCEL_RES_G /
 *            ICM_GYRO_RES_DPS 等）
 *          - 统一错误码（跨层公共基础类型，见下方说明）
 *          - 标准库 include（stdint / stdbool / stddef）
 *
 * @note    【不放什么】
 *          - 寄存器地址 / 位域 / SPI 模式宏 / 温度换算常数 → 见 icm_regs.h
 *          - 状态枚举 icm_state_t / 数据结构 / API 声明 → 见 icm_ctrl.h
 *
 * @note    【错误码为何在此（方案 A）】错误码是跨层公共基础类型：
 *          icm_hal.h / icm_ctrl.h / icm_integ.h 都要用它作返回值。
 *          若放在 icm_ctrl.h，则 icm_hal.h 必须包含 icm_ctrl.h，
 *          而 icm_ctrl.h 又包含 icm_hal.h，形成「hal → ctrl → hal」循环依赖。
 *          放在最底层的 cfg.h 可彻底避免环，且 cfg.h 仍是纯头文件（不新增 .c）。
 *
 * @note    【平台无关】不依赖任何 MCU 平台 / HAL / 编译器扩展，
 *          可安全被任意 .c 包含。移植到 RX / RA 等平台时本文件通常**无需改动**；
 *          仅需在 icm_hal.c 的 User Code 区段内实现 HAL。
 *
 * @note    手册核对补充（InvenSense/TDK ICM-42607-C Datasheet DS-000398 Rev1.0）：
 *          - 灵敏度与量程严格对应手册 54/55 页：FS_SEL=0 对应最大量程
 *            (陀螺 ±2000dps，加速度 ±16g)。本驱动用 FS/32768 推导分辨率，与手册
 *            "16.4 LSB/dps" 等数值完全一致。
 *          - 所有量程 / ODR 宏都做了「物理值 -> 寄存器位」的编译期映射，避免两者
 *            配置不一致导致静默错误（手册 53 页 PWR_MGMT0 / 54 页 GYRO_CONFIG0 要求
 *            写错位域会导致传感器不工作）。
 */

#ifndef ICM_CFG_H
#define ICM_CFG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /**< NULL，保证 HAL/ctrl 不依赖平台头也能用 NULL */

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * 1) 芯片选择（为将来兼容 ICM-42688 等同宗器件预留）
 * =============================================================================
 * 切换芯片时改这里 + 下方 ICM_WHO_AM_I_VALUE；若该型号寄存器有差异，
 * 将来可新增 icm42688_regs.h 并在本处选入。
 * ============================================================================= */
#define ICM_CHIP_42607            (42607)  /**< ICM-42607 系列（当前） */
#define ICM_CHIP_42688            (42688)  /**< ICM-42688 系列（预留，暂未实现） */

#ifndef ICM_CHIP_ID
#define ICM_CHIP_ID               (ICM_CHIP_42607)  /**< 目标芯片型号 */
#endif

/** 器件 WHO_AM_I 期望值；
 *  ICM-42607-C 默认 0x61；注意非 -C 版本(ICM-42607)为 0x60，
 *  若使用非 -C 料号请把本值改为 0x60。 */
#ifndef ICM_WHO_AM_I_VALUE
#define ICM_WHO_AM_I_VALUE        (0x61U)
#endif

/* =============================================================================
 * 2) 统一错误码（不依赖任何外部 drv_err_code.h，做到平台自包含）
 *    —— 跨层公共类型，放最底层以避免 hal/ctrl 循环依赖（方案 A）
 * ============================================================================= */
typedef uint8_t icm_err_t;

#define ICM_OK                  (0U)   /**< 执行成功 */
#define ICM_ERR_HAL             (1U)   /**< 底层 HAL（SPI/外设）返回失败 */
#define ICM_ERR_COMM            (2U)   /**< SPI 链路层错误（传输失败） */
#define ICM_ERR_WHOAMI          (3U)   /**< WHO_AM_I 校验不匹配（器件型号/接线错误） */
#define ICM_ERR_PARAM           (4U)   /**< 参数非法（空指针 / 非法取值） */
#define ICM_ERR_STATE           (5U)   /**< 状态机非法（未初始化就调用业务接口） */
#define ICM_ERR_TIMEOUT         (6U)   /**< 等待超时（MCLK 未就绪 / 数据未就绪） */

/* =============================================================================
 * 3) 传感器量程 / ODR 配置（最常修改区）
 *    —— 加速度计量程以「g」选择，陀螺以「dps」选择，编译期自动映射为寄存器位域，
 *       避免手动填位域出错。分辨率(每 LSB 代表的物理量)由 FS/32768 直接推导。
 * ============================================================================= */

/* ----- 加速度计量程（仅支持 2/4/8/16 g）----- */
#ifndef ICM_ACCEL_FS_G
#define ICM_ACCEL_FS_G            (16)    /**< 期望量程(g)：2/4/8/16 */
#endif
#if   (ICM_ACCEL_FS_G == 16)
  #define ICM_ACCEL_FS_SEL        (0U)
#elif (ICM_ACCEL_FS_G == 8)
  #define ICM_ACCEL_FS_SEL        (1U)
#elif (ICM_ACCEL_FS_G == 4)
  #define ICM_ACCEL_FS_SEL        (2U)
#elif (ICM_ACCEL_FS_G == 2)
  #define ICM_ACCEL_FS_SEL        (3U)
#else
  #error "ICM_ACCEL_FS_G 仅支持 2/4/8/16 (g)"
#endif
/** 加速度分辨率：每 LSB 对应多少 g（= 满量程 / 16bit 半量程） */
#define ICM_ACCEL_RES_G           ((float)ICM_ACCEL_FS_G / 32768.0f)

/* ----- 陀螺仪量程（仅支持 250/500/1000/2000 dps）----- */
#ifndef ICM_GYRO_FS_DPS
#define ICM_GYRO_FS_DPS           (2000)  /**< 期望量程(dps)：250/500/1000/2000 */
#endif
#if   (ICM_GYRO_FS_DPS == 2000)
  #define ICM_GYRO_FS_SEL         (0U)
#elif (ICM_GYRO_FS_DPS == 1000)
  #define ICM_GYRO_FS_SEL         (1U)
#elif (ICM_GYRO_FS_DPS == 500)
  #define ICM_GYRO_FS_SEL         (2U)
#elif (ICM_GYRO_FS_DPS == 250)
  #define ICM_GYRO_FS_SEL         (3U)
#else
  #error "ICM_GYRO_FS_DPS 仅支持 250/500/1000/2000 (dps)"
#endif
/** 陀螺分辨率：每 LSB 对应多少 dps */
#define ICM_GYRO_RES_DPS          ((float)ICM_GYRO_FS_DPS / 32768.0f)

/* ----- ODR 命名常量（手册 54/55 页 ODR 位域）-----
 *   0x05=1.6k 0x06=800 0x07=400 0x08=200 0x09=100 0x0A=50 0x0B=25 0x0C=12.5(Hz)
 *   直接在下方选择加速度/陀螺 ODR_CODE。两者可不同。 */
#define ICM_ODR_1600HZ            (0x05U)
#define ICM_ODR_800HZ             (0x06U)
#define ICM_ODR_400HZ             (0x07U)
#define ICM_ODR_200HZ             (0x08U)
#define ICM_ODR_100HZ             (0x09U)
#define ICM_ODR_50HZ              (0x0AU)
#define ICM_ODR_25HZ              (0x0BU)
#define ICM_ODR_12_5HZ            (0x0CU)

#ifndef ICM_GYRO_ODR
#define ICM_GYRO_ODR              (ICM_ODR_800HZ)   /**< 陀螺 ODR（默认 800Hz） */
#endif
#ifndef ICM_ACCEL_ODR
#define ICM_ACCEL_ODR             (ICM_ODR_800HZ)   /**< 加速度 ODR（默认 800Hz）*/
#endif

/* =============================================================================
 * 4) 时序参数（单位：ms / us）
 *    手册核对补充：
 *    - 软复位后等待：手册未给精确值，但器件内部 MCLK 启动需时间；原驱动仅 2ms 偏紧，
 *      这里默认 10ms，并在 init 中额外轮询 MCLK_RDY 兜底（见 ICM_CHECK_MCLK_RDY）。
 *    - 从 OFF 切到 LN 后 200us 内禁止写寄存器（手册 53 页）；enable 函数已在切模式后等待。
 *    - 陀螺上电后需保持 ON 至少 45ms 数据才稳定；启用 ICM_WAIT_GYRO_RDY 可在
 *      enable 后阻塞等待该时长（默认开启）。
 * ============================================================================= */
#ifndef ICM_SOFT_RESET_MS
#define ICM_SOFT_RESET_MS         (10U)   /**< 软复位后等待(ms) */
#endif
#ifndef ICM_PWR_ON_SETTLE_US
#define ICM_PWR_ON_SETTLE_US      (200U)  /**< OFF->LN 切换后禁止写寄存器的窗口(us) */
#endif
#ifndef ICM_GYRO_RDY_MS
#define ICM_GYRO_RDY_MS           (50U)   /**< 陀螺上电稳定最短时间(ms) */
#endif
#ifndef ICM_MCLK_RDY_TIMEOUT_MS
#define ICM_MCLK_RDY_TIMEOUT_MS   (100U)  /**< 轮询 MCLK_RDY 超时(ms) */
#endif
#ifndef ICM_DRDY_TIMEOUT_MS
#define ICM_DRDY_TIMEOUT_MS       (100U)  /**< 轮询 DATA_RDY 超时(ms) */
#endif
#ifndef ICM_SPI_TIMEOUT_MS
#define ICM_SPI_TIMEOUT_MS        (100U)  /**< SPI 传输超时(ms)，HAL 内部用于判断 */
#endif

#ifndef ICM_CHECK_MCLK_RDY
#define ICM_CHECK_MCLK_RDY        (1)     /**< 1=init 后轮询 MCLK_RDY 再继续（更稳） */
#endif
#ifndef ICM_WAIT_GYRO_RDY
#define ICM_WAIT_GYRO_RDY         (1)     /**< 1=enable 后等待陀螺稳定时长 */
#endif

/* =============================================================================
 * 5) 六轴积分解算（姿态/速度/位置）配置
 *    —— 见 icm_integ.c 实现。使用 float 以获得精度与可读性；若目标 MCU
 *       无 FPU（如 RA2L1 Cortex-M23、部分 RX），编译器会以软浮点实现，会占用数 KB
 *       Flash，但代码最简、最不易出错。如需极致体积可后续改 Q 格式定点（注释中已标注）。
 * ============================================================================= */
#ifndef ICM_INTEG_ENABLE
#define ICM_INTEG_ENABLE          (1)   /**< 总开关：0 可裁剪掉整个积分解算模块(省 Flash) */
#endif
#ifndef ICM_EST_TILT_CORRECT
#define ICM_EST_TILT_CORRECT      (1)   /**< 1=用加速度计做倾角校正(Mahony)，抑制 roll/pitch 漂移 */
#endif
#ifndef ICM_EST_GRAVITY_COMP
#define ICM_EST_GRAVITY_COMP      (1)   /**< 1=从加速度中扣除重力，得到「线加速度」 */
#endif
#ifndef ICM_EST_VEL_ENABLE
#define ICM_EST_VEL_ENABLE        (1)   /**< 1=积分出速度 */
#endif
#ifndef ICM_EST_POS_ENABLE
#define ICM_EST_POS_ENABLE        (1)   /**< 1=积分出位置（纯惯性易漂移，可单独关闭） */
#endif
#ifndef ICM_EST_ZUPT_ENABLE
#define ICM_EST_ZUPT_ENABLE       (1)   /**< 1=静止检测(ZUPT)归零速度，显著抑制漂移、提升定位准确度 */
#endif

#ifndef ICM_EST_KP
#define ICM_EST_KP                (0.2f)   /**< 倾角校正比例增益(Mahony)。
                                                取值权衡：过小->静态调平慢(≈1/Kp 秒)；
                                                过大->动态旋转时加速度计把真实旋转误判为倾斜，抑制姿态。
                                                0.1~0.3 是 Six-Axis(无磁)常用区间，本驱动默认 0.2。 */
#endif
#ifndef ICM_EST_KI
#define ICM_EST_KI                (0.0f)   /**< 倾角校正积分增益，>0 可额外消除缓慢零偏（需更多 RAM/CPU） */
#endif

#ifndef ICM_GRAVITY
#define ICM_GRAVITY               (9.80665f)     /**< 重力加速度 m/s^2 */
#endif
#ifndef ICM_DEG2RAD
#define ICM_DEG2RAD               (0.0174532925f) /**< π/180 */
#endif

/* 静止检测阈值（ZUPT）：加速度模长偏离 1g 很小 且 角速度很小 => 视为静止 */
#ifndef ICM_ZUPT_ACC_DEV
#define ICM_ZUPT_ACC_DEV          (0.08f)  /**< |a|-1g 偏差阈值(g) */
#endif
#ifndef ICM_ZUPT_GYRO_DEV
#define ICM_ZUPT_GYRO_DEV         (0.05f)  /**< 角速度阈值(rad/s)，约 2.9°/s */
#endif
#ifndef ICM_VEL_MAX
#define ICM_VEL_MAX               (20.0f)  /**< 速度限幅(m/s)，防积分爆炸 */
#endif
#ifndef ICM_DT_MAX
#define ICM_DT_MAX                (0.2f)   /**< 单次积分最大 dt(s)，超时/卡顿直接丢弃本帧防跳变 */
#endif

/* 陀螺零偏标定默认样本数（icm_est_capture_gyro_bias 使用，需器件静止水平放置） */
#ifndef ICM_BIAS_SAMPLES
#define ICM_BIAS_SAMPLES          (256U)   /**< 零偏标定采样数，需器件静止水平放置 */
#endif

/* =============================================================================
 * 6) 行为裁剪开关（影响体积/性能）
 * ============================================================================= */
#ifndef ICM_USE_BURST_READ
#define ICM_USE_BURST_READ        (1)   /**< 1=用单次 14 字节突发读取温/加/陀，时间戳对齐、省 SPI 事务 */
#endif
#ifndef ICM_KEEP_SEPARATE_READ
#define ICM_KEEP_SEPARATE_READ    (1)   /**< 1=保留 read_accel/read_gyro/read_temp 单轴接口（兼容旧代码） */
#endif

#ifdef __cplusplus
}
#endif

#endif /* ICM_CFG_H */
