/**
 * @file    icm_regs.h
 * @brief   ICM 系列六轴 IMU 驱动 —— 器件规格层（寄存器映射 / 位域 / 规格常数）
 *
 * @note    【本层定位】器件规格层。
 *          本文件内容全部来自芯片数据手册，是器件**固有规格**，
 *          不随平台（MCU/RTOS）、不随用户配置、不随应用场景变化，
 *          因此**通常无需修改**（切换芯片型号时由 icm_cfg.h 的
 *          ICM_CHIP_ID 选择，或将来新增 icm42688_regs.h 并在 cfg 中选入）。
 *
 * @note    【放什么】
 *          - 全部寄存器地址宏 ICM_REG_*
 *          - 全部位域宏（掩码 / 移位 / 模式值 / 复位位）
 *          - 器件规格常数：ICM_SPI_READ_FLAG、ICM_TEMP_DIV、ICM_TEMP_OFFSET_C
 *          - SPI 接口模式宏：ICM_SPI_4WIRE、ICM_SPI_MODE_0_3
 *
 * @note    【不放什么】
 *          - 用户可改参数（量程 / ODR / 时序 / 功能开关）→ 见 icm_cfg.h
 *          - 芯片选择宏 ICM_CHIP_ID / ICM_WHO_AM_I_VALUE → 见 icm_cfg.h
 *          - 错误码 ICM_ERR_* → 见 icm_cfg.h（跨层公共类型，放最底层避免循环依赖）
 *          - 状态机 icm_state_t / 数据结构 / API 声明 → 见 icm_ctrl.h
 *
 * @note    【依赖】无。仅依赖 <stdint.h> 等标准库（直接或经 icm_cfg.h 传递获得）。
 *          本文件可被任意层包含，不会产生依赖环。
 *
 * @note    手册核对（InvenSense/TDK ICM-42607-C Datasheet DS-000398 Rev1.0）：
 *          - 寄存器支持「地址自动自增」，故温/加/陀 14 字节可一次突发读回；
 *          - 数据就绪可用 INT_STATUS_DRDY(0x39) 的 DATA_RDY_INT(bit0) 轮询。
 */

#ifndef ICM_REGS_H
#define ICM_REGS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * 1) 器件规格常数（来自手册，不随平台/用户变化）
 * ============================================================================= */

/** 读命令标志：SPI 地址字节最高位置 1 表示读操作 */
#define ICM_SPI_READ_FLAG         (0x80U)

/** 温度分辨率：LSB/°C（手册 48 页：T = DATA/128 + 25） */
#define ICM_TEMP_DIV              (128.0f)

/** 温度零点偏移：°C */
#define ICM_TEMP_OFFSET_C         (25.0f)

/** SPI 接口宽度：1 = 4 线 SPI（推荐）；0 = 3 线 SPI（对应手册 43 页 DEVICE_CONFIG） */
#define ICM_SPI_4WIRE             (1U)

/** SPI 时钟模式：1 = Mode0/Mode3（CPOL=0/CPHA=0 或 CPOL=1/CPHA=1）；0 = Mode1/Mode2 */
#define ICM_SPI_MODE_0_3          (1U)

/* =============================================================================
 * 2) 寄存器地址映射（Bank 0 用户寄存器，无需切 bank；MREG 高级特性未用）
 * ============================================================================= */
#define ICM_REG_MCLK_RDY            (0x00U)  /**< 44页：MCLK 就绪状态（bit3 MCLK_RDY） */
#define ICM_REG_DEVICE_CONFIG       (0x01U)  /**< 43页：SPI 模式 / 4线 */
#define ICM_REG_SIGNAL_PATH_RESET   (0x02U)  /**< 44页：软复位 */
#define ICM_REG_PWR_MGMT0           (0x1FU)  /**< 53页：上电 / 功耗模式 */
#define ICM_REG_GYRO_CONFIG0        (0x20U)  /**< 54页：陀螺量程 / ODR */
#define ICM_REG_ACCEL_CONFIG0       (0x21U)  /**< 55页：加速度量程 / ODR */
#define ICM_REG_INT_STATUS_DRDY     (0x39U)  /**< 65页：数据就绪状态（bit0） */
#define ICM_REG_WHO_AM_I            (0x75U)  /**< 67页：器件 ID */

/* 传感器数据寄存器（连续地址，支持突发读） */
#define ICM_REG_TEMP_DATA1          (0x09U)  /**< 温度高字节，起始 */
#define ICM_REG_ACCEL_DATA_X1       (0x0BU)  /**< 加速度 X 高字节 */
#define ICM_REG_GYRO_DATA_X1        (0x11U)  /**< 陀螺 X 高字节 */

/* 突发读长度：TEMP(2) + ACCEL(6) + GYRO(6) = 14 字节数据 + 1 字节命令 */
#define ICM_BURST_DATA_LEN          (14U)
#define ICM_BURST_TX_LEN            (15U)

/* =============================================================================
 * 3) 寄存器位域（掩码 / 移位 / 模式值）
 * ============================================================================= */

/* --- MCLK_RDY (0x00) --- */
#define ICM_MCLK_RDY_BIT            (0x08U)  /**< bit3：1 = 内部时钟就绪 */

/* --- DEVICE_CONFIG (0x01) --- */
#define ICM_DEVICE_CONFIG_SPI_AP_4WIRE  (0x04U)  /**< bit2：1 = 4 线 SPI */
#define ICM_DEVICE_CONFIG_SPI_MODE      (0x01U)  /**< bit0：0 = Mode0/3，1 = Mode1/2 */

/* --- SIGNAL_PATH_RESET (0x02) --- */
#define ICM_SIGNAL_PATH_RESET_SOFT      (0x10U)  /**< bit4：软复位（自清位） */

/* --- PWR_MGMT0 (0x1F) --- */
#define ICM_PWR_MGMT0_GYRO_MODE_LN      (0x03U)  /**< bits3:2 = 11：陀螺低噪声(LN) */
#define ICM_PWR_MGMT0_ACCEL_MODE_LN     (0x03U)  /**< bits1:0 = 11：加速度低噪声(LN) */
#define ICM_PWR_MGMT0_GYRO_SHIFT        (2U)     /**< 陀螺模式位起始 */
#define ICM_PWR_MGMT0_ACCEL_SHIFT       (0U)     /**< 加速度模式位起始 */
#define ICM_PWR_MGMT0_MODE_MASK         (0x03U)  /**< 模式子域掩码 */

/* --- INT_STATUS_DRDY (0x39) --- */
#define ICM_DRDY_INT_BIT                (0x01U)  /**< bit0：DATA_RDY_INT */

/* --- GYRO_CONFIG0 / ACCEL_CONFIG0 (0x20 / 0x21) --- */
#define ICM_CONFIG0_FS_SEL_SHIFT        (5U)     /**< 量程位起始（bit6:5） */
#define ICM_CONFIG0_FS_SEL_MASK         (0x07U)  /**< 量程位掩码（手册为 3 位域） */
#define ICM_CONFIG0_ODR_MASK            (0x0FU)  /**< ODR 位掩码（bit3:0） */

#ifdef __cplusplus
}
#endif

#endif /* ICM_REGS_H */
