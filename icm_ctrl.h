/**
 * @file    icm_ctrl.h
 * @brief   ICM 系列六轴 IMU 驱动 —— 控制层：状态机、数据结构与器件控制 API
 *
 * @note    【本层定位】控制层。包含错误码（来自 cfg.h）、状态机、数据结构与控制 API。
 *          本层只做「器件控制」，不碰任何 MCU 寄存器（寄存器访问走 icm_hal）。
 *
 * @note    【放什么】
 *          - 状态机 icm_state_t
 *          - 数据结构 icm_raw_t / icm_scaled_t
 *          - 控制层 API 声明（icm_init / icm_enable_sensors / icm_read_all /
 *            icm_convert / icm_capture_gyro_bias / icm_get_state 等）
 *          - 错误码 icm_err_t / ICM_ERR_* 的**使用说明**（定义在 icm_cfg.h，见下）
 *
 * @note    【不放什么】
 *          - 寄存器地址 / 位域 / 器件规格常数 → 见 icm_regs.h
 *          - 用户可改参数 → 见 icm_cfg.h
 *
 * @note    【依赖】icm_cfg.h（配置/错误码）、icm_regs.h（寄存器映射）、
 *          icm_hal.h（SPI/延时）。严格单向，无循环依赖。
 *
 * @note    手册核对补充（DS-000398 Rev1.0）：
 *          - 寄存器支持「自动地址自增」，因此温/加/陀共 14 字节可一次突发读回，
 *            保证三轴时间戳一致（原驱动分 3 次读，存在几 ms 相位差，对积分不利）。
 *          - 数据就绪可用 INT_STATUS_DRDY(0x39) 的 DATA_RDY_INT(bit0) 轮询，
 *            比固定延时更准、更省 CPU（原驱动未使用）。
 */

#ifndef ICM_CTRL_H
#define ICM_CTRL_H

#include "icm_cfg.h"
#include "icm_regs.h"
#include "icm_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * 1) 驱动状态机（FSM）
 *    流转：UNINIT -> RESET -> IDLE -> ACTIVE
 *          \---------------------> ERROR（任意异常进入）
 *    - UNINIT：上电默认，未调用 icm_init()
 *    - RESET ：已完成软复位 + WHO_AM_I 校验
 *    - IDLE  ：传感器已上电但未使能（或待机），可随时进入 ACTIVE
 *    - ACTIVE：传感器已使能并处于低噪声(LN)模式，可连续采样
 *    - ERROR ：发生不可恢复错误，需重新 init()
 *
 *    注：枚举中 ERROR 排在 ACTIVE 之后，业务入口统一用「非 ERROR 且不低于
 *        min_state」判定（见 icm_ctrl.c 的 prv_state_ok），不能直接用 `<` 比较。
 * ============================================================================= */
typedef enum
{
    ICM_STATE_UNINIT = 0,
    ICM_STATE_RESET,
    ICM_STATE_IDLE,
    ICM_STATE_ACTIVE,
    ICM_STATE_ERROR
} icm_state_t;

/* =============================================================================
 * 2) 数据结构
 * ============================================================================= */
/** 原始数据（16bit 有符号，来自寄存器，未经换算） */
typedef struct
{
    int16_t temp;          /**< 温度原始值（TEMP_DATA） */
    int16_t accel[3];      /**< 加速度原始值 X/Y/Z */
    int16_t gyro[3];       /**< 陀螺原始值 X/Y/Z */
} icm_raw_t;

/** 物理量数据（已换算） */
typedef struct
{
    float temp_c;          /**< 温度 (°C) */
    float accel_g[3];      /**< 加速度 (g) */
    float gyro_dps[3];     /**< 陀螺角速度 (°/s) */
} icm_scaled_t;

/* =============================================================================
 * 3) 公共 API
 * ============================================================================= */

/**
 * @brief  初始化：HAL 初始化 -> 软复位 -> WHO_AM_I 校验 -> (可选)等 MCLK 就绪
 * @return ICM_OK 成功；其他见 icm_err_t
 * @note   失败会进入 ERROR 状态。成功后状态为 RESET/IDLE，需再调用
 *         icm_enable_sensors() 进入 ACTIVE 才能采样。
 */
icm_err_t icm_init(void);

/**
 * @brief  使能加速度计 + 陀螺仪（低噪声模式，按 icm_cfg.h 的量程/ODR）
 * @return ICM_OK 成功；其他见 icm_err_t
 * @note   内部顺序遵循手册：先配置 CONFIG0，再写 PWR_MGMT0 切 LN，
 *         等待 200us（OFF->LN 写禁忌窗口），再按需等待陀螺稳定(45ms)。
 */
icm_err_t icm_enable_sensors(void);

/**
 * @brief  软复位（写 SIGNAL_PATH_RESET.SOFT_RESET，自清）
 */
icm_err_t icm_soft_reset(void);

/**
 * @brief  校验 WHO_AM_I（与 ICM_WHO_AM_I_VALUE 比对）
 */
icm_err_t icm_check_who_am_i(void);

/**
 * @brief  读单个寄存器
 * @param  reg : 寄存器地址(7bit)
 * @param  p_val : 输出（0~255）
 */
icm_err_t icm_read_reg(uint8_t reg, uint8_t *p_val);

/**
 * @brief  写单个寄存器
 */
icm_err_t icm_write_reg(uint8_t reg, uint8_t val);

/**
 * @brief  轮询数据就绪（INT_STATUS_DRDY.DATA_RDY_INT），超时返回 ICM_ERR_TIMEOUT
 * @param  timeout_ms : 超时（毫秒）
 */
icm_err_t icm_wait_data_ready(uint32_t timeout_ms);

/**
 * @brief  突发读取 温度+加速度+陀螺 共 14 字节（一次 SPI 事务，时间戳对齐）
 * @note   推荐积分解算用本接口，避免分次读造成的相位差。
 */
icm_err_t icm_read_all(icm_raw_t *raw);

#if ICM_KEEP_SEPARATE_READ
/** 单轴/单器件读取（兼容旧代码；内部各自一次短突发） */
icm_err_t icm_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
icm_err_t icm_read_gyro (int16_t *gx, int16_t *gy, int16_t *gz);
icm_err_t icm_read_temp (int16_t *raw_temp);
#endif

/**
 * @brief  原始 -> 物理量换算（按 cfg 中的分辨率）
 * @param  raw : 原始数据
 * @param  out : 输出物理量（temp_c / accel_g / gyro_dps）
 */
icm_err_t icm_convert(const icm_raw_t *raw, icm_scaled_t *out);

/**
 * @brief  设置陀螺零偏（dps），会用于后续解算的零偏扣除
 */
void icm_set_gyro_bias(float x_dps, float y_dps, float z_dps);

/**
 * @brief  读取当前保存的陀螺零偏（dps）
 */
void icm_get_gyro_bias(float *x_dps, float *y_dps, float *z_dps);

/**
 * @brief  静止水平放置时采集 n 个样本，求平均作为陀螺零偏（需器件静止）
 * @return ICM_OK 成功；ICM_ERR_STATE 未进入 ACTIVE
 */
icm_err_t icm_capture_gyro_bias(uint16_t n);

/**
 * @brief  获取当前驱动状态机状态
 */
icm_state_t icm_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM_CTRL_H */
