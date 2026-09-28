/**
 * @file    icm_integ.h
 * @brief   ICM 系列六轴 IMU 驱动 —— 六轴积分解算（姿态 / 速度 / 位置）
 *
 * @note    【本层定位】算法层。只依赖 icm_ctrl.h（取 icm_scaled_t 与读取接口），
 *          **不直接依赖 icm_regs.h**——寄存器属于器件控制细节，算法层无需感知。
 *
 * @note    解算链路（详见 icm_integ.c）：
 *          1) 陀螺角速度 -> 四元数积分，得到姿态(q / roll,pitch,yaw)；
 *          2) 加速度计对「上方向」的观测 -> Mahony 互补校正，抑制 roll/pitch 漂移；
 *             （注：无磁力计，yaw 仅由陀螺积分，会缓慢漂移，属正常现象）
 *          3) 扣除重力得到「线加速度」，积分得速度、再积分得位置（惯性导航）；
 *          4) 静止检测(ZUPT)在设备不运动时把速度归零，显著抑制纯惯性积分的爆炸式漂移。
 *
 * @note    数值：使用 float + 一阶四元数更新（采样率 >=25Hz 时每步转角极小，
 *          一阶已足够准），全程**无 sin/cos/矩阵求逆**，仅 1~2 次 sqrtf，
 *          CPU/Flash 占用极低。无 FPU 的 MCU 用软浮点即可运行。
 */

#ifndef ICM_INTEG_H
#define ICM_INTEG_H

#include "icm_ctrl.h"   /* 用到 icm_scaled_t / 读取接口 */

#ifdef __cplusplus
extern "C" {
#endif

#if ICM_INTEG_ENABLE

/** 六轴状态估计器（解算上下文） */
typedef struct
{
    float q[4];            /**< 姿态四元数 [q0,q1,q2,q3]，标量在前，单位四元数 */
    float euler[3];        /**< 由 q 推导的姿态角 roll/pitch/yaw (rad) */
    float vel[3];          /**< 世界系速度 (m/s) */
    float pos[3];          /**< 世界系位置 (m) */
    float gyro_bias[3];    /**< 陀螺零偏 (dps)，解算时扣除 */
    float w_int[3];        /**< Mahony 积分项(KI)，默认未启用(=0) */
    uint32_t last_tick;    /**< 上次 update 的系统滴答(ms)，用于自动 dt */
    uint8_t  inited;       /**< 是否已初始化 */
    uint8_t  first_frame;  /**< update_auto 首帧标志：1=下一帧仅记录时间戳、不积分。
                                【修复】原先用 last_tick==0 判首帧，在无滴答平台
                                (get_tick_ms 恒 0) 下会永远停在首帧分支、永不积分。 */
} icm_estimator_t;

/* ------------------------------------------------------------------ API -- */

/** 初始化 / 复位估计器：q=单位四元数，速度/位置=0 */
void icm_est_init(icm_estimator_t *e);
void icm_est_reset(icm_estimator_t *e);

/** 设置 / 获取陀螺零偏(dps) */
void icm_est_set_gyro_bias(icm_estimator_t *e, float x, float y, float z);
void icm_est_get_gyro_bias(const icm_estimator_t *e, float *x, float *y, float *z);

/**
 * @brief  静止水平放置时采集 n 个样本求平均，作为陀螺零偏
 * @return ICM_OK 成功；ICM_ERR_STATE 未进入 ACTIVE
 * @note   务必在标定期间保持器件静止；标定后调用方无需再手动 set_bias。
 */
icm_err_t icm_est_capture_gyro_bias(icm_estimator_t *e, uint16_t n);

/**
 * @brief  核心：用一帧物理量做一次积分解算
 * @param  e  : 估计器上下文
 * @param  s  : 本帧物理量（已换算：temp_c/accel_g/gyro_dps）
 * @param  dt : 距上一帧的时间(s)，需 >0 且 <= ICM_DT_MAX
 * @return ICM_OK 成功；ICM_ERR_PARAM dt 非法或指针空；ICM_ERR_STATE 未初始化
 */
icm_err_t icm_est_update(icm_estimator_t *e,
                         const icm_scaled_t *s, float dt);

/**
 * @brief  同上，但 dt 由 HAL 系统滴答自动计算（推荐：最省心、且带超时保护）
 */
icm_err_t icm_est_update_auto(icm_estimator_t *e,
                              const icm_scaled_t *s);

/* 取值接口 */
void icm_est_get_quat(const icm_estimator_t *e, float q[4]);
void icm_est_get_euler(const icm_estimator_t *e, float euler_rad[3]);
void icm_est_get_velocity(const icm_estimator_t *e, float v[3]);
void icm_est_get_position(const icm_estimator_t *e, float p[3]);

#endif /* ICM_INTEG_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* ICM_INTEG_H */
