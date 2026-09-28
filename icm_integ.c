/**
 * @file    icm_integ.c
 * @brief   ICM 系列六轴 IMU 驱动 —— 六轴积分解算实现
 *
 * @note    算法：四元数陀螺积分 + Mahony 倾角校正 + 重力补偿 + 速度/位置积分 + ZUPT。
 *          全程 float，仅用 sqrtf / atan2f / asinf / fabsf，无矩阵求逆，开销极小。
 *          逻辑与原 icm42607_integ.c 完全一致，仅更新前缀与 include。
 *
 * @note    本文件**不直接包含 icm_regs.h**：算法层只依赖 icm_ctrl.h 暴露的
 *          icm_scaled_t 与读取接口，寄存器映射属器件控制细节，算法层无需感知。
 *
 * @note    Mahony 叉积符号验证方法（见 icm_est_update 内注释）：
 *          把器件静止水平放置，连续调用 est_update 若干帧：
 *            - roll/pitch 应收敛到 0（不发散）→ 当前 prv_cross(v_meas,v_pred,err) 正确；
 *            - 若 roll/pitch 反而增大发散 → 改为 prv_cross(v_pred,v_meas,err) 或 err 取反。
 */

#include "icm_integ.h"
#include <math.h>
#include <string.h>

#if ICM_INTEG_ENABLE

/* =============================================================================
 * 内部小工具
 * ============================================================================= */

/** 向量叉积 c = a × b */
static inline void prv_cross(const float a[3], const float b[3], float c[3])
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

/** 向量点积 */
// static inline float prv_dot(const float a[3], const float b[3])
// {
//     return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
// }

/**
 * @brief  用四元数 q 旋转向量 v（机体->世界，Hamilton，标量在前）
 * @note  公式：v' = v + 2*q0*(qv×v) + 2*(qv×(qv×v))，无三角函数，极快。
 */
static inline void prv_quat_rot(const float q[4], const float v[3], float out[3])
{
    float qx = q[1], qy = q[2], qz = q[3], qw = q[0];
    float ux = qy*v[2] - qz*v[1];
    float uy = qz*v[0] - qx*v[2];
    float uz = qx*v[1] - qy*v[0];
    /* 2*q0*(qv×v) */
    float tx = 2.0f * qw * ux;
    float ty = 2.0f * qw * uy;
    float tz = 2.0f * qw * uz;
    /* 2*(qv×(qv×v)) */
    float vx2 = qy*uz - qz*uy;
    float vy2 = qz*ux - qx*uz;
    float vz2 = qx*uy - qy*ux;
    tx += 2.0f * vx2; ty += 2.0f * vy2; tz += 2.0f * vz2;
    out[0] = v[0] + tx;
    out[1] = v[1] + ty;
    out[2] = v[2] + tz;
}

/** 由四元数求 roll/pitch/yaw (rad) */
static inline void prv_euler_from_q(const float q[4], float e[3])
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float s;
    e[0] = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)); /* roll  */
    /* 【加固】pitch 用 asinf，其定义域为 [-1,1]；四元数数值误差可能使入参
       略微越界(如 1.0000001)导致返回 NaN，这里先限幅再求值。 */
    s = 2.0f*(q0*q2 - q3*q1);
    if (s >  1.0f) { s =  1.0f; }
    if (s < -1.0f) { s = -1.0f; }
    e[1] = asinf(s);                                                  /* pitch */
    e[2] = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)); /* yaw   */
}

/* =============================================================================
 * API 实现
 * ============================================================================= */
void icm_est_init(icm_estimator_t *e)
{
    if (NULL == e) { return; }
    memset(e, 0, sizeof(*e));
    e->q[0] = 1.0f;          /* 单位四元数：世界系 == 机体系 */
    e->inited = 1U;
    e->first_frame = 1U;     /* 第一帧仅记录时间戳，见 update_auto */
}

void icm_est_reset(icm_estimator_t *e)
{
    if (NULL == e) { return; }
    memset(e, 0, sizeof(*e));
    e->q[0] = 1.0f;
    e->inited = 1U;
    e->first_frame = 1U;
}

void icm_est_set_gyro_bias(icm_estimator_t *e, float x, float y, float z)
{
    if (NULL == e) { return; }
    e->gyro_bias[0] = x; e->gyro_bias[1] = y; e->gyro_bias[2] = z;
}

void icm_est_get_gyro_bias(const icm_estimator_t *e, float *x, float *y, float *z)
{
    if (NULL == e) { return; }
    if (NULL != x) *x = e->gyro_bias[0];
    if (NULL != y) *y = e->gyro_bias[1];
    if (NULL != z) *z = e->gyro_bias[2];
}

icm_err_t icm_est_capture_gyro_bias(icm_estimator_t *e, uint16_t n)
{
    float x, y, z;
    icm_err_t ret;

    if (NULL == e) { return ICM_ERR_PARAM; }

    /* 【去重】复用控制层的标定实现，避免两处维护同一逻辑（均值/异常返回保持一致）。
       ctrl 版把结果存入其内部静态零偏，这里再同步到估计器。 */
    ret = icm_capture_gyro_bias(n);
    if (ICM_OK != ret) { return ret; }

    icm_get_gyro_bias(&x, &y, &z);
    icm_est_set_gyro_bias(e, x, y, z);
    return ICM_OK;
}

icm_err_t icm_est_update(icm_estimator_t *e,
                         const icm_scaled_t *s, float dt)
{
    float wx, wy, wz;
    float q0, q1, q2, q3, nrm;
#if ICM_EST_TILT_CORRECT || ICM_EST_ZUPT_ENABLE || ICM_EST_VEL_ENABLE
    float ax, ay, az;
#endif
#if ICM_EST_TILT_CORRECT || ICM_EST_ZUPT_ENABLE
    float an;
#endif
#if ICM_EST_VEL_ENABLE
    float f_mps2[3], a_world[3];
    uint8_t i;
#endif
#if ICM_EST_TILT_CORRECT
    float v_meas[3], v_pred[3], err[3];
#endif

    if ((NULL == e) || (NULL == s)) { return ICM_ERR_PARAM; }
    if (0U == e->inited)            { return ICM_ERR_STATE; }
    if ((dt <= 0.0f) || (dt > ICM_DT_MAX)) { return ICM_ERR_PARAM; }

    /* 1) 角速度(rad/s)：扣除零偏 */
    wx = (s->gyro_dps[0] - e->gyro_bias[0]) * ICM_DEG2RAD;
    wy = (s->gyro_dps[1] - e->gyro_bias[1]) * ICM_DEG2RAD;
    wz = (s->gyro_dps[2] - e->gyro_bias[2]) * ICM_DEG2RAD;

#if ICM_EST_TILT_CORRECT || ICM_EST_ZUPT_ENABLE || ICM_EST_VEL_ENABLE
    ax = s->accel_g[0]; ay = s->accel_g[1]; az = s->accel_g[2];
#endif
#if ICM_EST_TILT_CORRECT || ICM_EST_ZUPT_ENABLE
    an = sqrtf(ax*ax + ay*ay + az*az);   /* 加速度模长(g)，静止应≈1 */
#endif

    /* 2) 加速度计倾角校正（Mahony）：用「上方向」观测量修正角速度 */
#if ICM_EST_TILT_CORRECT
    if (an > 1e-4f)
    {
        /* 测量到的「上」方向（机体）：加速度归一化后约等于比力方向 */
        v_meas[0] = ax / an; v_meas[1] = ay / an; v_meas[2] = az / an;
        /* 由当前姿态预测机体下的重力方向 = R^T * [0,0,1]，即用 q 的共轭旋转 [0,0,1]。
           静止水平时该预测应与测量一致，叉积误差≈0；倾斜时误差驱动角速度修正。 */
        float qc[4] = { e->q[0], -e->q[1], -e->q[2], -e->q[3] };
        const float up[3] = {0.0f, 0.0f, 1.0f};
        prv_quat_rot(qc, up, v_pred);
        /* 误差 = 测量 × 预测。
           符号依据：err = v_meas × v_pred；当估计姿态正确时两者共线、叉积≈0。
           若实测「静止水平放置时 roll/pitch 不收敛反而发散」，说明符号相反，
           改为 prv_cross(v_pred, v_meas, err) 或对 err 取反即可（验证方法见文件头）。 */
        prv_cross(v_meas, v_pred, err);
        wx += ICM_EST_KP * err[0] + e->w_int[0];
        wy += ICM_EST_KP * err[1] + e->w_int[1];
        wz += ICM_EST_KP * err[2] + e->w_int[2];
        /* KI 积分项：ICM_EST_KI=0 时恒为 0，无副作用（保留以便后续微调） */
        e->w_int[0] += ICM_EST_KI * err[0] * dt;
        e->w_int[1] += ICM_EST_KI * err[1] * dt;
        e->w_int[2] += ICM_EST_KI * err[2] * dt;
    }
#endif

    /* 3) 四元数积分（一阶，小角近似）: q_dot = 0.5 * q ⊗ (0, w)
       注意：必须先用「同一时刻的旧值」算出四个导数 q0d~q3d，
             再统一更新 q0~q3。若边更新边用（先算 q0 再用新 q0 算 q1），
             相当于隐式使用不同时刻分量，会引入额外误差、破坏精度。 */
    q0 = e->q[0]; q1 = e->q[1]; q2 = e->q[2]; q3 = e->q[3];
    {
        float q0d = -0.5f*(q1*wx + q2*wy + q3*wz);
        float q1d =  0.5f*(q0*wx + q2*wz - q3*wy);
        float q2d =  0.5f*(q0*wy - q1*wz + q3*wx);
        float q3d =  0.5f*(q0*wz + q1*wy - q2*wx);
        q0 += q0d * dt;
        q1 += q1d * dt;
        q2 += q2d * dt;
        q3 += q3d * dt;
    }
    nrm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (nrm > 1e-8f) { q0 /= nrm; q1 /= nrm; q2 /= nrm; q3 /= nrm; }
    e->q[0] = q0; e->q[1] = q1; e->q[2] = q2; e->q[3] = q3;
    prv_euler_from_q(e->q, e->euler);

    /* 4) 重力补偿：a_world = R * f_b + g_world(=[0,0,-G])
           （仅速度/位置积分需要；纯姿态应用不计算 a_world，省算力） */
#if ICM_EST_VEL_ENABLE
#if ICM_EST_GRAVITY_COMP
    f_mps2[0] = ax * ICM_GRAVITY;
    f_mps2[1] = ay * ICM_GRAVITY;
    f_mps2[2] = az * ICM_GRAVITY;
    prv_quat_rot(e->q, f_mps2, a_world);
    a_world[2] -= ICM_GRAVITY;   /* 抵消重力(世界系 z 向上) => 线加速度 */
#else
    /* 不做重力补偿：直接输出比力（含重力），用于纯姿态/方向用途 */
    f_mps2[0] = ax * ICM_GRAVITY;
    f_mps2[1] = ay * ICM_GRAVITY;
    f_mps2[2] = az * ICM_GRAVITY;
    prv_quat_rot(e->q, f_mps2, a_world);
#endif
#endif /* VEL_ENABLE */

    /* 5) 速度积分 + 静止检测(ZUPT) */
#if ICM_EST_VEL_ENABLE
    for (i = 0U; i < 3U; i++)
    {
        e->vel[i] += a_world[i] * dt;
        if (e->vel[i] >  ICM_VEL_MAX) e->vel[i] =  ICM_VEL_MAX;
        if (e->vel[i] < -ICM_VEL_MAX) e->vel[i] = -ICM_VEL_MAX;
    }
#if ICM_EST_ZUPT_ENABLE
    /* 角速度很小 且 加速度模长≈1g => 静止，速度归零，抑制纯惯性漂移 */
    {
        float wmag = sqrtf(wx*wx + wy*wy + wz*wz);
        if ((wmag < ICM_ZUPT_GYRO_DEV) &&
            (fabsf(an - 1.0f) < ICM_ZUPT_ACC_DEV))
        {
            e->vel[0] = 0.0f; e->vel[1] = 0.0f; e->vel[2] = 0.0f;
        }
    }
#endif
    /* 6) 位置积分 */
#if ICM_EST_POS_ENABLE
    for (i = 0U; i < 3U; i++)
    {
        e->pos[i] += e->vel[i] * dt;
    }
#endif
#endif

    return ICM_OK;
}

icm_err_t icm_est_update_auto(icm_estimator_t *e,
                              const icm_scaled_t *s)
{
    uint32_t now;
    float dt;

    if ((NULL == e) || (NULL == s)) { return ICM_ERR_PARAM; }
    if (0U == e->inited)            { return ICM_ERR_STATE; }

    now = icm_hal_get_tick_ms();

    /* 首帧：不积分，仅记录时间基准（用 first_frame 标志，而非 last_tick==0，
       否则在「滴答恒为 0」的平台上会永远判为首帧，导致永不积分）。 */
    if (0U != e->first_frame)
    {
        e->first_frame = 0U;
        e->last_tick = now;
        return ICM_OK;
    }

    /* 运行期检测到 HAL 未提供滴答（now 与上次均恒为 0）：无法算 dt，报错提示。
       此时请改用 icm_est_update(e, s, dt) 由调用方显式传入 dt。 */
    if ((0U == now) && (0U == e->last_tick))
    {
        return ICM_ERR_HAL;
    }

    dt = (float)(now - e->last_tick) * 0.001f;
    e->last_tick = now;
    if (dt <= 0.0f) { return ICM_OK; }   /* 同一时刻/回绕，不积分本帧 */
    return icm_est_update(e, s, dt);
}

void icm_est_get_quat(const icm_estimator_t *e, float q[4])
{
    if ((NULL == e) || (NULL == q)) { return; }
    q[0] = e->q[0]; q[1] = e->q[1]; q[2] = e->q[2]; q[3] = e->q[3];
}

void icm_est_get_euler(const icm_estimator_t *e, float euler_rad[3])
{
    if ((NULL == e) || (NULL == euler_rad)) { return; }
    euler_rad[0] = e->euler[0]; euler_rad[1] = e->euler[1]; euler_rad[2] = e->euler[2];
}

void icm_est_get_velocity(const icm_estimator_t *e, float v[3])
{
    if ((NULL == e) || (NULL == v)) { return; }
    v[0] = e->vel[0]; v[1] = e->vel[1]; v[2] = e->vel[2];
}

void icm_est_get_position(const icm_estimator_t *e, float p[3])
{
    if ((NULL == e) || (NULL == p)) { return; }
    p[0] = e->pos[0]; p[1] = e->pos[1]; p[2] = e->pos[2];
}

#endif /* ICM_INTEG_ENABLE */
