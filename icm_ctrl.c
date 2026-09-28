/**
 * @file    icm_ctrl.c
 * @brief   ICM 系列六轴 IMU 驱动 —— 控制层实现（状态机 + 寄存器读写 + 原始/物理量换算）
 *
 * @note    仅依赖 HAL（icm_hal.c）与配置/规格（icm_cfg.h / icm_regs.h），
 *          不引用任何 MCU 外设。逻辑与原 icm42607_ctrl.c 完全一致，仅更新前缀。
 */

#include "icm_ctrl.h"
#include <string.h>

/* -----------------------------------------------------------------------------
 * 内部状态（仅本文件可见，最小 RAM 占用）
 * --------------------------------------------------------------------------- */
static icm_state_t g_state = ICM_STATE_UNINIT;
static float g_gyro_bias_dps[3] = { 0.0f, 0.0f, 0.0f };  /**< 陀螺零偏(dps) */

/* =============================================================================
 * 内部工具
 * ============================================================================= */

/**
 * @brief  业务入口状态校验：当前状态「非 ERROR」且「不低于 min_state」
 * @note   枚举里 ICM_STATE_ERROR 排在 ACTIVE 之后，直接用 `g_state < X`
 *         会在 ERROR 状态下误判为「已满足」（因为 ERROR 的数值更大）。
 *         因此所有业务入口统一走本函数，而非简单的 `<` 比较。
 */
static bool prv_state_ok(icm_state_t min_state)
{
    return (g_state != ICM_STATE_ERROR) && (g_state >= min_state);
}

/**
 * @brief  等待 MCLK 就绪（MCLK_RDY 寄存器 0x00 bit3 = 1）
 * @note   手册未强制，但软复位/上电后轮询一次可避免提前配置导致的不稳定。
 *         仅当 ICM_CHECK_MCLK_RDY=1 时才会被 icm_init 调用，故一并做条件编译，
 *         避免关闭该宏后产生「defined but not used」告警。
 */
#if ICM_CHECK_MCLK_RDY
static icm_err_t prv_wait_mclk_ready(uint32_t timeout_ms)
{
    uint8_t val = 0U;
    uint32_t t = timeout_ms;
    do
    {
        icm_err_t r = icm_read_reg(ICM_REG_MCLK_RDY, &val);
        if (ICM_OK != r) { return r; }
        if (0U != (val & ICM_MCLK_RDY_BIT)) { return ICM_OK; }  /* bit3 MCLK_RDY */
        icm_hal_delay_ms(1U);
    } while (t-- != 0U);
    return ICM_ERR_TIMEOUT;
}
#endif /* ICM_CHECK_MCLK_RDY */

/* =============================================================================
 * 寄存器读写（公开，也可被业务层直接调用）
 * @note  read_reg / write_reg 有意**不做状态机校验**：便于在任何状态下做调试
 *        （读 WHO_AM_I、MCLK_RDY、软复位等）。业务数据路径请走带校验的接口。
 * ============================================================================= */
icm_err_t icm_read_reg(uint8_t reg, uint8_t *p_val)
{
    uint8_t tx[2] = { (uint8_t)(reg | ICM_SPI_READ_FLAG), 0xFFU };
    uint8_t rx[2] = { 0U, 0U };
    icm_err_t ret;

    if (NULL == p_val) { return ICM_ERR_PARAM; }

    ret = icm_hal_spi_xfer(tx, rx, 2U);
    if (ICM_OK != ret) { return ret; }

    *p_val = rx[1];
    return ICM_OK;
}

icm_err_t icm_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    uint8_t rx[2] = { 0U, 0U };
    return icm_hal_spi_xfer(tx, rx, 2U);
}

/* =============================================================================
 * 初始化 / 配置
 * ============================================================================= */
icm_err_t icm_init(void)
{
    icm_err_t ret;
    uint8_t dev_cfg = 0U;

    g_state = ICM_STATE_UNINIT;

    ret = icm_hal_init();
    if (ICM_OK != ret) { g_state = ICM_STATE_ERROR; return ret; }

    /* 软复位（自清位） */
    ret = icm_soft_reset();
    if (ICM_OK != ret) { g_state = ICM_STATE_ERROR; return ret; }

    /* WHO_AM_I 校验 */
    ret = icm_check_who_am_i();
    if (ICM_OK != ret) { g_state = ICM_STATE_ERROR; return ret; }

    /* SPI 接口配置：4 线 + Mode0/3（按 cfg/regs） */
    if (ICM_SPI_4WIRE)     { dev_cfg |= ICM_DEVICE_CONFIG_SPI_AP_4WIRE; }
    if (!ICM_SPI_MODE_0_3) { dev_cfg |= ICM_DEVICE_CONFIG_SPI_MODE; }
    ret = icm_write_reg(ICM_REG_DEVICE_CONFIG, dev_cfg);
    if (ICM_OK != ret) { g_state = ICM_STATE_ERROR; return ret; }

    /* 等待内部时钟稳定 */
#if ICM_CHECK_MCLK_RDY
    ret = prv_wait_mclk_ready(ICM_MCLK_RDY_TIMEOUT_MS);
    if (ICM_OK != ret) { g_state = ICM_STATE_ERROR; return ret; }
#endif

    g_state = ICM_STATE_RESET;
    return ICM_OK;
}

icm_err_t icm_soft_reset(void)
{
    icm_err_t ret = icm_write_reg(ICM_REG_SIGNAL_PATH_RESET,
                                  ICM_SIGNAL_PATH_RESET_SOFT);
    if (ICM_OK != ret) { return ret; }
    icm_hal_delay_ms(ICM_SOFT_RESET_MS);
    return ICM_OK;
}

icm_err_t icm_check_who_am_i(void)
{
    uint8_t who = 0U;
    icm_err_t ret = icm_read_reg(ICM_REG_WHO_AM_I, &who);
    if (ICM_OK != ret) { return ret; }
    if (who != ICM_WHO_AM_I_VALUE) { return ICM_ERR_WHOAMI; }
    return ICM_OK;
}

icm_err_t icm_enable_sensors(void)
{
    icm_err_t ret;
    uint8_t val;

    if (!prv_state_ok(ICM_STATE_RESET)) { return ICM_ERR_STATE; }

    /* 1) 陀螺：量程 + ODR（位域：FS_SEL 在 bit6:5，ODR 在 bit3:0）
       注：手册中 FS_SEL 为 3 位域，掩码用 0x07 更严谨（当前取值 < 4 不受影响）。 */
    val = (uint8_t)(((ICM_GYRO_FS_SEL & ICM_CONFIG0_FS_SEL_MASK) << ICM_CONFIG0_FS_SEL_SHIFT) |
                    (ICM_GYRO_ODR & ICM_CONFIG0_ODR_MASK));
    ret = icm_write_reg(ICM_REG_GYRO_CONFIG0, val);
    if (ICM_OK != ret) { g_state = ICM_STATE_ERROR; return ret; }

    /* 2) 加速度：量程 + ODR */
    val = (uint8_t)(((ICM_ACCEL_FS_SEL & ICM_CONFIG0_FS_SEL_MASK) << ICM_CONFIG0_FS_SEL_SHIFT) |
                    (ICM_ACCEL_ODR & ICM_CONFIG0_ODR_MASK));
    ret = icm_write_reg(ICM_REG_ACCEL_CONFIG0, val);
    if (ICM_OK != ret) { g_state = ICM_STATE_ERROR; return ret; }

    /* 3) 上电：加速度 + 陀螺 都进入低噪声(LN)模式 */
    val = (uint8_t)(((ICM_PWR_MGMT0_GYRO_MODE_LN & ICM_PWR_MGMT0_MODE_MASK) << ICM_PWR_MGMT0_GYRO_SHIFT) |
                    ((ICM_PWR_MGMT0_ACCEL_MODE_LN & ICM_PWR_MGMT0_MODE_MASK) << ICM_PWR_MGMT0_ACCEL_SHIFT));
    ret = icm_write_reg(ICM_REG_PWR_MGMT0, val);
    if (ICM_OK != ret) { g_state = ICM_STATE_ERROR; return ret; }

    /* 4) OFF->LN 切换后 200us 内禁止写寄存器（手册 53 页） */
    icm_hal_delay_us(ICM_PWR_ON_SETTLE_US);

    /* 5) 陀螺上电需稳定 45ms 数据才可信（手册 53 页） */
#if ICM_WAIT_GYRO_RDY
    icm_hal_delay_ms(ICM_GYRO_RDY_MS);
#endif

    g_state = ICM_STATE_ACTIVE;
    return ICM_OK;
}

/* =============================================================================
 * 数据就绪 / 读取
 * ============================================================================= */
icm_err_t icm_wait_data_ready(uint32_t timeout_ms)
{
    uint8_t val = 0U;
    uint32_t t = timeout_ms;
    do
    {
        icm_err_t r = icm_read_reg(ICM_REG_INT_STATUS_DRDY, &val);
        if (ICM_OK != r) { return r; }
        if (0U != (val & ICM_DRDY_INT_BIT)) { return ICM_OK; }
        icm_hal_delay_ms(1U);
    } while (t-- != 0U);
    return ICM_ERR_TIMEOUT;
}

icm_err_t icm_read_all(icm_raw_t *raw)
{
    uint8_t tx[ICM_BURST_TX_LEN];
    uint8_t rx[ICM_BURST_TX_LEN] = { 0U };
    icm_err_t ret;

    if (NULL == raw) { return ICM_ERR_PARAM; }
    if (!prv_state_ok(ICM_STATE_ACTIVE)) { return ICM_ERR_STATE; }

    tx[0] = (uint8_t)(ICM_SPI_READ_FLAG | ICM_REG_TEMP_DATA1);
    for (uint8_t i = 1U; i < ICM_BURST_TX_LEN; i++) { tx[i] = 0xFFU; }

    ret = icm_hal_spi_xfer(tx, rx, ICM_BURST_TX_LEN);
    if (ICM_OK != ret) { return ret; }

    /* rx[0]=dummy；随后 14 字节 = TEMP(2)+ACCEL(6)+GYRO(6) */
    raw->temp        = (int16_t)(((uint16_t)rx[1]  << 8U) | rx[2]);
    raw->accel[0]    = (int16_t)(((uint16_t)rx[3]  << 8U) | rx[4]);
    raw->accel[1]    = (int16_t)(((uint16_t)rx[5]  << 8U) | rx[6]);
    raw->accel[2]    = (int16_t)(((uint16_t)rx[7]  << 8U) | rx[8]);
    raw->gyro[0]     = (int16_t)(((uint16_t)rx[9]  << 8U) | rx[10]);
    raw->gyro[1]     = (int16_t)(((uint16_t)rx[11] << 8U) | rx[12]);
    raw->gyro[2]     = (int16_t)(((uint16_t)rx[13] << 8U) | rx[14]);
    return ICM_OK;
}

#if ICM_KEEP_SEPARATE_READ
icm_err_t icm_read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t tx[7]; uint8_t rx[7] = {0U};
    icm_err_t ret;
    if ((NULL==ax)||(NULL==ay)||(NULL==az)) { return ICM_ERR_PARAM; }
    if (!prv_state_ok(ICM_STATE_ACTIVE)) { return ICM_ERR_STATE; }
    tx[0] = (uint8_t)(ICM_SPI_READ_FLAG | ICM_REG_ACCEL_DATA_X1);
    for (uint8_t i=1U;i<7U;i++) tx[i]=0xFFU;
    ret = icm_hal_spi_xfer(tx, rx, 7U);
    if (ICM_OK != ret) return ret;
    *ax = (int16_t)(((uint16_t)rx[1]<<8U)|rx[2]);
    *ay = (int16_t)(((uint16_t)rx[3]<<8U)|rx[4]);
    *az = (int16_t)(((uint16_t)rx[5]<<8U)|rx[6]);
    return ICM_OK;
}

icm_err_t icm_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t tx[7]; uint8_t rx[7] = {0U};
    icm_err_t ret;
    if ((NULL==gx)||(NULL==gy)||(NULL==gz)) { return ICM_ERR_PARAM; }
    if (!prv_state_ok(ICM_STATE_ACTIVE)) { return ICM_ERR_STATE; }
    tx[0] = (uint8_t)(ICM_SPI_READ_FLAG | ICM_REG_GYRO_DATA_X1);
    for (uint8_t i=1U;i<7U;i++) tx[i]=0xFFU;
    ret = icm_hal_spi_xfer(tx, rx, 7U);
    if (ICM_OK != ret) return ret;
    *gx = (int16_t)(((uint16_t)rx[1]<<8U)|rx[2]);
    *gy = (int16_t)(((uint16_t)rx[3]<<8U)|rx[4]);
    *gz = (int16_t)(((uint16_t)rx[5]<<8U)|rx[6]);
    return ICM_OK;
}

icm_err_t icm_read_temp(int16_t *raw_temp)
{
    uint8_t tx[3]; uint8_t rx[3] = {0U};
    icm_err_t ret;
    if (NULL==raw_temp) { return ICM_ERR_PARAM; }
    if (!prv_state_ok(ICM_STATE_ACTIVE)) { return ICM_ERR_STATE; }
    tx[0] = (uint8_t)(ICM_SPI_READ_FLAG | ICM_REG_TEMP_DATA1);
    tx[1]=0xFFU; tx[2]=0xFFU;
    ret = icm_hal_spi_xfer(tx, rx, 3U);
    if (ICM_OK != ret) return ret;
    *raw_temp = (int16_t)(((uint16_t)rx[1]<<8U)|rx[2]);
    return ICM_OK;
}
#endif /* KEEP_SEPARATE_READ */

/* =============================================================================
 * 换算 / 零偏
 * ============================================================================= */
icm_err_t icm_convert(const icm_raw_t *raw, icm_scaled_t *out)
{
    uint8_t i;
    if ((NULL == raw) || (NULL == out)) { return ICM_ERR_PARAM; }

    out->temp_c = ((float)raw->temp / ICM_TEMP_DIV) + ICM_TEMP_OFFSET_C;
    for (i = 0U; i < 3U; i++)
    {
        out->accel_g[i]  = (float)raw->accel[i]  * ICM_ACCEL_RES_G;
        out->gyro_dps[i] = (float)raw->gyro[i]   * ICM_GYRO_RES_DPS;
    }
    return ICM_OK;
}

void icm_set_gyro_bias(float x_dps, float y_dps, float z_dps)
{
    g_gyro_bias_dps[0] = x_dps;
    g_gyro_bias_dps[1] = y_dps;
    g_gyro_bias_dps[2] = z_dps;
}

void icm_get_gyro_bias(float *x_dps, float *y_dps, float *z_dps)
{
    if (NULL != x_dps) *x_dps = g_gyro_bias_dps[0];
    if (NULL != y_dps) *y_dps = g_gyro_bias_dps[1];
    if (NULL != z_dps) *z_dps = g_gyro_bias_dps[2];
}

icm_err_t icm_capture_gyro_bias(uint16_t n)
{
    uint16_t i;
    float acc[3] = {0.0f, 0.0f, 0.0f};
    icm_raw_t raw;
    icm_err_t ret;

    if (!prv_state_ok(ICM_STATE_ACTIVE)) { return ICM_ERR_STATE; }
    if (0U == n) { return ICM_ERR_PARAM; }

    for (i = 0U; i < n; i++)
    {
        ret = icm_read_all(&raw);
        if (ICM_OK != ret) { return ret; }
        acc[0] += (float)raw.gyro[0] * ICM_GYRO_RES_DPS;
        acc[1] += (float)raw.gyro[1] * ICM_GYRO_RES_DPS;
        acc[2] += (float)raw.gyro[2] * ICM_GYRO_RES_DPS;
    }
    icm_set_gyro_bias(acc[0] / (float)n, acc[1] / (float)n, acc[2] / (float)n);
    return ICM_OK;
}

icm_state_t icm_get_state(void)
{
    return g_state;
}
