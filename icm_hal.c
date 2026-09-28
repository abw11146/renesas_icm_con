/**
 * @file    icm_hal.c
 * @brief   ICM 系列六轴 IMU 驱动 —— 硬件抽象层（HAL）模板【移植只需改本文件】
 *
 * @note    【本文件为 HAL 模板】
 *          移植时只需在下列 User Code 区段内填写平台代码；
 *          区段外内容由驱动维护者管理，升级驱动时不会被覆盖
 *          （配合带 User Code 标记的同步脚本，用户代码按区段名一一保留）。
 *
 * @note    【规则】
 *          - 每个函数体只包含一个 User Code 区段，区段名 == 函数名；
 *          - 文件顶部有一个 [includes] 文件级区段，用于放平台头文件；
 *          - 区段内的默认内容是可编译、可运行的 STUB（PC 可跑通自测）；
 *          - 用户把区段内内容替换为真实平台实现即可，**不要改区段外的代码**；
 *          - 本文件不提供任何 ICM_HAL_IMPL_* 平台条件编译分支，
 *            平台差异全部由用户在区段内消化。
 *
 * @note    【默认 STUB 行为】不接硬件，但模拟器件寄存器响应，
 *          使 PC 自测可真正跑通 init -> enable -> read_all -> convert -> 解算：
 *            - 0x00 MCLK_RDY  -> 0x08（MCLK 就绪）
 *            - 0x75 WHO_AM_I  -> 0x61（ICM-42607-C）
 *            - 0x39 INT_STAT  -> 0x01（数据就绪）
 *            - 0x09 起 14 字节 -> 静止水平放置（加速度 Z=2048 LSB≈1g，其余 0）
 *          滴答返回 0：此时请用 icm_est_update(e, s, dt) 显式传 dt，
 *          或在本文件 [icm_hal_get_tick_ms] 区段内返回真实毫秒滴答。
 *
 * @note    【SPI 约定（与数据手册一致）】4 线、Mode0/3、8bit、MSB first；
 *          CS 在 icm_hal_spi_xfer 内部管理（进入拉低、退出拉高），业务层零关心。
 */

#include "icm_hal.h"

/*< User Code >[includes]*/
/* 例如：#include "hal_data.h"        (Renesas RA / FSP)
          #include "r_spi_rx_if.h"      (Renesas RX)
          #include "drv/drv_systick.h"  (你的系统滴答)          */
/*< Code end >[includes]*/

/* =============================================================================
 * 1) 初始化底层 SPI 外设 + CS 置为无效(高)
 *    - RA(FSP): R_BSP_PinWrite(CS, HIGH); R_SPI_Open(&ctrl, &cfg);
 *    - RX     : 初始化 SCI/SPI 模块(8bit/MSB/Mode0或3)、CS 配为输出并拉高
 * ============================================================================= */
icm_err_t icm_hal_init(void)
{
    /*< User Code >[icm_hal_init]*/
    /* 默认空实现：已视为就绪（STUB）。请替换为你的平台初始化代码。 */
    return ICM_OK;
    /*< Code end >[icm_hal_init]*/
}

/* =============================================================================
 * 2) 单次全双工 SPI 传输（自动管理 CS）
 *    要求：进入拉低 CS -> 传输 len 字节 -> 退出拉高 CS；tx 不可为 NULL。
 * ============================================================================= */
icm_err_t icm_hal_spi_xfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    /*< User Code >[icm_hal_spi_xfer]*/
    /* ---- 默认 STUB：模拟器件寄存器响应（替换为真实 SPI 传输即可） ---- */
    /* 帧格式与真实器件一致：tx[0] bit7=1 表示读、bit6:0=寄存器地址。
       注：本区段不引用 icm_regs.h（HAL 只依赖 icm_cfg.h），故用字面量 0x80。 */
    #define ICM_STUB_READ_FLAG   (0x80U)   /* 读命令标志（= ICM_SPI_READ_FLAG） */
    #define ICM_STUB_REG_MCLK    (0x00U)   /* MCLK_RDY 区 */
    #define ICM_STUB_REG_DRDY    (0x39U)   /* INT_STATUS_DRDY */
    #define ICM_STUB_REG_WHOAMI  (0x75U)   /* WHO_AM_I */
    #define ICM_STUB_REG_DATA    (0x09U)   /* TEMP_DATA1 起始（连续 14 字节） */

    uint8_t  addr;
    uint16_t i;

    if (0U == len) { return ICM_OK; }
    if ((NULL == tx) || (NULL == rx)) { return ICM_ERR_PARAM; }

    /* 默认清零填充，避免复用缓冲残留干扰判断 */
    for (i = 0U; i < len; i++) { rx[i] = 0xFFU; }

    /* 仅读操作(bit7=1)才回读数据；写操作直接成功返回 */
    if (0U == (tx[0] & ICM_STUB_READ_FLAG)) { return ICM_OK; }

    addr = (uint8_t)(tx[0] & 0x7FU);

    if (ICM_STUB_REG_MCLK == addr)
    {
        if (len >= 2U) { rx[1] = 0x08U; }          /* bit3 MCLK_RDY = 1 */
    }
    else if (ICM_STUB_REG_WHOAMI == addr)
    {
        if (len >= 2U) { rx[1] = 0x61U; }          /* ICM-42607-C */
    }
    else if (ICM_STUB_REG_DRDY == addr)
    {
        if (len >= 2U) { rx[1] = 0x01U; }          /* DATA_RDY_INT */
    }
    else if (ICM_STUB_REG_DATA == addr)
    {
        /* 从 TEMP_DATA1(0x09) 起 14 字节：TEMP(2)+ACCX/Y/Z(6)+GYRX/Y/Z(6)，大端。
           模拟静止水平放置：温度 raw=0，加速度 Z=2048 LSB(≈1g@±16g)，其余 0。
           convert 后 accel Z ≈ 2048 * (16/32768) = 1.0 g，陀螺全 0。 */
        static const uint8_t sim[14] = {
            0x00U, 0x00U,        /* TEMP_DATA1, TEMP_DATA0  (raw=0)      */
            0x00U, 0x00U,        /* ACCEL_DATA_X1, X0       (0)          */
            0x00U, 0x00U,        /* ACCEL_DATA_Y1, Y0       (0)          */
            0x08U, 0x00U,        /* ACCEL_DATA_Z1, Z0       (2048=0x0800)*/
            0x00U, 0x00U,        /* GYRO_DATA_X1, X0        (0)          */
            0x00U, 0x00U,        /* GYRO_DATA_Y1, Y0        (0)          */
            0x00U, 0x00U         /* GYRO_DATA_Z1, Z0        (0)          */
        };
        for (i = 0U; (i < (uint16_t)(len - 1U)) && (i < 14U); i++)
        {
            rx[1U + i] = sim[i];
        }
    }
    else
    {
        /* 其它寄存器：回 0（状态寄存器等） */
        for (i = 1U; i < len; i++) { rx[i] = 0x00U; }
    }

    return ICM_OK;
    /*< Code end >[icm_hal_spi_xfer]*/
}

/* =============================================================================
 * 3) 微秒级延时（用于 OFF->LN 后的 200us 禁写窗口，精度 ±50% 内即可）
 * ============================================================================= */
void icm_hal_delay_us(uint32_t us)
{
    /*< User Code >[icm_hal_delay_us]*/
    /* 默认空实现（STUB）。请替换为：硬件定时器 / RTOS 微秒延时 / 校准空转循环。 */
    (void)us;
    /*< Code end >[icm_hal_delay_us]*/
}

/* =============================================================================
 * 4) 毫秒级延时（软复位后等待、陀螺稳定等待、轮询间隔）
 * ============================================================================= */
void icm_hal_delay_ms(uint32_t ms)
{
    /*< User Code >[icm_hal_delay_ms]*/
    /* 默认空实现（STUB）。请替换为：RTOS 延时 / 系统 SysTick 延时 / 校准空转。 */
    (void)ms;
    /*< Code end >[icm_hal_delay_ms]*/
}

/* =============================================================================
 * 5) 系统毫秒滴答（供积分自动计算 dt）
 *    无滴答平台可保持返回 0，此时请改用 icm_est_update(e, s, dt) 显式传 dt。
 * ============================================================================= */
uint32_t icm_hal_get_tick_ms(void)
{
    /*< User Code >[icm_hal_get_tick_ms]*/
    /* 默认返回 0（表示本平台无滴答）。请替换为你的系统毫秒计数，
       例如：return g_system_tick_ms;  或  return drv_systick_get_ms(); */
    return 0U;
    /*< Code end >[icm_hal_get_tick_ms]*/
}
