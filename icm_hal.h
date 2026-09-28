/**
 * @file    icm_hal.h
 * @brief   ICM 系列六轴 IMU 驱动 —— 硬件抽象层（HAL）接口声明
 *
 * @note    【本层定位】HAL 接口声明。**跨平台移植唯一需要实现的层**。
 *          业务代码（icm_ctrl / icm_integ）完全不碰任何 MCU 寄存器，
 *          只调用下面这 5 个函数。移植到 Renesas RA(FSP) / RX 或任意 MCU 时，
 *          只需在 icm_hal.c 的 User Code 区段内实现这 5 个函数，
 *          其余文件（cfg / regs / ctrl / integ）**一行都不用改**。
 *
 * @note    【与 icm_hal.c 的对应关系】icm_hal.c 是 HAL 模板：
 *          每个函数体只包含一个 User Code 区段，区段名 == 函数名，
 *          用户把实现写在区段内；区段外内容由驱动维护者管理，
 *          升级驱动时（配合同步脚本）不会覆盖用户代码。对应关系：
 *
 *              icm_hal_init()         <-> 区段 [icm_hal_init]
 *              icm_hal_spi_xfer()     <-> 区段 [icm_hal_spi_xfer]
 *              icm_hal_delay_us()     <-> 区段 [icm_hal_delay_us]
 *              icm_hal_delay_ms()     <-> 区段 [icm_hal_delay_ms]
 *              icm_hal_get_tick_ms()  <-> 区段 [icm_hal_get_tick_ms]
 *              文件级 include         <-> 区段 [includes]
 *
 * @note    【SPI 约定（与数据手册一致）】
 *          - 4 线 SPI，Mode 0(CPOL=0,CPHA=0) 或 Mode 3(CPOL=1,CPHA=1)；
 *          - 8 bit 帧、MSB first；
 *          - 单次「事务」内 CS 由 icm_hal_spi_xfer 内部管理（开始拉低、结束拉高），
 *            业务层不需要关心 CS，减少出错面。
 *          - tx/rx 缓冲可指向同一地址（全双工回环）也可不同；len 为字节数。
 *
 * @note    【依赖】icm_cfg.h（取错误码 icm_err_t / ICM_OK / ICM_ERR_*）。
 *          错误码放 cfg.h 是为避免「hal → ctrl → hal」循环依赖。
 */

#ifndef ICM_HAL_H
#define ICM_HAL_H

#include "icm_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * HAL 接口（在 icm_hal.c 的 User Code 区段内实现）
 * ============================================================================= */

/**
 * @brief  初始化底层 SPI 外设 + 将 CS 置为无效(高)
 * @return ICM_OK 成功；其他见 icm_err_t
 * @note   例如 RA 平台调用 R_SPI_Open()；RX 平台初始化 SCI/SPI 模块。
 *         若外设已由别处打开，返回 ICM_OK 即可（视为已就绪）。
 */
icm_err_t icm_hal_init(void);

/**
 * @brief  单次全双工 SPI 传输（自动管理 CS）
 * @param  tx : 发送缓冲（长度 len），不可为 NULL
 * @param  rx : 接收缓冲（长度 len），可为 NULL 仅发不收
 * @param  len: 字节数（0 直接返回 ICM_OK）
 * @return ICM_OK 成功；ICM_ERR_HAL 底层失败；ICM_ERR_PARAM 参数非法；
 *         ICM_ERR_TIMEOUT 等待传输完成超时
 * @note   实现要求：进入时拉低 CS，传输 len 字节，退出前拉高 CS。
 *         若平台使用中断/DMA，请在内部阻塞直到完成（或返回超时错误）。
 */
icm_err_t icm_hal_spi_xfer(uint8_t *tx, uint8_t *rx, uint16_t len);

/**
 * @brief  微秒级延时
 * @note   用于 PWR 切换后的 200us 等待等，建议使用硬件定时器/RTOS 延时，
 *         不可用空转时请保证精度在 ±50% 以内（过短会违反手册时序）。
 */
void icm_hal_delay_us(uint32_t us);

/**
 * @brief  毫秒级延时
 */
void icm_hal_delay_ms(uint32_t ms);

/**
 * @brief  获取系统滴答（毫秒），用于积分 dt 计算
 * @return 自启动以来的毫秒计数（32 位回绕可接受，积分层做了差值处理）
 * @note   建议直接返回 SysTick / RTOS 时基；若平台无滴答，可返回 0（此时
 *         icm_est_update_auto 会报 ICM_ERR_HAL，需由调用方改用
 *         icm_est_update(e, s, dt) 显式传入 dt）。
 */
uint32_t icm_hal_get_tick_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM_HAL_H */
