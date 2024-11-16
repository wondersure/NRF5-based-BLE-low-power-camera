
#ifndef __IO_IIC_CONFIG_H__
#define __IO_IIC_CONFIG_H__
/***********************************************************
 * @brief 用户设置选项
 **/
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define IIC_SDA_PIN            24
#define IIC_SCL_PIN            25    

// IO方向设置
#define SCL_OUT() nrf_gpio_cfg_output(IIC_SCL_PIN)
#define SDA_IN() nrf_gpio_cfg_input(IIC_SDA_PIN, NRF_GPIO_PIN_PULLUP)
#define SDA_OUT() nrf_gpio_cfg_output(IIC_SDA_PIN)
// IO操作函数
#define IIC_SCL(X) nrf_gpio_pin_write(IIC_SCL_PIN, X) // SCL
#define IIC_SDA(X) nrf_gpio_pin_write(IIC_SDA_PIN, X) // SDA
#define READ_SDA nrf_gpio_pin_read(IIC_SDA_PIN)       //输入SDA
// 配置为输出方向的IO读取
#define SCL_READ nrf_gpio_pin_out_read(IIC_SCL_PIN)
#define SDA_READ nrf_gpio_pin_out_read(IIC_SDA_PIN)
// 延时函数
inline void delay_us(uint32_t us)
{
    nrf_delay_us(us * 5);
}
// 调试定义 e.g #define log_d printf
#define log_d NRF_LOG_DEBUG // 调试输出
#endif
