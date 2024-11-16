
#ifndef __IO_IIC_H__
#define __IO_IIC_H__

#include <stdint.h>
#include <stdio.h>

#include "io_iic_config.h"

#define IIC_RESET 0
#define IIC_SET 1

// IIC所有操作函数
void io_iic_init(void);                      //初始化IIC的IO口
uint8_t io_iic_start(void);                  //发送IIC开始信号
void io_iic_stop(void);                      //发送IIC停止信号
void io_iic_send_byte(uint8_t txd);          // IIC发送一个字节
uint8_t io_iic_read_byte(unsigned char ack); // IIC读取一个字节
uint8_t io_iic_wait_ack(void);               // IIC等待ACK信号
void io_iic_send_ack(void);                  // IIC发送ACK信号
void io_iic_send_nack(void);                 // IIC不发送ACK信号

uint8_t io_iic_read_bytes(uint8_t slave_addr,
                        uint8_t reg,
                        uint8_t *data,
                        uint8_t length); // 读取N字节长度
uint8_t io_iic_write_bytes(uint8_t slave_addr,
                        uint8_t reg,
                        uint8_t *data,
                        uint8_t length); //写入N字节长度

uint8_t io_iic_read_bytes_16bit_reg(uint8_t slave_addr,
                        uint16_t reg,
                        uint8_t *data,
                        uint8_t length); // 读取N字节长度
uint8_t io_iic_write_bytes_16bit_reg(uint8_t slave_addr,
                        uint16_t reg,
                        uint8_t *data,
                        uint8_t length); //写入N字节长度

void delay_us(uint32_t us); // config里面的内联函数

#endif
