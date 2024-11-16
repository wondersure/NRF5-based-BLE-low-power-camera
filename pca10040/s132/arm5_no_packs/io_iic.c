
#include "io_iic.h"
#include <stdint.h>
#include <string.h>

//初始化IIC
void io_iic_init(void)
{
	SCL_OUT();
	SDA_OUT();
	IIC_SCL(1);
	IIC_SDA(1);
}

//产生IIC起始信号
uint8_t io_iic_start(void)
{
	// sda线输出
	SDA_OUT();
	IIC_SDA(1);
	IIC_SCL(1);
	if (!SDA_READ)
	{
		log_d("%s(%d):ERROR!!!", __func__, __LINE__);
		return 0;
	}
	delay_us(5);
	IIC_SDA(0);
	if (SDA_READ)
	{
		log_d("%s(%d):ERROR!!!", __func__, __LINE__);
		return 0;
	}
	delay_us(5);
	//钳住IIC总线，准备发送或接收数据
	IIC_SCL(0);
	return 1;
}

//产生IIC停止信号
void io_iic_stop(void)
{
	// sda线输出
	SDA_OUT();
	IIC_SCL(0);
	IIC_SDA(0);
	delay_us(5);
	IIC_SCL(1);
	//发送IIC总线结束信号
	IIC_SDA(1);
	delay_us(5);
}

//等待应答信号到来返回值：1，接收应答失败  0，接收应答成功
uint8_t io_iic_wait_ack(void)
{
	uint32_t ucErrTime = 0;
	// SDA设置为输入
	SDA_IN();
	IIC_SDA(1);
	delay_us(1);
	IIC_SCL(1);
	delay_us(1);
	while (READ_SDA)
	{
		ucErrTime++;
		if (ucErrTime > 0xFFFF0)
		{
			io_iic_stop();
			log_d("%s(%d):ERROR!!!", __func__, __LINE__);
			return 1;
		}
		delay_us(1);
	}
	//时钟输出0
	IIC_SCL(0);
	return 0;
}

//产生ACK应答
void io_iic_send_ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	delay_us(5);
	IIC_SCL(1);
	delay_us(5);
	IIC_SCL(0);
}

//不产生ACK应答
void io_iic_send_nack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	delay_us(5);
	IIC_SCL(1);
	delay_us(5);
	IIC_SCL(0);
}

// IIC发送一个字节,返回从机有无应答 1，有应答  0，无应答
void io_iic_send_byte(uint8_t txd)
{
	uint8_t t;
	SDA_OUT();
	IIC_SCL(0); //拉低时钟开始数据传输
	for (t = 0; t < 8; t++)
	{
		if ((txd & 0x80) >> 7)
			IIC_SDA(1);
		else
			IIC_SDA(0);
		txd <<= 1;
		delay_us(50);
		IIC_SCL(1);
		delay_us(50);
		IIC_SCL(0);
		delay_us(50);
	}
}

//读1个字节，ack=1时，发送ACK，ack=0，发送NACK
uint8_t io_iic_read_byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	// SDA设置为输入
	SDA_IN();
	for (i = 0; i < 8; i++)
	{
		IIC_SCL(0);
		delay_us(50);
		IIC_SCL(1);
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delay_us(50);
	}
	//判断发送ACK
	if (!ack)
		io_iic_send_nack();
	else
		io_iic_send_ack();
	return receive;
}

uint8_t io_iic_write_bytes(uint8_t slave_addr,
							uint8_t reg,
							uint8_t *data,
							uint8_t length)
{
	//起始信号
	io_iic_start();
	//发送设备地址+写信号
	io_iic_send_byte(slave_addr);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//内部寄存器地址
	io_iic_send_byte(reg);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//内部寄存器数据
	while (length)
	{
		io_iic_send_byte(*data++);
		if (io_iic_wait_ack())
		{
			io_iic_stop();
			return IIC_RESET;
		}
		length--;
	}
	//发送停止信号
	io_iic_stop();
	return IIC_SET;
}

uint8_t io_iic_read_bytes(uint8_t slave_addr,
						  uint8_t reg,
						  uint8_t *data,
						  uint8_t length)
{
	//起始信号
	if (io_iic_start() == 0)
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//发送设备地址+写信号
	io_iic_send_byte(slave_addr);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//发送存储单元地址
	io_iic_send_byte(reg);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//起始信号
	if (io_iic_start() == 0)
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//发送设备地址+读信号
	io_iic_send_byte(slave_addr + 1);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//读出寄存器数据
	while (length - 1)
	{
		*data++ = io_iic_read_byte(1);
		length--;
	}
	*data = io_iic_read_byte(0);
	//停止信号
	io_iic_stop();
	return IIC_SET;
}

uint8_t io_iic_write_bytes_16bit_reg(uint8_t slave_addr,
							uint16_t reg,
							uint8_t *data,
							uint8_t length)
{
	//起始信号
	io_iic_start();
	//发送设备地址+写信号
	io_iic_send_byte(slave_addr);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//内部寄存器地址
	io_iic_send_byte((uint8_t)(reg>>8)&0xFF);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}

	io_iic_send_byte((uint8_t)(reg)&0xFF);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//内部寄存器数据
	while (length)
	{
		io_iic_send_byte(*data++);
		if (io_iic_wait_ack())
		{
			io_iic_stop();
			return IIC_RESET;
		}
		length--;
	}
	//发送停止信号
	io_iic_stop();
	return IIC_SET;
}

uint8_t io_iic_read_bytes_16bit_reg(uint8_t slave_addr,
						  uint16_t reg,
						  uint8_t *data,
						  uint8_t length)
{
	//起始信号
	if (io_iic_start() == 0)
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//发送设备地址+写信号
	io_iic_send_byte(slave_addr);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//发送存储单元地址
	io_iic_send_byte((uint8_t)(reg>>8)&0xFF);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}

	io_iic_send_byte((uint8_t)(reg)&0xFF);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//起始信号
	if (io_iic_start() == 0)
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//发送设备地址+读信号
	io_iic_send_byte(slave_addr + 1);
	if (io_iic_wait_ack())
	{
		io_iic_stop();
		return IIC_RESET;
	}
	//读出寄存器数据
	while (length - 1)
	{
		*data++ = io_iic_read_byte(1);
		length--;
	}
	*data = io_iic_read_byte(0);
	//停止信号
	io_iic_stop();
	return IIC_SET;
}
