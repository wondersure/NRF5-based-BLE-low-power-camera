#ifndef BOARD_CAMERA_H__
#define BOARD_CAMERA_H__

#include <stdint.h>
#include <nrf_delay.h>

#define HM01B0_X_NUM		(150)
#define HM01B0_Y_NUM		(150)		// Just Try 100 WHole Picture is 324*324

#define TWI_ADDRESSES       127
#define HM01B0_SENSOR_ID    0x24

#define MY_SCL_PIN     			5
#define MY_SDA_PIN     			7

#define MY_RX 				22
#define MY_TX 				6

#define D0     			11
#define D1     			19
#define D2     			14
#define D3     			18
#define D4     			13
#define D5     			2
#define D6     			3
#define D7    			12

#define MCLK 				15

#define VSYNC    17
#define HSYNC    10	

#define PCLK    9
#define TRIG    16
#define SPI_SS_PIN	31

enum i2c_mode{
	I2C_MODE_16_8 = 0,
	I2C_MODE_8_8 = 1,
};

struct senosr_reg{
	uint16_t reg;
	uint8_t  val;
};

struct boardcam_config {
  uint8_t sensor_address;
	enum i2c_mode sccb_mode;
	uint8_t *image_buf;
	uint16_t image_buf_size;
};

#if 0
void board_sleep_us(uint8_t us)
{
	nrf_delay_us(us);
}
#endif

void  hm01b0_8_bit_model(void);
void  hm01b0_1_bit_model(void);
	
void unit_twi_test(void);

uint32_t hm01b0_write_reg(uint16_t addr16, uint8_t value);

void hm01b0_read_oneframe( uint8_t *pui8Buffer, uint32_t ui32BufferLen);
void hm01b0_out_one_frame(void);
void hm01b0_streaming(void);
void hm01b0_hdtrig_streaming(void);
void hm01b0_spiread_oneframe( uint8_t *pui8Buffer, uint32_t ui32BufferLen );
void hm01b0_wait_one_frame(void);
void test_temp(void);
void hm01b0_test_figure(void);
void autoexposure( uint8_t expnum );

#endif

#define HM01B0_REG_AE_TARGET_MEAN                       (0x2101)
#define HM01B0_REG_AE_MIN_MEAN                          (0x2102)
#define HM01B0_REG_CONVERGE_IN_TH                       (0x2103)
#define HM01B0_REG_CONVERGE_OUT_TH                      (0x2104)
