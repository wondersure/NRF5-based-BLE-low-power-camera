#ifndef BOARD_CAMERA_H__
#define BOARD_CAMERA_H__

#include <stdint.h>
#include <nrf_delay.h>

#define HM01B0_X_NUM		(80)
#define HM01B0_Y_NUM		(60)		// Just Try 100 WHole Picture is 324*324

#define TWI_ADDRESSES       127
#define HM01B0_SENSOR_ID    0x24

#define MY_SCL_PIN     			10
#define MY_SDA_PIN     			9

#define MY_RX 				26
#define MY_TX 				27

#define D0     			15
#define D1     			24
#define D2     			18
#define D3     			23
#define D4     			17
#define D5     			11
#define D6     			12
#define D7    			16

#define MCLK 				19

#define VSYNC    22
#define HSYNC    14	

#define PCLK    13
#define TRIG    20
#define SPI_SS_PIN	31

typedef enum {
    HM01B0_ERR_OK               = 0x00,
    HM01B0_ERR,
    HM01B0_ERR_I2C,
    HM01B0_ERR_MODE,
    HM01B0_ERR_AE_NOT_CONVERGED,
    HM01B0_ERR_MCLK,
    HM01B0_ERR_INIT,
    HM01B0_ERR_DEINIT,
    HM01B0_ERR_PARAMS,
    HM01B0_ERR_UNIMPLEMENTED,

    HM01B0_NUM_ERR
} hm01b0_status_e;

typedef struct
{
    uint8_t                 ui8AETargetMean;
    uint8_t                 ui8AEMinMean;
    uint8_t                 ui8ConvergeInTh;
    uint8_t                 ui8ConvergeOutTh;
    uint8_t                 ui8AEMean;
} hm01b0_ae_cfg_t;


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
	
void hm01b0_init(void);

uint32_t hm01b0_write_reg(uint16_t addr16, uint8_t value);

void hm01b0_read_oneframe( uint8_t *pui8Buffer, uint32_t ui32BufferLen);
void hm01b0_spiread_oneframe_decimated(uint8_t *pui8Buffer, uint32_t ui32BufferLen);

void hm01b0_out_one_frame(void);
void hm01b0_streaming(void);
void hm01b0_hdtrig_streaming(void);
void hm01b0_set_model(uint8_t mode);
void hm01b0_spiread_oneframe( uint8_t *pui8Buffer, uint32_t ui32BufferLen );
void hm01b0_wait_one_frame(void);
void test_temp(void);
void hm01b0_test_figure(void);
void autoexposure( uint8_t expnum );
void hm01b0_cmd_update(void);
hm01b0_status_e hm01b0_get_mode(uint8_t *pui8Mode);
hm01b0_status_e hm01b0_set_mode(uint8_t ui8Mode, uint8_t ui8FrameCnt);
hm01b0_status_e hm01b0_get_ae(hm01b0_ae_cfg_t *psAECfg);
void hm01b0_init_fixed_rom_qvga_fixed(void);
void hm01b0_init_fixed_rom_qvga_fixed_acc(void);
void hm01b0_init_fixed_rom_qvga_fixed_maxfps(void);
void hm01b0_init_brighter(void);
void hm01b0_init_datasheet_default(void);
hm01b0_status_e hm01b0_cal_ae(uint8_t ui8CalFrames, uint8_t *pui8Buffer, uint32_t ui32BufferLen, hm01b0_ae_cfg_t* pAECfg);

#endif

#define HM01B0_REG_AE_TARGET_MEAN                       (0x2101)
#define HM01B0_REG_AE_MIN_MEAN                          (0x2102)
#define HM01B0_REG_CONVERGE_IN_TH                       (0x2103)
#define HM01B0_REG_CONVERGE_OUT_TH                      (0x2104)
