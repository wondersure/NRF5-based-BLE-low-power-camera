#ifndef BOARD_CAMERA_H__
#define BOARD_CAMERA_H__

#include <stdint.h>
#include <nrf_delay.h>

#define HM01B0_X_NUM		(240)
#define HM01B0_Y_NUM		(240)		// Just Try 100 WHole Picture is 324*324

#define TWI_ADDRESSES       127
#define HM01B0_SENSOR_ID    0x24

#define MY_SCL_PIN     			40
#define MY_SDA_PIN     			11

#define MY_RX 				25
#define MY_TX 				15

#define D0     			29
#define D1     			25
#define D2     			30
#define D3     			26
#define D4     			31
#define D5     			8
#define D6     			7
#define D7    			2

#define MCLK 				42

#define VSYNC    3
#define HSYNC    31	

#define PCLK    30
#define TRIG    28

#define SPI_SS_PIN	31
#define TEST_PATTERN 0x00

/**#############################Registers#############################**/
// Sensor ID
#define REG_MODEL_ID_H 0x0000
#define REG_MODEL_ID_L 0x0001
#define REG_SILICON_REV 0x0002
#define REG_FRAME_COUNT 0x0005
#define REG_PIXEL_ORDER 0x0006

// Sensor mode control
#define REG_MODE_SELECT 0x0100
#define REG_IMAGE_ORIENTATION 0x0101
#define REG_SW_RESET 0x0103
#define REG_GRP_PARAM_HOLD 0x0104

// Sensor exposure gain control

// Frame timing control
#define REG_FRAME_LENGTH_LINES_H 0x0340
#define REG_FRAME_LENGTH_LINES_L 0x0341
#define REG_FRAME_LENGTH_PCK_H 0x0342
#define REG_FRAME_LENGTH_PCK_L 0x0343

// Bining mode control

// Test pattern control
#define REG_TEST_PATTERN_MODE 0x0601

// Black level control
#define REG_BLC_CFG 0x1000
#define REG_BLC_TGT 0x1003
#define REG_BLI_EN 0x1006
#define REG_BLC2_TGT 0x1007

// Sensor resevred

// VSYNC, HSYNC and pixel shift register
#define REG_VSYNC_HSYNC_PIXEL_SHIFT_EN 0x1012

// Binning mode control
#define REG_BIN_RDOUT_X 0x0383
#define REG_BIN_RDOUT_Y 0x0387
#define REG_BIN_MODE 0x0390

#define HM01B0_REG_MODE_SELECT (0x0100)

#define HM01B0_REG_PMU_PROGRAMMABLE_FRAMECNT (0x3020)

// #define HM01B0_REG_MODE_SELECT (0x0100)
#define HM01B0_REG_MODE_SELECT_STANDBY (0x00)
#define HM01B0_REG_MODE_SELECT_STREAMING (0x01)
#define HM01B0_REG_MODE_SELECT_STREAMING_NFRAMES (0x03)
#define HM01B0_REG_MODE_SELECT_STREAMING_HW_TRIGGER (0x05)

// #define HM01B0_REG_IMAGE_ORIENTATION                    (0x0101)
#define HM01B0_REG_IMAGE_ORIENTATION_DEFAULT (0x00)
#define HM01B0_REG_IMAGE_ORIENTATION_HMIRROR (0x01)
#define HM01B0_REG_IMAGE_ORIENTATION_VMIRROR (0x02)
#define HM01B0_REG_IMAGE_ORIENTATION_HVMIRROR (HM01B0_REG_IMAGE_ORIENTATION_HMIRROR | HM01B0_REG_IMAGE_ORIENTATION_HVMIRROR)

// Statistic control and read only

// Automatic exposure gain control

// Motion detection control

// Sensor timing control
#define REG_QVGA_WIN_EN 0x3010
#define REG_SIX_BIT_MODE_EN 0x3011
#define REG_PMU_PROGRAMMABLE_FRAMECNT 0x3020
#define REG_ADVANCE_VSYNC 0x3022
#define REG_ADVANCE_HSYNC 0x3023
#define REG_EARLY_GAIN 0x3035

// IO and clock control
#define REG_BIT_CONTROL 0x3059
#define REG_OSC_CLK_DIV 0x3060
#define REG_ANA_REGISTER_11 0x3061
#define REG_IO_DRIVE_STR 0x3062
#define REG_IO_DRIVE_STR2 0x3063
#define REG_ANA_REGISTER_14 0x3064
#define REG_OUTPUT_PIN_STATUS_CONTROL 0x3065
#define REG_ANA_REGISTER_17 0x3067
#define REG_PCLK_POLARITY 0x3068

// I2C slave registers

/**#############################Modes#############################**/
#define MODE_STAND_BY 0x00
#define MODE_STREAMING 0x01
#define MODE_STREAMING2 0x02
#define MODE_STREAMING3 0x03

#define ORIENTATION_HORINONTAL 0x00
#define ORIENTATION_VERTICAL 0x01

#define TEST_PATTERN_OFF 0x00
#define TEST_PATTERN_COLOR_BAR 0x01
#define TEST_PATTERN_WALKING_1 0x11



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
