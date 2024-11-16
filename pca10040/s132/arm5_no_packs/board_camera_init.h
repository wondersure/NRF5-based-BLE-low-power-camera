#ifndef BOARD_CAMERA_INIT_H
#define BOARD_CAMERA_INIT_H

#include <stdint.h>
#include "board_camera.h"

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

struct senosr_reg hm01b0_324x244[] = {
    {0x0103, 0x00},
    {0x0100, 0x00},

    {0x1003, 0x08},
    {0x1007, 0x08},
    {0x3044, 0x0A},
    {0x3045, 0x00},
    {0x3047, 0x0A},
    {0x3050, 0xC0},
    {0x3051, 0x42},
    {0x3052, 0x50},
    {0x3053, 0x00},
    {0x3054, 0x03},
    {0x3055, 0xF7},
    {0x3056, 0xF8},
    {0x3057, 0x29},
    {0x3058, 0x1F},
    {0x3059, 0x1E},
    {0x3064, 0x00},
    {0x3065, 0x04},
    // IO 和时钟控制
    {0x1000, 0x43},
    {0x1001, 0x40},
    {0x1002, 0x32},
    {0x0350, 0x7F},
    {0x1006, 0x01},
    {0x1008, 0x00},
    {0x1009, 0xA0},
    {0x100A, 0x60},
    {0x100B, 0x90},
    {0x100C, 0x40},
    {0x3022, 0x01}, // 将 VSYNC 字段从 0 提前到20
    {0x1012, 0x01},



    // 统计控制和只读
    {0x2000, 0x07},
    {0x2003, 0x00},
    {0x2004, 0x1C},
    {0x2007, 0x00},
    {0x2008, 0x58},
    {0x200B, 0x00},
    {0x200C, 0x7A},
    {0x200F, 0x00},
    {0x2010, 0xB8},
    {0x2013, 0x00},
    {0x2014, 0x58},
    {0x2017, 0x00},
    {0x2018, 0x9B},
    // 自动曝光增益
    // // AE
    // W 24 2100 01 2 1 ; [0]: AE control enable
    {0x2100, 0x01},
    // W 24 2104 07 2 1 ; converge out th
    {0x2104, 0x07},
    // W 24 2105 0C 2 1 ; max INTG Hb
    {0x2105, 0x0C},
    // W 24 2106 78 2 1 ; max INTG Lb
    {0x2106, 0x78},
    // W 24 2108 03 2 1 ; max AGain in full
    {0x2108, 0x03},
    // W 24 2109 03 2 1 ; max AGain in bin2
    {0x2109, 0x03},
    // W 24 210B 80 2 1 ; max DGain
    {0x210B, 0x80},
    // W 24 210F 00 2 1 ; FS 60Hz Hb
    {0x210F, 0x00},
    // W 24 2110 85 2 1 ; FS 60Hz Lb
    {0x2110, 0x85},
    // W 24 2111 00 2 1 ; Fs 50Hz Hb
    {0x2111, 0x00},
    // W 24 2112 A0 2 1 ; FS 50Hz Lb
    {0x2112, 0xA0},
		
    {0x2101, 0x1F},
    {0x2102, 0x05},
    {0x2103, 0x03},
    {0x2104, 0x05},
		
    //		{0x2100, 0x01},
    //    {0x2101, 0x5F},
    //    {0x2102, 0x0A},
    //    {0x2103, 0x03},
    //    {0x2104, 0x05},
    //    {0x2105, 0x01},
    //    {0x2106, 0x78},
    //    {0x2107, 0x02},
    //    {0x2108, 0x03},
    //    {0x2109, 0x03},
    //    {0x210A, 0x00},
    //    {0x210B, 0x80},
    //    {0x210C, 0x40},
    //    {0x210D, 0x20},
    //    {0x210E, 0x03},
    //    {0x210F, 0x00},
    //    {0x2110, 0x42},
    //    {0x2111, 0x00},
    //    {0x2112, 0x50},
    // Motion detection control
    {0x2150, 0x03},
    {0x0340, 0x0C},
    {0x0341, 0x7A},
    {0x0342, 0x01},
    {0x0343, 0x77},

    {0x3010, 0x01}, // bit[0] 1 enable QVGA
    {0x0383, 0x03},
    {0x0387, 0x03},
    {0x0390, 0x03},

    {0x3011, 0x00},
    {0x3059, 0x22},
    {0x3060, 0x30}, // CHANGE TO 0X10 NOT OK

    {0x0101, 0x01},
    {0x0104, 0x01},
    //{0x0100, 0x00},
    //{0x0100, 0x01},
    //{0x3059, (1 << 5)},
    //{0x0100, 0x05}, // 硬件触发模式
		
		
		// *** 黑白平衡，似乎也没用
		//{0x1000, 0x01},
    //{0x1003, 0x05},
		//{0x1006, 0x01},
    //{0x1007, 0x05},
		
		
		// *** 图片获取时间控制
		{0x0340, 0x01},
    {0x0341, 0x58},
		
		// *** 曝光时间控制
    {0x0202, 0x00},
    {0x0203, 0x05},
		
		// *** 205理论上是模拟增益 30代表8*
    {0x0205, 0x0C},
		
		// *** 这两个是数字增益
    {0x020E, 0x01},
    {0x020F, 0x10},
		
		
		{0x0101, 0x01},
    {0x0104, 0x01},
		
    {0xFFFF, 0xFF},

};
#endif
