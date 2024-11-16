#include <stdint.h>
#include <nrf_gpio.h>
#include <nrf_delay.h>
#include <stdio.h>
#include <string.h>
#include <nrf_drv_spi.h>
#include <nrf_drv_twi.h>
#include <nrf_drv_gpiote.h>
#include "nrf_drv_spis.h"
#include "nrf_log.h"
#include "board_camera.h"
#include "board_camera_init.h"
#include "nrf_drv_pwm.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"

static volatile bool V_flag = false;
static nrf_drv_timer_t timer = NRF_DRV_TIMER_INSTANCE(1);
static void timer_dummy_handler(nrf_timer_event_t event_type, void * p_context){
	NRF_LOG_INFO("timer_dummy_handler \r\n");
}

int h_num = 0;
void hm01b0_inputpin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	V_flag = (!V_flag);
}



// ****** HM01B0 Spi init
#define SPIS_INSTANCE 1 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
static volatile bool spis_xfer_done; 
void spis_event_handler(nrf_drv_spis_event_t event)
{
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
        spis_xfer_done = true;
    }
}

void hm01b0_spi_init(void)
{
    //printf("spi0_init_start. \r\n");
	
		nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin               = HSYNC;
    spis_config.miso_pin              = NRF_DRV_SPIS_PIN_NOT_USED;
    spis_config.mosi_pin              = D0;
    spis_config.sck_pin               = PCLK;
		spis_config.mode = NRF_SPIS_MODE_0;
		APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handler));
	
    //printf("spi0_init_end. \r\n");
	
}


// *** HM01B0 Spi read one picture
#define txnum   5
#define loc   0
static uint8_t       m_tx_buf[txnum] = {0xff};         /**< TX buffer. */
static uint8_t       m_rx_buf1[HM01B0_X_NUM*2];         /**< TX buffer. */
static uint8_t       m_rx_buf2[HM01B0_X_NUM*2];         /**< TX buffer. */
void hm01b0_spiread_oneframe( uint8_t *pui8Buffer, uint32_t ui32BufferLen )
{	

	while(!V_flag);
	
#if 0
	int j = 0;
	for (int i = 0; i < (HM01B0_Y_NUM * 3 ); i++)
	{	
		if ( (i%3) == 0) 
		{
			// *** number is odd 0 2 4 6 8
			nrf_drv_spis_buffers_set(&spis, m_tx_buf, txnum, ( pui8Buffer + j * HM01B0_X_NUM ) , HM01B0_X_NUM) ;
			while (!spis_xfer_done);
			spis_xfer_done = false;
			j++;
		} 
		else 
		{
			// *** number is even 1 3 5 7
			nrf_drv_spis_buffers_set(&spis, m_tx_buf, txnum, m_rx_buf , HM01B0_X_NUM) ;
			while (!spis_xfer_done);
			spis_xfer_done = false;
		}
	}
	
#endif

#if 1
	for (int i = 0; i < HM01B0_Y_NUM + loc; i++)
	{	

			nrf_drv_spis_buffers_set(&spis, m_tx_buf, txnum, ( pui8Buffer + (i-loc) * HM01B0_X_NUM ) , HM01B0_X_NUM);
			while (!spis_xfer_done);
			spis_xfer_done = false;
	}
#endif
	
	nrf_drv_spis_buffers_set(&spis, m_tx_buf, txnum, m_rx_buf1 , HM01B0_X_NUM) ;
	while(V_flag);
	
}


void hm01b0_spiread_oneframe_decimated(uint8_t *pui8Buffer, uint32_t ui32BufferLen)
{
    while(!V_flag);

    int j = 0; // Index for the decimated buffer
    for (int i = 0; i < HM01B0_Y_NUM * 2; i += 2) // Increment by 2 for every row
    {
			
				// Receive the 1st row of the 2x2 block (which we'll discard)
        nrf_drv_spis_buffers_set(&spis, m_tx_buf, txnum, m_rx_buf1, HM01B0_X_NUM*2);
        while (!spis_xfer_done);
        spis_xfer_done = false;
			
        // Receive the first row of the 2x2 block
        nrf_drv_spis_buffers_set(&spis, m_tx_buf, txnum, m_rx_buf2, HM01B0_X_NUM*2);
			
        // While DMA is receiving the next row, we'll process the current row
        for (int k = 0; k < HM01B0_X_NUM * 2; k += 2) 	// Increment by 2 for every pixel
        {
            pui8Buffer[j++] = m_rx_buf1[k]; 						// Keep the first pixel of every 2x2 block
        }
        while (!spis_xfer_done);
        spis_xfer_done = false;


    }

    while(V_flag);
}

#define TWI_INSTANCE  0
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);
void hm01b0_twi_init(void)
{
    //printf("twi0_start. \r\n");
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_hm01b0_config = {
       .scl                = MY_SCL_PIN,
       .sda                = MY_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_hm01b0_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
    //printf("twi0_end. \r\n");
}

//

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
#define USED_PWM(idx) (1UL << idx)
static uint8_t m_used = 0;
void mclk_init(void)
{
    printf("MCLK Init");

    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            MCLK | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_16MHz,//NRF_PWM_CLK_125kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 4, // = 0x8000
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
    m_used |= USED_PWM(0);

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static uint16_t /*const*/ seq_values[] =
    {
        0x02,
//        0x2000,
//        0x4000,
//        0x6000,
//        0x7FFF,
//        0x8000
    };
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = seq_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats         = 0,
        .end_delay       = 0
    };

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

// ****** HM01B0 pib init
void hm01b0_pin_init()
{		
    nrf_gpio_cfg_output(TRIG);
		//nrf_gpio_cfg_output(D0);
	
	
		//APP_ERROR_CHECK(nrfx_gpiote_init());

    APP_ERROR_CHECK(nrf_drv_gpiote_init());
	
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    APP_ERROR_CHECK( nrf_drv_gpiote_in_init(VSYNC, &in_config, hm01b0_inputpin_handler) );
		nrfx_gpiote_in_event_enable(VSYNC, true);
	
    //nrfx_gpiote_in_config_t in_config2 = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
		//in_config2.pull = NRF_GPIO_PIN_PULLDOWN;
    //APP_ERROR_CHECK( nrf_drv_gpiote_in_init(HSYNC, &in_config2, hm01b0_inputpin_handler) );
		//nrfx_gpiote_in_event_enable(HSYNC, true);
	
	  ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&timer, &timer_cfg, timer_dummy_handler);
    APP_ERROR_CHECK(err_code);

	  uint32_t compare_evt_addr;
    uint32_t gpiote_task_addr;
    nrf_ppi_channel_t ppi_channel;

    nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    err_code = nrf_drv_gpiote_out_init(MCLK, &config);
    APP_ERROR_CHECK(err_code);


    nrf_drv_timer_extended_compare(&timer, (nrf_timer_cc_channel_t)0, 2UL, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel);
    APP_ERROR_CHECK(err_code);

    compare_evt_addr = nrf_drv_timer_event_address_get(&timer, NRF_TIMER_EVENT_COMPARE0);
    gpiote_task_addr = nrf_drv_gpiote_out_task_addr_get(MCLK);

    err_code = nrf_drv_ppi_channel_assign(ppi_channel, compare_evt_addr, gpiote_task_addr);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(ppi_channel);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_task_enable(MCLK);
		
	
	  nrf_drv_timer_enable(&timer);	


}







// *** HM01B0 Read reg value
uint32_t hm01b0_read_reg(uint16_t addr16, uint8_t *value, uint8_t number_rd)
{
    ret_code_t err_code;

    uint8_t reg[2] = {(uint8_t)(addr16 >> 8) & 0xFF, (uint8_t)(addr16)&0xFF};

    err_code = nrf_drv_twi_tx(&m_twi, HM01B0_SENSOR_ID, reg, sizeof(reg), false);

    if (NRF_SUCCESS != err_code)
    {
        return false;
    }

    err_code = nrf_drv_twi_rx(&m_twi, HM01B0_SENSOR_ID, value, number_rd);

    if (NRF_SUCCESS != err_code)
    {
        return false;
    }

    return true;
}


// *** HM01B0 write reg value
uint32_t hm01b0_write_reg(uint16_t addr16, uint8_t value)
{
    ret_code_t err_code;

    uint8_t reg[3] = {(uint8_t)(addr16 >> 8) & 0xFF, (uint8_t)(addr16)&0xFF, value};
    err_code = nrf_drv_twi_tx(&m_twi, HM01B0_SENSOR_ID, reg, sizeof(reg), false);
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }

    return true;
}



// *** HM01B0 Pin and Regs Init
void hm01b0_init_regs(struct senosr_reg *regs_list) 
	{
    //NRF_LOG_INFO("hm01b0_init_regs start \r\n");
    while (1) 
		{
        uint16_t reg   = regs_list->reg;
        uint8_t  value = regs_list->val;

        if (reg == 0xFFFF && value == 0xFF) {
          break;
        }

    if(false == hm01b0_write_reg(reg, value)){
			NRF_LOG_INFO("hm01b0_write_reg fail address:%d value:%d",reg,value);
		}

    regs_list++;
  }
  //printf("hm01b0_init_regs end \r\n");
}





// *** read byte
uint8_t read_byte()
{
	uint32_t P_byte;
	uint8_t d0 = nrf_gpio_pin_read(D0);
	uint8_t d1 = nrf_gpio_pin_read(D1);
	uint8_t d2 = nrf_gpio_pin_read(D2);
	uint8_t d3 = nrf_gpio_pin_read(D3);
	uint8_t d4 = nrf_gpio_pin_read(D4);
	uint8_t d5 = nrf_gpio_pin_read(D5);
	uint8_t d6 = nrf_gpio_pin_read(D6);
	uint8_t d7 = nrf_gpio_pin_read(D7);
	
	P_byte = (d0<<0)|(d1<<1)|(d2<<2)|(d3<<3)|(d4<<4)|(d5<<5)|(d6<<6)|(d7<<7);
	return P_byte;
}




// *** HM01B0 read one frame
void hm01b0_read_oneframe( uint8_t *pui8Buffer, uint32_t ui32BufferLen)
{

    uint32_t    ui32Idx         = 0x00;
    uint32_t    ui32HsyncCnt    = 0x00;
	
    while((ui32HsyncCnt < HM01B0_Y_NUM))
    {
        while (0x00 == nrf_gpio_pin_read(HSYNC));
			
        while( nrf_gpio_pin_read(HSYNC) )
        {
            while(0x00 == nrf_gpio_pin_read(PCLK));

            *(pui8Buffer + ui32Idx++) = read_byte();

            if (ui32Idx == ui32BufferLen) 
						{
                goto end;
            }
            while(nrf_gpio_pin_read(PCLK));
        }
        ui32HsyncCnt++;
    }
end:
		printf("A picture is Done. \r\n");
}


// *** HM01B0 output one frame
void hm01b0_out_one_frame()
{
		hm01b0_write_reg( (0x3020),	(0x01) );		// output one picture
		hm01b0_write_reg( (0x0100),	(0x03) );
}


// *** HM01B0 wait one frame
void hm01b0_wait_one_frame()
{	
		//printf("Start a picture.\r\n");
		while(!V_flag);
		while(V_flag);
		//printf("End a picture.\r\n");
}

// *** HM01B0 streaming
void hm01b0_streaming()
{
		hm01b0_write_reg( (0x0100),	(0x01) );
}

// *** HM01B0 streaming
void hm01b0_hdtrig_streaming()
{
		hm01b0_write_reg( (0x0100),	(0x05) );
}



// *** One wire operation
void  hm01b0_8_bit_model()
{
		hm01b0_write_reg( (0x3059),	(0x00) );				// 8 bit model
		//hm01b0_write_reg( (0x3060),	(0x01<<1) );		// 1/8 divided frequency
}

void  hm01b0_set_model(uint8_t mode)
{
		hm01b0_write_reg( (0x3059),	mode );				// 8 bit model
		//hm01b0_write_reg( (0x3060),	(0x01<<1) );		// 1/8 divided frequency
}

hm01b0_status_e hm01b0_get_mode(uint8_t *pui8Mode)
{
    uint8_t ui8Data[1] = {0x01};
    bool ui32Err;

    ui32Err = hm01b0_read_reg(HM01B0_REG_MODE_SELECT, ui8Data, sizeof(ui8Data));

    *pui8Mode = ui8Data[0];
    
		if(ui32Err)return HM01B0_ERR_OK;
    else return HM01B0_ERR;
}


hm01b0_status_e hm01b0_set_mode(uint8_t ui8Mode, uint8_t ui8FrameCnt)
{
    bool ui32Err = true;

    if (ui8Mode == HM01B0_REG_MODE_SELECT_STREAMING_NFRAMES)
    {
        ui32Err = hm01b0_write_reg(HM01B0_REG_PMU_PROGRAMMABLE_FRAMECNT, ui8FrameCnt);
    }

    if(ui32Err)
    {
        ui32Err = hm01b0_write_reg(HM01B0_REG_MODE_SELECT, ui8Mode);
    }
    if(ui32Err)return HM01B0_ERR_OK;
    else return HM01B0_ERR;
}

// *** eight wires operation
void  hm01b0_1_bit_model()
{
		hm01b0_write_reg( (0x3059),	(0x01<<5) );				// 1 bit model
}

// *** eight wires operation
void  hm01b0_test_figure()
{
		hm01b0_write_reg( (0x0601),	(0x30) );				// 1 bit model
}


hm01b0_status_e hm01b0_cal_ae(uint8_t ui8CalFrames, uint8_t *pui8Buffer, uint32_t ui32BufferLen, hm01b0_ae_cfg_t* pAECfg)
{
    uint32_t        ui32Err     = HM01B0_ERR_OK;
    if(pAECfg == NULL){
        return HM01B0_ERR_PARAMS;
    }

    hm01b0_set_mode(HM01B0_REG_MODE_SELECT_STREAMING_HW_TRIGGER, ui8CalFrames);

    for (uint8_t i = 0; i < ui8CalFrames; i++)
    {
        nrf_gpio_pin_set(TRIG);
        hm01b0_spiread_oneframe(pui8Buffer, ui32BufferLen);
//        ui32Err = hm01b0_get_ae(pAECfg);
        nrf_gpio_pin_clear(TRIG);
        // // todo: could report out intermediate results here (without using printing - perhaps a callback function)
        // SERIAL_PORT.printf("AE Calibration(0x%02X) TargetMean 0x%02X, ConvergeInTh 0x%02X, AEMean 0x%02X\n",
        //                                 ui32Err, sAECfg.ui8AETargetMean, sAECfg.ui8ConvergeInTh, sAECfg.ui8AEMean);

        // if AE calibration is done in ui8CalFrames, just exit to save some time.
//        if (ui32Err == HM01B0_ERR_OK)
//            break;
    }

    hm01b0_set_mode(HM01B0_REG_MODE_SELECT_STANDBY, 0);

    return ui32Err;
}



hm01b0_status_e hm01b0_get_ae(hm01b0_ae_cfg_t *psAECfg)
{
    bool    ui32Err = false;
    uint8_t     ui8AETargetMean;
    uint8_t     ui8AEMinMean;
    uint8_t     ui8AEMean;
    uint8_t     ui8ConvergeInTh;
    uint8_t     ui8ConvergeOutTh;

    ui32Err = hm01b0_read_reg(HM01B0_REG_AE_TARGET_MEAN, &ui8AETargetMean, sizeof(ui8AETargetMean));
    if (!ui32Err) return HM01B0_ERR;

    ui32Err = hm01b0_read_reg(HM01B0_REG_AE_MIN_MEAN, &ui8AEMinMean, sizeof(ui8AEMinMean));
    if (!ui32Err) return HM01B0_ERR;

    ui32Err = hm01b0_read_reg(HM01B0_REG_CONVERGE_IN_TH, &ui8ConvergeInTh, sizeof(ui8ConvergeInTh));
    if (!ui32Err) return HM01B0_ERR;

    ui32Err = hm01b0_read_reg(HM01B0_REG_CONVERGE_OUT_TH, &ui8ConvergeOutTh, sizeof(ui8ConvergeOutTh));
    if (!ui32Err) return HM01B0_ERR;

    ui32Err = hm01b0_read_reg(0x2020, &ui8AEMean, sizeof(ui8AEMean));
    if (!ui32Err) return HM01B0_ERR;

    if ((ui8AEMean < (ui8AETargetMean - ui8ConvergeInTh)) || (ui8AEMean > (ui8AETargetMean + ui8ConvergeInTh)))
        ui32Err = false;

    if (psAECfg)
    {
        psAECfg->ui8AETargetMean    = ui8AETargetMean;
        psAECfg->ui8AEMinMean       = ui8AEMinMean;
        psAECfg->ui8ConvergeInTh    = ui8ConvergeInTh;
        psAECfg->ui8ConvergeOutTh   = ui8ConvergeOutTh;
        psAECfg->ui8AEMean          = ui8AEMean;
    }

    if(ui32Err)return HM01B0_ERR_OK;
    else return HM01B0_ERR;
}

// *** auto set the exploure
void autoexposure( uint8_t expnum )
{
		for (uint8_t i = 0; i < expnum; i++)
    {
			nrf_gpio_pin_set(TRIG);
			
			hm01b0_wait_one_frame();
			//hm01b0_get_ae();
			
			nrf_gpio_pin_clear(TRIG);
    }
}

// *** cmd update
void hm01b0_cmd_update()
{
		hm01b0_write_reg(0x0104,0x01);				// 1 bit model
}

// *** Main function Interface
void hm01b0_init()
{
		hm01b0_pin_init();
		hm01b0_spi_init();
    hm01b0_twi_init(); 
	  //hm01b0_init_fixed_rom_qvga_fixed();
		hm01b0_init_regs(hm01b0_324x244);

}

void hm01b0_init_fixed_rom_qvga_fixed(void)
{

    //    hm01b0_write_reg(REG_SW_RESET, 0x00); //Software reset, reset all serial interface registers to its default values
    hm01b0_write_reg(REG_MODE_SELECT, 0x00);               // go to stand by mode
    hm01b0_write_reg(REG_ANA_REGISTER_17, 0x00);           // register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default
    hm01b0_write_reg(REG_TEST_PATTERN_MODE, TEST_PATTERN); // Enable the test pattern, set it to walking 1

    hm01b0_write_reg(REG_BIN_MODE, 0x00);    // VERTICAL BIN MODE
    hm01b0_write_reg(REG_QVGA_WIN_EN, 0x01); // Set line length LSB to QQVGA => enabled: makes the image 160(row)*240(col)
    //    disable: image 160*320 //In test pattern mode, enabling this does not have any effect

    /*looking at lattice cfg setting*/
    // hm01b0_write_reg(0x0103,0x00);

    // 100*100 optimization
    hm01b0_write_reg(REG_BIN_RDOUT_X, 0x01); // Horizontal Binning enable
    hm01b0_write_reg(REG_BIN_RDOUT_Y, 0x01); // vertical Binning enable => this register should be always 0x03 because we never go more than 160 for the height
                                         // frame timing control
    hm01b0_write_reg(REG_FRAME_LENGTH_PCK_H, 0x01);
    hm01b0_write_reg(REG_FRAME_LENGTH_PCK_L, 0x78); // changed by Ali

    hm01b0_write_reg(REG_FRAME_LENGTH_LINES_H, 0x02); // changed by Ali
    hm01b0_write_reg(REG_FRAME_LENGTH_LINES_L, 0x12); // changed by Ali

    /*looking at lattice cfg setting*/
    // hm01b0_write_reg(0x0103,0x00);

    hm01b0_write_reg(0x3044, 0x0A);
    hm01b0_write_reg(0x3045, 0x00);
    hm01b0_write_reg(0x3047, 0x0A);
    hm01b0_write_reg(0x3050, 0xC0);
    hm01b0_write_reg(0x3051, 0x42);
    //    hm01b0_write_reg(0x3052,0x50);
    hm01b0_write_reg(0x3053, 0x00);
    hm01b0_write_reg(0x3054, 0x03);
    hm01b0_write_reg(0x3055, 0xF7);
    hm01b0_write_reg(0x3056, 0xF8);
    hm01b0_write_reg(0x3057, 0x29);
    hm01b0_write_reg(0x3058, 0x1F);
    //    hm01b0_write_reg(0x3059,0x1E);//bit control
    hm01b0_write_reg(0x3064, 0x00);
    hm01b0_write_reg(0x3065, 0x04);

    // black level control
    hm01b0_write_reg(0x1000, 0x43);
    hm01b0_write_reg(0x1001, 0x40);
    hm01b0_write_reg(0x1002, 0x32);
    hm01b0_write_reg(0x1003, 0x08); // default from lattice 0x08
    hm01b0_write_reg(0x1006, 0x01);
    hm01b0_write_reg(0x1007, 0x08); // default from lattice 0x08

    hm01b0_write_reg(0x0350, 0x7F);

    // Sensor reserved
    hm01b0_write_reg(0x1008, 0x00);
    hm01b0_write_reg(0x1009, 0xA0);
    hm01b0_write_reg(0x100A, 0x60);
    hm01b0_write_reg(0x100B, 0x90); // default from lattice 0x90
    hm01b0_write_reg(0x100C, 0x40); // default from lattice 0x40

    // Vsync, hsync and pixel shift register
    //    hm01b0_write_reg(0x1012,0x07);//changed by Ali
    hm01b0_write_reg(0x1012, 0x00); // lattice value

    // Statistic control and read only
    hm01b0_write_reg(0x2000, 0x07);
    hm01b0_write_reg(0x2003, 0x00);
    hm01b0_write_reg(0x2004, 0x1C);
    hm01b0_write_reg(0x2007, 0x00);
    hm01b0_write_reg(0x2008, 0x58);
    hm01b0_write_reg(0x200B, 0x00);
    hm01b0_write_reg(0x200C, 0x7A);
    hm01b0_write_reg(0x200F, 0x00);
    hm01b0_write_reg(0x2010, 0xB8);
    hm01b0_write_reg(0x2013, 0x00);
    hm01b0_write_reg(0x2014, 0x58);
    hm01b0_write_reg(0x2017, 0x00);
    hm01b0_write_reg(0x2018, 0x9B);

    // Automatic exposure gain control
    hm01b0_write_reg(0x2100, 0x01);
    hm01b0_write_reg(0x2101, 0x70); // 0x70);//lattice 0xA0
    hm01b0_write_reg(0x2102, 0x01); // lattice 0x06
    hm01b0_write_reg(0x2104, 0x07);
    hm01b0_write_reg(0x2105, 0x03);
    hm01b0_write_reg(0x2106, 0xA4);
    hm01b0_write_reg(0x2108, 0x33);
    hm01b0_write_reg(0x210A, 0x00);
    // hm01b0_write_reg(0x210C,0x04);
    hm01b0_write_reg(0x210B, 0x80);
    hm01b0_write_reg(0x210F, 0x00);
    hm01b0_write_reg(0x2110, 0xE9);
    hm01b0_write_reg(0x2111, 0x01);
    hm01b0_write_reg(0x2112, 0x17);
    hm01b0_write_reg(0x2150, 0x03);

    // Sensor exposure gain
    hm01b0_write_reg(0x0205, 0x05); // Vikram
    hm01b0_write_reg(0x020E, 0x01); // Vikram
    hm01b0_write_reg(0x020F, 0x00); // Vikram
    hm01b0_write_reg(0x0202, 0x01); // Vikram
    hm01b0_write_reg(0x0203, 0x08); // Vikram

    // frame timing control
    //    hm01b0_write_reg(0x0340,0x02);//changed by Ali
    //    hm01b0_write_reg(0x0341,0x32);//changed by Ali
    ////    hm01b0_write_reg(0x0340,0x0C);
    ////    hm01b0_write_reg(0x0341,0x5C);
    //
    //    hm01b0_write_reg(0x0342,0x01);
    //    hm01b0_write_reg(0x0343,0x78);//changed by Ali
    //    hm01b0_write_reg(0x0343,0x78);

    //    hm01b0_write_reg(0x3010,0x01); //done in lower lines
    //    hm01b0_write_reg(0x0383,0x00); //done in lower lines
    //    hm01b0_write_reg(0x0387,0x00); //done in lower lines
    //    hm01b0_write_reg(0x0390,0x00); //done in lower lines
    //    hm01b0_write_reg(0x3059,0x42); //done in lower lines
    //    hm01b0_write_reg(0x3060,0x51); //done in lower lines

    //    hm01b0_write_reg(0x0101,0x00);//this part gives error
    //    hm01b0_write_reg(0x0100,0x05);

    ////    hm01b0_write_reg(0x3061,0x20);
    ////    hm01b0_write_reg(0x3067,0x01);

    //    hm01b0_write_reg(0x0104,0xFF);//changed by Ali
    //    hm01b0_write_reg(0x0104,0x00);//makes the image look crooked!

    // hm01b0_write_reg(0x0205,0x30);

    /*looking at lattice cfg setting*/

    //    hm01b0_write_reg( 0x3044, 0x0A);/*this part gives error*/

    //    i2c_write(REG_FRAME_LENGTH_LINES_H, 0x00);//Set frame length lines MSB to QQVGA :
    //    i2c_write(REG_FRAME_LENGTH_LINES_L, 0xD7);//Set frame length lines LSB to QQVGA : 0xD7 = 215
    //
    //    i2c_write(REG_FRAME_LENGTH_PCK_H, 0x00);//Set line length MSB to QQVGA
    //    i2c_write(REG_FRAME_LENGTH_PCK_L, 0x80);//Set line length LSB to QQVGA : 0x80 = 128

    //    hm01b0_write_reg( REG_OSC_CLK_DIV, 0x20);//This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data
    hm01b0_write_reg(REG_OSC_CLK_DIV, 0x30); // This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

    hm01b0_write_reg(REG_BIT_CONTROL, 0x20); // Set the output to send 1 bit serial

    hm01b0_write_reg(REG_PMU_PROGRAMMABLE_FRAMECNT, 0x01); // set the number of frames to be sent out, it sends N frames
}

void hm01b0_init_fixed_rom_qvga_fixed_acc(void)
{

    //    hm01b0_write_reg(REG_SW_RESET, 0x00); //Software reset, reset all serial interface registers to its default values
    hm01b0_write_reg(REG_MODE_SELECT, 0x00);               // go to stand by mode
    hm01b0_write_reg(REG_ANA_REGISTER_17, 0x00);           // register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default
    hm01b0_write_reg(REG_TEST_PATTERN_MODE, TEST_PATTERN); // Enable the test pattern, set it to walking 1

    hm01b0_write_reg(REG_BIN_MODE, 0x00);    // VERTICAL BIN MODE
    hm01b0_write_reg(REG_QVGA_WIN_EN, 0x01); // Set line length LSB to QQVGA => enabled: makes the image 160(row)*240(col)
    //    disable: image 160*320 //In test pattern mode, enabling this does not have any effect

    /*looking at lattice cfg setting*/
    // hm01b0_write_reg(0x0103,0x00);

    // 100*100 optimization
    hm01b0_write_reg(REG_BIN_RDOUT_X, 0x01); // Horizontal Binning enable
    hm01b0_write_reg(REG_BIN_RDOUT_Y, 0x01); // vertical Binning enable => this register should be always 0x03 because we never go more than 160 for the height
                                         // frame timing control
    hm01b0_write_reg(REG_FRAME_LENGTH_PCK_H, 0x01);
    hm01b0_write_reg(REG_FRAME_LENGTH_PCK_L, 0x78); // changed by Ali

    hm01b0_write_reg(REG_FRAME_LENGTH_LINES_H, 0x02); // changed by Ali
    hm01b0_write_reg(REG_FRAME_LENGTH_LINES_L, 0x12); // changed by Ali

    /*looking at lattice cfg setting*/
    // hm01b0_write_reg(0x0103,0x00);

    //    hm01b0_write_reg(0x3044,0x0A);
    //    hm01b0_write_reg(0x3045,0x00);
    //    hm01b0_write_reg(0x3047,0x0A);
    //    hm01b0_write_reg(0x3050,0xC0);
    //    hm01b0_write_reg(0x3051,0x42);
    ////    hm01b0_write_reg(0x3052,0x50);
    //    hm01b0_write_reg(0x3053,0x00);
    //    hm01b0_write_reg(0x3054,0x03);
    //    hm01b0_write_reg(0x3055,0xF7);
    //    hm01b0_write_reg(0x3056,0xF8);
    //    hm01b0_write_reg(0x3057,0x29);
    //    hm01b0_write_reg(0x3058,0x1F);
    ////    hm01b0_write_reg(0x3059,0x1E);//bit control
    //    hm01b0_write_reg(0x3064,0x00);
    //    hm01b0_write_reg(0x3065,0x04);
    //
    //    //black level control
    //    hm01b0_write_reg(0x1000,0x43);
    //    hm01b0_write_reg(0x1001,0x40);
    //    hm01b0_write_reg(0x1002,0x32);
    //    hm01b0_write_reg(0x1003,0x08);//default from lattice 0x08
    //    hm01b0_write_reg(0x1006,0x01);
    //    hm01b0_write_reg(0x1007,0x08);//default from lattice 0x08
    //
    //    hm01b0_write_reg(0x0350,0x7F);

    //    //Sensor reserved
    //    hm01b0_write_reg(0x1008,0x00);
    //    hm01b0_write_reg(0x1009,0xA0);
    //    hm01b0_write_reg(0x100A,0x60);
    //    hm01b0_write_reg(0x100B,0x90);//default from lattice 0x90
    //    hm01b0_write_reg(0x100C,0x40);//default from lattice 0x40
    //
    //    //Vsync, hsync and pixel shift register
    ////    hm01b0_write_reg(0x1012,0x07);//changed by Ali
    //    hm01b0_write_reg(0x1012,0x00);//lattice value

    // Statistic control and read only
    //    hm01b0_write_reg(0x2000,0x07);
    //    hm01b0_write_reg(0x2003,0x00);
    //    hm01b0_write_reg(0x2004,0x1C);
    //    hm01b0_write_reg(0x2007,0x00);
    //    hm01b0_write_reg(0x2008,0x58);
    //    hm01b0_write_reg(0x200B,0x00);
    //    hm01b0_write_reg(0x200C,0x7A);
    //    hm01b0_write_reg(0x200F,0x00);
    //    hm01b0_write_reg(0x2010,0xB8);
    //    hm01b0_write_reg(0x2013,0x00);
    //    hm01b0_write_reg(0x2014,0x58);
    //    hm01b0_write_reg(0x2017,0x00);
    //    hm01b0_write_reg(0x2018,0x9B);

    // Automatic exposure gain control
    //    hm01b0_write_reg(0x2100,0x01);
    //    hm01b0_write_reg(0x2101,0x70);//0x70);//lattice 0xA0
    //    hm01b0_write_reg(0x2102,0x01);//lattice 0x06
    //    hm01b0_write_reg(0x2104,0x07);
    //    hm01b0_write_reg(0x2105,0x03);
    //    hm01b0_write_reg(0x2106,0xA4);
    //    hm01b0_write_reg(0x2108,0x33);
    //    hm01b0_write_reg(0x210A,0x00);
    //    //hm01b0_write_reg(0x210C,0x04);
    //    hm01b0_write_reg(0x210B,0x80);
    //    hm01b0_write_reg(0x210F,0x00);
    //    hm01b0_write_reg(0x2110,0xE9);
    //    hm01b0_write_reg(0x2111,0x01);
    //    hm01b0_write_reg(0x2112,0x17);
    //    hm01b0_write_reg(0x2150,0x03);
    //
    //    //Sensor exposure gain
    //    hm01b0_write_reg(0x0205,0x05);//Vikram
    //    hm01b0_write_reg(0x020E,0x01);//Vikram
    //    hm01b0_write_reg(0x020F,0x00);//Vikram
    //    hm01b0_write_reg(0x0202,0x01);//Vikram
    //    hm01b0_write_reg(0x0203,0x08);//Vikram

    // frame timing control
    //    hm01b0_write_reg(0x0340,0x02);//changed by Ali
    //    hm01b0_write_reg(0x0341,0x32);//changed by Ali
    ////    hm01b0_write_reg(0x0340,0x0C);
    ////    hm01b0_write_reg(0x0341,0x5C);
    //
    //    hm01b0_write_reg(0x0342,0x01);
    //    hm01b0_write_reg(0x0343,0x78);//changed by Ali
    //    hm01b0_write_reg(0x0343,0x78);

    //    hm01b0_write_reg(0x3010,0x01); //done in lower lines
    //    hm01b0_write_reg(0x0383,0x00); //done in lower lines
    //    hm01b0_write_reg(0x0387,0x00); //done in lower lines
    //    hm01b0_write_reg(0x0390,0x00); //done in lower lines
    //    hm01b0_write_reg(0x3059,0x42); //done in lower lines
    //    hm01b0_write_reg(0x3060,0x51); //done in lower lines

    //    hm01b0_write_reg(0x0101,0x00);//this part gives error
    //    hm01b0_write_reg(0x0100,0x05);

    ////    hm01b0_write_reg(0x3061,0x20);
    ////    hm01b0_write_reg(0x3067,0x01);

    //    hm01b0_write_reg(0x0104,0xFF);//changed by Ali
    //    hm01b0_write_reg(0x0104,0x00);//makes the image look crooked!

    // hm01b0_write_reg(0x0205,0x30);

    /*looking at lattice cfg setting*/

    //    hm01b0_write_reg( 0x3044, 0x0A);/*this part gives error*/

    //    i2c_write(REG_FRAME_LENGTH_LINES_H, 0x00);//Set frame length lines MSB to QQVGA :
    //    i2c_write(REG_FRAME_LENGTH_LINES_L, 0xD7);//Set frame length lines LSB to QQVGA : 0xD7 = 215
    //
    //    i2c_write(REG_FRAME_LENGTH_PCK_H, 0x00);//Set line length MSB to QQVGA
    //    i2c_write(REG_FRAME_LENGTH_PCK_L, 0x80);//Set line length LSB to QQVGA : 0x80 = 128

    //    hm01b0_write_reg( REG_OSC_CLK_DIV, 0x20);//This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data
    hm01b0_write_reg(REG_OSC_CLK_DIV, 0x30); // This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

    hm01b0_write_reg(REG_BIT_CONTROL, 0x20); // Set the output to send 1 bit serial

    hm01b0_write_reg(REG_PMU_PROGRAMMABLE_FRAMECNT, 0x01); // set the number of frames to be sent out, it sends N frames
}

void hm01b0_init_fixed_rom_qvga_fixed_maxfps(void)
{

    //    hm01b0_write_reg(REG_SW_RESET, 0x00); //Software reset, reset all serial interface registers to its default values
    hm01b0_write_reg(REG_MODE_SELECT, 0x00);       // go to stand by mode
    hm01b0_write_reg(REG_ANA_REGISTER_17, 0x00);   // register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default
    hm01b0_write_reg(REG_TEST_PATTERN_MODE, 0x01); // Enable the test pattern, set it to walking 1

    /*looking at lattice cfg setting*/
    //    hm01b0_write_reg(0x0103,0x00);

    hm01b0_write_reg(0x1003, 0x08);
    hm01b0_write_reg(0x1007, 0x08);
    hm01b0_write_reg(0x3044, 0x0A);
    hm01b0_write_reg(0x3045, 0x00);
    hm01b0_write_reg(0x3047, 0x0A);
    hm01b0_write_reg(0x3050, 0xC0);
    hm01b0_write_reg(0x3051, 0x42);
    hm01b0_write_reg(0x3052, 0x50);
    hm01b0_write_reg(0x3053, 0x00);
    hm01b0_write_reg(0x3054, 0x03);
    hm01b0_write_reg(0x3055, 0xF7);
    hm01b0_write_reg(0x3056, 0xF8);
    hm01b0_write_reg(0x3057, 0x29);
    hm01b0_write_reg(0x3058, 0x1F);
    //    hm01b0_write_reg(0x3059,0x1E);//bit control
    hm01b0_write_reg(0x3064, 0x00);
    hm01b0_write_reg(0x3065, 0x04);

    // black level control
    hm01b0_write_reg(0x1000, 0x43);
    hm01b0_write_reg(0x1001, 0x40);
    hm01b0_write_reg(0x1002, 0x32);
    hm01b0_write_reg(0x0350, 0x7F);
    hm01b0_write_reg(0x1006, 0x01);

    // Sensor reserved
    hm01b0_write_reg(0x1008, 0x00);
    hm01b0_write_reg(0x1009, 0xA0);
    hm01b0_write_reg(0x100A, 0x60);
    hm01b0_write_reg(0x100B, 0x90);
    hm01b0_write_reg(0x100C, 0x40);

    // Vsync, hsync and pixel shift register
    hm01b0_write_reg(0x1012, 0x07); // changed by Ali
    //    hm01b0_write_reg(0x1012,0x00);

    // Statistic control and read only
    hm01b0_write_reg(0x2000, 0x07);
    hm01b0_write_reg(0x2003, 0x00);
    hm01b0_write_reg(0x2004, 0x1C);
    hm01b0_write_reg(0x2007, 0x00);
    hm01b0_write_reg(0x2008, 0x58);
    hm01b0_write_reg(0x200B, 0x00);
    hm01b0_write_reg(0x200C, 0x7A);
    hm01b0_write_reg(0x200F, 0x00);
    hm01b0_write_reg(0x2010, 0xB8);
    hm01b0_write_reg(0x2013, 0x00);
    hm01b0_write_reg(0x2014, 0x58);
    hm01b0_write_reg(0x2017, 0x00);
    hm01b0_write_reg(0x2018, 0x9B);

    // Automatic exposure gain control
    hm01b0_write_reg(0x2100, 0x00);
    hm01b0_write_reg(0x2104, 0x07);
    hm01b0_write_reg(0x2105, 0x03);
    hm01b0_write_reg(0x2106, 0xA4);
    hm01b0_write_reg(0x2108, 0x33);
    hm01b0_write_reg(0x210A, 0x00);
    hm01b0_write_reg(0x210B, 0x80);
    hm01b0_write_reg(0x210F, 0x00);
    hm01b0_write_reg(0x2110, 0xE9);
    hm01b0_write_reg(0x2111, 0x01);
    hm01b0_write_reg(0x2112, 0x17);
    hm01b0_write_reg(0x2150, 0x03);

    // frame timing control
    hm01b0_write_reg(0x0340, 0x02); // changed by Ali
    hm01b0_write_reg(0x0341, 0x32); // changed by Ali
    //    hm01b0_write_reg(0x0340,0x0C);
    //    hm01b0_write_reg(0x0341,0x5C);

    hm01b0_write_reg(0x0342, 0x01);
    hm01b0_write_reg(0x0343, 0x72); // changed by Ali
    //    hm01b0_write_reg(0x0343,0x78);

    //    hm01b0_write_reg(0x3010,0x01); //done in lower lines
    //    hm01b0_write_reg(0x0383,0x00); //done in lower lines
    //    hm01b0_write_reg(0x0387,0x00); //done in lower lines
    //    hm01b0_write_reg(0x0390,0x00); //done in lower lines
    //    hm01b0_write_reg(0x3059,0x42); //done in lower lines
    //    hm01b0_write_reg(0x3060,0x51); //done in lower lines

    //    hm01b0_write_reg(0x0101,0x00);/*this part gives error*/
    //    hm01b0_write_reg(0x0100,0x05);
    //    hm01b0_write_reg(0x2101,0x80);
    //    hm01b0_write_reg(0x2102,0x40);
    //    hm01b0_write_reg(0x020F,0x00);
    //    hm01b0_write_reg(0x3061,0x20);
    //    hm01b0_write_reg(0x3067,0x01);
    //    hm01b0_write_reg(0x0104,0x00);
    //    hm01b0_write_reg(0x0205,0x30);
    /*looking at lattice cfg setting*/

    //    hm01b0_write_reg( 0x3044, 0x0A);/*this part gives error*/

    //    i2c_write(REG_FRAME_LENGTH_LINES_H, 0x00);//Set frame length lines MSB to QQVGA :
    //    i2c_write(REG_FRAME_LENGTH_LINES_L, 0xD7);//Set frame length lines LSB to QQVGA : 0xD7 = 215
    //
    //    i2c_write(REG_FRAME_LENGTH_PCK_H, 0x00);//Set line length MSB to QQVGA
    //    i2c_write(REG_FRAME_LENGTH_PCK_L, 0x80);//Set line length LSB to QQVGA : 0x80 = 128

    hm01b0_write_reg(REG_QVGA_WIN_EN, 0x01); // Set line length LSB to QQVGA => enabled: makes the image 160(row)*240(col)
    //    disable: image 160*320 //In test pattern mode, enabling this does not have any effect

    hm01b0_write_reg(REG_OSC_CLK_DIV, 0x20); // This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

    hm01b0_write_reg(REG_BIN_RDOUT_X, 0x03); // Horizontal Binning enable
    hm01b0_write_reg(REG_BIN_RDOUT_Y, 0x03); // vertical Binning enable
    hm01b0_write_reg(REG_BIN_MODE, 0x03);    // VERTICAL BIN MODE

    hm01b0_write_reg(REG_BIT_CONTROL, 0x20); // Set the output to send 1 bit serial

    hm01b0_write_reg(REG_PMU_PROGRAMMABLE_FRAMECNT, 0x01); // set the number of frames to be sent out, it sends N frames
}

void hm01b0_init_brighter(void)
{

    //    hm01b0_write_reg(REG_SW_RESET, 0x00); //Software reset, reset all serial interface registers to its default values
    hm01b0_write_reg(REG_MODE_SELECT, 0x00);       // go to stand by mode
    hm01b0_write_reg(REG_ANA_REGISTER_17, 0x00);   // register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default
    hm01b0_write_reg(REG_TEST_PATTERN_MODE, 0x00); // Enable the test pattern, set it to walking 1

    /*looking at lattice cfg setting*/
    //    hm01b0_write_reg(0x0103,0x00);

    hm01b0_write_reg(0x1003, 0x08);
    hm01b0_write_reg(0x1007, 0x08);
    hm01b0_write_reg(0x3044, 0x0A);
    hm01b0_write_reg(0x3045, 0x00);
    hm01b0_write_reg(0x3047, 0x0A);
    hm01b0_write_reg(0x3050, 0xC0);
    hm01b0_write_reg(0x3051, 0x42);
    hm01b0_write_reg(0x3052, 0x50);
    hm01b0_write_reg(0x3053, 0x00);
    hm01b0_write_reg(0x3054, 0x03);
    hm01b0_write_reg(0x3055, 0xF7);
    hm01b0_write_reg(0x3056, 0xF8);
    hm01b0_write_reg(0x3057, 0x29);
    hm01b0_write_reg(0x3058, 0x1F);
    //    hm01b0_write_reg(0x3059,0x1E);//bit control
    hm01b0_write_reg(0x3064, 0x00);
    hm01b0_write_reg(0x3065, 0x04);

    // black level control
    hm01b0_write_reg(0x1000, 0x00); // changed by Ali
    //    hm01b0_write_reg(0x1000,0x43);
    hm01b0_write_reg(0x1001, 0x40);
    hm01b0_write_reg(0x1002, 0x32);
    hm01b0_write_reg(0x0350, 0x7F);
    hm01b0_write_reg(0x1006, 0x01);

    // Sensor reserved
    hm01b0_write_reg(0x1008, 0x00);
    hm01b0_write_reg(0x1009, 0xA0);
    hm01b0_write_reg(0x100A, 0x60);
    hm01b0_write_reg(0x100B, 0x90);
    hm01b0_write_reg(0x100C, 0x40);

    // Vsync, hsync and pixel shift register
    hm01b0_write_reg(0x1012, 0x07); // changed by Ali
    //    hm01b0_write_reg(0x1012,0x00);

    // Statistic control and read only
    hm01b0_write_reg(0x2000, 0x07);
    hm01b0_write_reg(0x2003, 0x00);
    hm01b0_write_reg(0x2004, 0x1C);
    hm01b0_write_reg(0x2007, 0x00);
    hm01b0_write_reg(0x2008, 0x58);
    hm01b0_write_reg(0x200B, 0x00);
    hm01b0_write_reg(0x200C, 0x7A);
    hm01b0_write_reg(0x200F, 0x00);
    hm01b0_write_reg(0x2010, 0xB8);
    hm01b0_write_reg(0x2013, 0x00);
    hm01b0_write_reg(0x2014, 0x58);
    hm01b0_write_reg(0x2017, 0x00);
    hm01b0_write_reg(0x2018, 0x9B);

    // Automatic exposure gain control
    hm01b0_write_reg(0x2100, 0x00); // changed by Ali
    //    hm01b0_write_reg(0x2100,0x01);
    hm01b0_write_reg(0x2104, 0x07);
    hm01b0_write_reg(0x2105, 0x03);
    hm01b0_write_reg(0x2106, 0xA4);
    hm01b0_write_reg(0x2108, 0x33);
    hm01b0_write_reg(0x210A, 0x00);
    hm01b0_write_reg(0x210B, 0x80);
    hm01b0_write_reg(0x210F, 0x00);
    hm01b0_write_reg(0x2110, 0xE9);
    hm01b0_write_reg(0x2111, 0x01);
    hm01b0_write_reg(0x2112, 0x17);
    hm01b0_write_reg(0x2150, 0x03);

    // frame timing control
    hm01b0_write_reg(0x0340, 0x02); // changed by Ali
    hm01b0_write_reg(0x0341, 0x32); // changed by Ali
    //    hm01b0_write_reg(0x0340,0x0C);
    //    hm01b0_write_reg(0x0341,0x5C);

    hm01b0_write_reg(0x0342, 0x01);
    hm01b0_write_reg(0x0343, 0x72); // changed by Ali
    //    hm01b0_write_reg(0x0343,0x78);

    //    hm01b0_write_reg(0x3010,0x01); //done in lower lines
    //    hm01b0_write_reg(0x0383,0x00); //done in lower lines
    //    hm01b0_write_reg(0x0387,0x00); //done in lower lines
    //    hm01b0_write_reg(0x0390,0x00); //done in lower lines
    //    hm01b0_write_reg(0x3059,0x42); //done in lower lines
    //    hm01b0_write_reg(0x3060,0x51); //done in lower lines

    //    hm01b0_write_reg(0x0101,0x00);/*this part gives error*/
    //    hm01b0_write_reg(0x0100,0x05);

    hm01b0_write_reg(0x2101, 0x00);
    hm01b0_write_reg(0x2102, 0x00);
    //    hm01b0_write_reg(0x2101,0xA0);//changed by Ali
    //    hm01b0_write_reg(0x2102,0x60);//changed by Ali
    hm01b0_write_reg(0x020F, 0x00);
    ////    hm01b0_write_reg(0x3061,0x20);
    ////    hm01b0_write_reg(0x3067,0x01);

    //    hm01b0_write_reg(0x0104,0xFF);//changed by Ali
    //    hm01b0_write_reg(0x0104,0x00);//makes the image look crooked!

    hm01b0_write_reg(0x0205, 0x30);
    /*looking at lattice cfg setting*/

    //    hm01b0_write_reg( 0x3044, 0x0A);/*this part gives error*/

    //    i2c_write(REG_FRAME_LENGTH_LINES_H, 0x00);//Set frame length lines MSB to QQVGA :
    //    i2c_write(REG_FRAME_LENGTH_LINES_L, 0xD7);//Set frame length lines LSB to QQVGA : 0xD7 = 215
    //
    //    i2c_write(REG_FRAME_LENGTH_PCK_H, 0x00);//Set line length MSB to QQVGA
    //    i2c_write(REG_FRAME_LENGTH_PCK_L, 0x80);//Set line length LSB to QQVGA : 0x80 = 128

    hm01b0_write_reg(REG_QVGA_WIN_EN, 0x01); // Set line length LSB to QQVGA => enabled: makes the image 160(row)*240(col)
    //    disable: image 160*320 //In test pattern mode, enabling this does not have any effect

    hm01b0_write_reg(REG_OSC_CLK_DIV, 0x20); // This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

    hm01b0_write_reg(REG_BIN_RDOUT_X, 0x03); // Horizontal Binning enable
    hm01b0_write_reg(REG_BIN_RDOUT_Y, 0x03); // vertical Binning enable
    hm01b0_write_reg(REG_BIN_MODE, 0x03);    // VERTICAL BIN MODE

    hm01b0_write_reg(REG_BIT_CONTROL, 0x20); // Set the output to send 1 bit serial

    hm01b0_write_reg(REG_PMU_PROGRAMMABLE_FRAMECNT, 0x01); // set the number of frames to be sent out, it sends N frames
}

void hm01b0_init_datasheet_default(void)
{

    //    hm01b0_write_reg(REG_SW_RESET, 0x00); //Software reset, reset all serial interface registers to its default values
    hm01b0_write_reg(REG_MODE_SELECT, 0x00);       // go to stand by mode
    hm01b0_write_reg(REG_ANA_REGISTER_17, 0x00);   // register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default
    hm01b0_write_reg(REG_TEST_PATTERN_MODE, 0x00); // Enable the test pattern, set it to walking 1

    /*looking at lattice cfg setting*/
    //    hm01b0_write_reg(0x0103,0x00);

    //    hm01b0_write_reg(0x1003,0x08);
    //    hm01b0_write_reg(0x1007,0x08);
    //    hm01b0_write_reg(0x3044,0x0A);
    //    hm01b0_write_reg(0x3045,0x00);
    //    hm01b0_write_reg(0x3047,0x0A);
    //    hm01b0_write_reg(0x3050,0xC0);
    //    hm01b0_write_reg(0x3051,0x42);
    //    hm01b0_write_reg(0x3052,0x50);
    //    hm01b0_write_reg(0x3053,0x00);
    //    hm01b0_write_reg(0x3054,0x03);
    //    hm01b0_write_reg(0x3055,0xF7);
    //    hm01b0_write_reg(0x3056,0xF8);
    //    hm01b0_write_reg(0x3057,0x29);
    //    hm01b0_write_reg(0x3058,0x1F);
    ////    hm01b0_write_reg(0x3059,0x1E);//bit control
    //    hm01b0_write_reg(0x3064,0x00);
    //    hm01b0_write_reg(0x3065,0x04);
    //
    //    //black level control
    //    hm01b0_write_reg(0x1000,0x43);
    //    hm01b0_write_reg(0x1001,0x40);
    //    hm01b0_write_reg(0x1002,0x32);
    //    hm01b0_write_reg(0x0350,0x7F);
    //    hm01b0_write_reg(0x1006,0x01);
    //
    //    //Sensor reserved
    //    hm01b0_write_reg(0x1008,0x00);
    //    hm01b0_write_reg(0x1009,0xA0);
    //    hm01b0_write_reg(0x100A,0x60);
    //    hm01b0_write_reg(0x100B,0x90);
    //    hm01b0_write_reg(0x100C,0x40);
    //
    //    //Vsync, hsync and pixel shift register
    //    hm01b0_write_reg(0x1012,0x07);//changed by Ali
    ////    hm01b0_write_reg(0x1012,0x00);
    //
    //    //Statistic control and read only
    //    hm01b0_write_reg(0x2000,0x07);
    //    hm01b0_write_reg(0x2003,0x00);
    //    hm01b0_write_reg(0x2004,0x1C);
    //    hm01b0_write_reg(0x2007,0x00);
    //    hm01b0_write_reg(0x2008,0x58);
    //    hm01b0_write_reg(0x200B,0x00);
    //    hm01b0_write_reg(0x200C,0x7A);
    //    hm01b0_write_reg(0x200F,0x00);
    //    hm01b0_write_reg(0x2010,0xB8);
    //    hm01b0_write_reg(0x2013,0x00);
    //    hm01b0_write_reg(0x2014,0x58);
    //    hm01b0_write_reg(0x2017,0x00);
    //    hm01b0_write_reg(0x2018,0x9B);
    //
    //    //Automatic exposure gain control
    //    hm01b0_write_reg(0x2100,0x00);
    //    hm01b0_write_reg(0x2104,0x07);
    //    hm01b0_write_reg(0x2105,0x03);
    //    hm01b0_write_reg(0x2106,0xA4);
    //    hm01b0_write_reg(0x2108,0x33);
    //    hm01b0_write_reg(0x210A,0x00);
    //    hm01b0_write_reg(0x210B,0x80);
    //    hm01b0_write_reg(0x210F,0x00);
    //    hm01b0_write_reg(0x2110,0xE9);
    //    hm01b0_write_reg(0x2111,0x01);
    //    hm01b0_write_reg(0x2112,0x17);
    //    hm01b0_write_reg(0x2150,0x03);
    //
    //    //frame timing control
    //    hm01b0_write_reg(0x0340,0x02);//changed by Ali
    //    hm01b0_write_reg(0x0341,0x32);//changed by Ali
    ////    hm01b0_write_reg(0x0340,0x0C);
    ////    hm01b0_write_reg(0x0341,0x5C);
    //
    //    hm01b0_write_reg(0x0342,0x01);
    //    hm01b0_write_reg(0x0343,0x72);//changed by Ali
    ////    hm01b0_write_reg(0x0343,0x78);
    //
    ////    hm01b0_write_reg(0x3010,0x01); //done in lower lines
    ////    hm01b0_write_reg(0x0383,0x00); //done in lower lines
    ////    hm01b0_write_reg(0x0387,0x00); //done in lower lines
    ////    hm01b0_write_reg(0x0390,0x00); //done in lower lines
    ////    hm01b0_write_reg(0x3059,0x42); //done in lower lines
    ////    hm01b0_write_reg(0x3060,0x51); //done in lower lines
    //
    ////    hm01b0_write_reg(0x0101,0x00);/*this part gives error*/
    ////    hm01b0_write_reg(0x0100,0x05);
    //
    //    hm01b0_write_reg(0x2101,0x80);
    //    hm01b0_write_reg(0x2102,0x40);
    //    hm01b0_write_reg(0x020F,0x00);
    //////    hm01b0_write_reg(0x3061,0x20);
    //////    hm01b0_write_reg(0x3067,0x01);
    //
    ////    hm01b0_write_reg(0x0104,0xFF);//changed by Ali
    ////    hm01b0_write_reg(0x0104,0x00);//makes the image look crooked!
    //
    //    hm01b0_write_reg(0x0205,0x30);
    /*looking at lattice cfg setting*/

    //    hm01b0_write_reg( 0x3044, 0x0A);/*this part gives error*/

    //    i2c_write(REG_FRAME_LENGTH_LINES_H, 0x00);//Set frame length lines MSB to QQVGA :
    //    i2c_write(REG_FRAME_LENGTH_LINES_L, 0xD7);//Set frame length lines LSB to QQVGA : 0xD7 = 215
    //
    //    i2c_write(REG_FRAME_LENGTH_PCK_H, 0x00);//Set line length MSB to QQVGA
    //    i2c_write(REG_FRAME_LENGTH_PCK_L, 0x80);//Set line length LSB to QQVGA : 0x80 = 128

    hm01b0_write_reg(REG_QVGA_WIN_EN, 0x00); // Set line length LSB to QQVGA => enabled: makes the image 160(row)*240(col)
    //    disable: image 160*320 //In test pattern mode, enabling this does not have any effect

    hm01b0_write_reg(REG_OSC_CLK_DIV, 0x20); // This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

    hm01b0_write_reg(REG_BIN_RDOUT_X, 0x01); // Horizontal Binning enable
    hm01b0_write_reg(REG_BIN_RDOUT_Y, 0x03); // vertical Binning enable
    hm01b0_write_reg(REG_BIN_MODE, 0x01);    // VERTICAL BIN MODE

    hm01b0_write_reg(REG_BIT_CONTROL, 0x20); // Set the output to send 1 bit serial

    hm01b0_write_reg(REG_PMU_PROGRAMMABLE_FRAMECNT, 0x01); // set the number of frames to be sent out, it sends N frames
}

void test_temp()
{

}


