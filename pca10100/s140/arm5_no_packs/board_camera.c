#include <stdint.h>
#include <nrf_gpio.h>
#include <nrf_delay.h>
#include <stdio.h>
#include <string.h>
#include <nrf_drv_spi.h>
#include <nrf_drv_twi.h>
#include <nrf_drv_gpiote.h>

#include "nrf_log.h"
#include "board_camera.h"
#include "board_camera_init.h"
// #include "io_iic.h"

#define TWI_ADDRESSES       127
#define HM01B0_SENSOR_ID    0x24

#define ARDUINO_SCL_PIN     27
#define ARDUINO_SDA_PIN     26

#define HM01B0_PIN_VSYNC    17//17
#define HM01B0_PIN_HSYNC    20//20
#define HM01B0_PIN_PXCLK    22//19
#define HM01B0_PIN_FTRIG    20

#define SPI_SCK_PIN         28//22//-28//MCLK
#define SPI_MOSI_PIN        29//-NOT USED
#define SPI_MISO_PIN        30//16//-30//D0
#define SPI_SS_PIN          31//-NOT USED

#define SPIM_SCK_PIN         28//22//-28//MCLK
#define SPIM_MOSI_PIN        29//-NOT USED
#define SPIM_MISO_PIN        30//16//-30//D0
#define SPIM_SS_PIN          31//-NOT USED

#define GET_BIT(num, bit)   ((num>>bit) & 1)

#define TWI_INSTANCE  0
/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

#if 0
#define SPI_INSTANCE  3 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

static uint8_t       m_tx_buf[1] = {0xff};           /**< TX buffer. */
static uint8_t       m_rx_buf[120][245];    /**< RX buffer. */
static const uint16_t m_length = 245;        /**< Transfer length. */

static volatile bool fvld = false;
static volatile bool lvld = false;

void hm01b0_spi_captrue(void)
{
    uint8_t FVLD = 0;
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.sck_pin  = SPI_SCK_PIN;
    spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
    nrf_drv_spi_init(&spi, &spi_config, NULL, NULL);

    NRF_LOG_INFO("SPI captrue one frame started.");

    nrf_gpio_pin_set(SPI_SS_PIN);
    nrf_gpio_pin_clear(SPI_SS_PIN);
    memset(m_rx_buf, 0, m_length);
    fvld = false;
    while(fvld == true);
    for (int i = 0; i < 120; ++i)
    {
        lvld = false;
        while(lvld == true);
        nrf_gpio_pin_set(SPI_SS_PIN);
        nrf_gpio_pin_clear(SPI_SS_PIN);
        nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf[i], m_length);
        nrf_gpio_pin_clear(SPI_SS_PIN);
        nrf_gpio_pin_set(SPI_SS_PIN);
        lvld = false;
    }
    fvld = false;

    for (int i = 0; i < 120; ++i)
    {
        for (int j = 0; j < 245; ++j)
        {
            NRF_LOG_INFO("m_rx_buf[%d][%d] = %x", i, j, m_rx_buf[i][j]);
        }
    }

#if 0
    for (int i = 0; i < 120; ++i)
    {
        nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf[i], m_length);
    }

    for (int i = 0; i < 120; ++i)
    {
        for (int j = 0; j < 245; ++j)
        {
            NRF_LOG_INFO("Camera buf[%d][%d] = %x", i, j, m_rx_buf[i][j]);
        }
    }
#endif

    nrf_gpio_pin_clear(SPI_SS_PIN);
    nrf_gpio_pin_set(SPI_SS_PIN);

    NRF_LOG_INFO("SPI captrue one frame ended.");
}
#endif

#define SPIM_INSTANCE  3                                           /**< SPI instance index. */
static const nrfx_spim_t spim = NRFX_SPIM_INSTANCE(SPIM_INSTANCE);  /**< SPI instance. */

static volatile bool spim_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

// #define TEST_STRING "Nordic123456789012345678901234567890"
// static uint8_t       mm_tx_buf[] = TEST_STRING;           /**< TX buffer. */
// static uint8_t       mm_rx_buf[sizeof(TEST_STRING) + 1];  /**< RX buffer. */
// static const uint8_t mm_length = sizeof(m_tx_buf);        /**< Transfer length. */

static uint8_t       mm_tx_buf[245] = {0xff};           /**< TX buffer. */
static uint8_t       frame_buf[120][245] = {0};  /**< RX buffer. */
static uint8_t       mm_rx_buf[245] = {0};  /**< RX buffer. */
static const uint8_t mm_length = 245;        /**< Transfer length. */


static volatile bool fvld = false;
static volatile bool lvld = false;

#if 0
void spim_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                  p_context)
{
    spim_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    // if (mm_rx_buf[0] != 0)
    // {
    //     NRF_LOG_INFO(" Received:");
    //     // NRF_LOG_HEXDUMP_INFO(mm_rx_buf, strlen((const char *)mm_rx_buf));
    // }
}
#endif

void hm01b0_spim_captrue(void)
{
    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(mm_tx_buf, mm_length, mm_rx_buf, mm_length);

    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_16M;
    spi_config.ss_pin         = NRFX_SPIM_PIN_NOT_USED;
    spi_config.miso_pin       = SPIM_MISO_PIN;
    spi_config.mosi_pin       = NRFX_SPIM_PIN_NOT_USED;
    spi_config.sck_pin        = SPIM_SCK_PIN;
    spi_config.dcx_pin        = NRFX_SPIM_PIN_NOT_USED;
    spi_config.use_hw_ss      = false;
    spi_config.ss_active_high = false;
    APP_ERROR_CHECK(nrfx_spim_init(&spim, &spi_config, NULL, NULL));

    NRF_LOG_INFO("NRFX SPIM example started.");


    for(int k = 0; k < 60; ++k)
    {
        fvld = false;
        while(fvld == true);
        memset(mm_rx_buf, 0, mm_length);
        for (int i = 0; i < 120; ++i)
        {
            lvld = false;
            while(lvld == true);
            APP_ERROR_CHECK(nrfx_spim_xfer_dcx(&spim, &xfer_desc, 0, 15));
            for (int j = 0; j < 245; ++j)
            {
                frame_buf[i][j] =  mm_rx_buf[j];
                NRF_LOG_INFO("mm_buf[%d] = %d", j, mm_rx_buf[j]);

            }
            lvld = false;
        }
        fvld = false;
    }


#if 0//SPI CLK TEST OK
    for (int i = 0; i < 65535; ++i)
    {
        // Reset rx buffer and transfer done flag
        memset(mm_rx_buf, 0, mm_length);
        nrfx_spim_xfer_dcx(&spim, &xfer_desc, 0, 15);
    }
#endif

}

void hm01b0_inputpin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(pin == HM01B0_PIN_VSYNC)
        fvld = true;
    if (pin == HM01B0_PIN_HSYNC)
        lvld = true;
}

static void hm01b0_gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    // nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(HM01B0_PIN_VSYNC, &in_config,
                                            hm01b0_inputpin_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_in_init(HM01B0_PIN_HSYNC, &in_config,
                                            hm01b0_inputpin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(HM01B0_PIN_VSYNC, true);
    nrf_drv_gpiote_in_event_enable(HM01B0_PIN_HSYNC, true);
}

void board_cam_pin_init()
{
    nrf_gpio_cfg_output(SPI_SS_PIN);
    nrf_gpio_cfg_output(SPIM_SS_PIN);
    hm01b0_gpio_init();

#if 0
    nrf_gpio_cfg_input(HM01B0_PIN_FTRIG, NRF_GPIO_PIN_PULLUP);
    nrf_delay_us(1);
    nrf_gpio_cfg_input(HM01B0_PIN_VSYNC, NRF_GPIO_PIN_PULLUP);
    nrf_delay_us(1);
    nrf_gpio_cfg_input(HM01B0_PIN_HSYNC, NRF_GPIO_PIN_PULLUP);
    nrf_delay_us(1);
    nrf_gpio_cfg_input(HM01B0_PIN_PXCLK, NRF_GPIO_PIN_PULLUP);
    nrf_delay_us(1);
#endif
}

void hm01b0_twi_init(void)
{
    NRF_LOG_INFO("twi0_start");
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_hm01b0_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_hm01b0_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
    NRF_LOG_INFO("twi0_end");
}

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

void hm01b0_init_regs(struct senosr_reg *regs_list) {
    NRF_LOG_INFO("hm01b0_init_regs start");
    while (1) {
        uint16_t reg   = regs_list->reg;
        uint8_t  value = regs_list->val;

        if (reg == 0xFFFF && value == 0xFF) {
          break;
    }

    hm01b0_write_reg(reg, value);

    regs_list++;
  }
  NRF_LOG_INFO("hm01b0_init_regs end");
}

void hm01b0_init()
{
	//reset the camera
    board_cam_pin_init();
	//init camera regs
    hm01b0_init_regs(hm01b0_324x244);

}

void unit_twi_test()
{
    uint8_t sensorIDH[1];
    uint8_t sensorIDL[1];
    uint8_t address;

    uint8_t sample_data;
    bool detected_device = false;
    int err_code = 0;

    hm01b0_twi_init();

    NRF_LOG_INFO("TWI detection start");

#if 0
    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
        }
    }
    if (!detected_device)
    {
        NRF_LOG_INFO("No device was found.");
    }

    NRF_LOG_INFO("TWI detection end");

    hm01b0_read_reg(0x0000, sensorIDH, 1);
    hm01b0_read_reg(0x0001, sensorIDL, 1);

    NRF_LOG_INFO("sensorIDL = 0x%x", sensorIDL[0]);
    NRF_LOG_INFO("sensorIDH = 0x%x", sensorIDH[0]);
#endif

    hm01b0_init();
		
    // hm01b0_spi_captrue();
    hm01b0_spim_captrue();

}

void unit_captrue_test()
{

}