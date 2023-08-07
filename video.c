#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

#include "pins.h"
#include "video.h"

#include "ili9341.h"

#include "ov7670reg.h"
#define OV7670_ADDR 0x21 //< Default I2C address if unspecified

#include "ov7670.pio.h"

//Validate pins.h and ov7670.pio agree on fixed GPIO pin assignments
#if PIN_OV7670_HREF != ov7670_pio_href_pin
# error ("Mismatched OV7670 HREF pin")
#endif
#if PIN_OV7670_VSYNC != ov7670_pio_vsync_pin
# error ("Mismatched OV7670 VSYNC pin")
#endif
#if PIN_OV7670_PCLK != ov7670_pio_pclk_pin
# error ("Mismatched OV7670 PCLK pin")
#endif


//OV7670: QVGA: 320x240
#define VIDEO_COLUMNS 320
#define VIDEO_ROWS    240

// using RGB565 encoding (16 bits) - but we only hold 1 row at a time (limited RAM while we wait for SPI writes to the LCD)
// 1 extra value holds the row number that this buffer holds
uint16_t video_buffer[VIDEO_COLUMNS];
uint16_t video_buffer_lcd[VIDEO_COLUMNS];

PIO ov7670_pio;
uint dma_pixel_chan;
i2c_inst_t *ov7670_i2c;
spi_inst_t *lcd_spi;

//====================================================================================================

static void pixel_dma_restart() {
    dma_channel_abort(dma_pixel_chan); //if it was still running
    dma_channel_set_write_addr(dma_pixel_chan, video_buffer, true);
}

static void init_pixel_dma(PIO pio) {
    // We want to use DMA to gather all the pixel data for 1 FRAME from the OV7670
    dma_pixel_chan = dma_claim_unused_channel(true);

    dma_channel_config cfg = dma_channel_get_default_config(dma_pixel_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm_ov7670, false));
    dma_channel_configure(dma_pixel_chan, &cfg,
        video_buffer,           // Destination pointer
        &pio->rxf[sm_ov7670],   // Source pointer
        VIDEO_COLUMNS,          // Number of transfers
        false                   // Start later
    );

    //wait to be triggered
    //dma_channel_start(dma_pixel_chan);
}

//====================================================================================================

static void ov7670_write_reg(uint8_t reg, uint8_t val) {
	i2c_write_blocking(ov7670_i2c, OV7670_ADDR, (uint8_t[]){ reg, val }, 2, false);
}

static uint8_t ov7670_read_reg(uint8_t reg) {
    uint8_t data;
	i2c_write_blocking(ov7670_i2c, OV7670_ADDR, &reg, 1, true);
	i2c_read_blocking(ov7670_i2c, OV7670_ADDR, &data, 1, false);
    return data;
}

static void init_ov7670(i2c_inst_t *i2c) {
    ov7670_i2c = i2c;
    ov7670_write_reg(REG_COM7,  0b10000000); //Reset all registers to defaults
    ov7670_write_reg(REG_COM7,  0b00010100); //QVGA, RGB565
    ov7670_write_reg(REG_COM10, 0b00100100); //No PCLK when no data, vsync on falling edge of pclk
    ov7670_write_reg(REG_COM11, 0b00010000); //enable 50/60HZ detection
    ov7670_write_reg(REG_COM15, 0b00010000); //RGB565
}

//====================================================================================================

/**
 * @brief select the LCD using the CS pin.
 */
static void lcd_cs_select() {
    sleep_us(1);
    gpio_put(PIN_LCD_CS, 0); //active low
    sleep_us(1);
}

/**
 * @brief deselect the LCD using the CS pin.
 */
static void lcd_cs_deselect() {
    sleep_us(1);
    gpio_put(PIN_LCD_CS, 1); //active low
    sleep_us(1);
}

static void lcd_write_command(uint8_t cmd) {
    lcd_cs_select();
    gpio_put(PIN_LCD_DC, 0); // command is active low
    spi_write_blocking(lcd_spi, &cmd, 1);
    gpio_put(PIN_LCD_DC, 1);
    lcd_cs_deselect();
}

static void lcd_write_data(void* data, uint size) {
    lcd_cs_select();
    gpio_put(PIN_LCD_DC, 1);
    spi_write_blocking(lcd_spi, data, size);
    gpio_put(PIN_LCD_DC, 1);
    lcd_cs_deselect();
}

static void lcd_write_byte(uint8_t byt) {
    lcd_write_data(&byt, 1);
}

static void lcd_set_row(uint row) {
    // page address set
    lcd_write_command(ILI9341_PASET);
    lcd_write_byte(0x00);
    lcd_write_byte(row);  // start page
    lcd_write_byte(0x01);
    lcd_write_byte(0x3f);  // end page -> 319

    lcd_write_command(ILI9341_RAMWR);
}

static void init_lcd(spi_inst_t *spi) {
    lcd_spi = spi;

    // ILI9341 ??

    lcd_write_command(0x01);//soft reset
    sleep_ms(100);

    lcd_write_command(ILI9341_GAMMASET);
    lcd_write_byte(0x01);

    // positive gamma correction
    lcd_write_command(ILI9341_GMCTRP1);
    lcd_write_data((uint8_t[15]){ 0x0f, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1, 0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00 }, 15);

    // negative gamma correction
    lcd_write_command(ILI9341_GMCTRN1);
    lcd_write_data((uint8_t[15]){ 0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xc1, 0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f }, 15);

    // memory access control
    lcd_write_command(ILI9341_MADCTL);
    lcd_write_byte(0x48);

    // pixel format
    lcd_write_command(ILI9341_PIXFMT);
    lcd_write_byte(0x55);  // 16-bit

    // frame rate; default, 70 Hz
    lcd_write_command(ILI9341_FRMCTR1);
    lcd_write_byte(0x00);
    lcd_write_byte(0x1B);

    // exit sleep
    lcd_write_command(ILI9341_SLPOUT);

    // display on
    lcd_write_command(ILI9341_DISPON);

    //


    // column address set
    lcd_write_command(ILI9341_CASET);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);  // start column
    lcd_write_byte(0x00);
    lcd_write_byte(0xef);  // end column -> 239

    // page address set
    lcd_write_command(ILI9341_PASET);
    lcd_write_byte(0x00);
    lcd_write_byte(0x00);  // start page
    lcd_write_byte(0x01);
    lcd_write_byte(0x3f);  // end page -> 319

    lcd_write_command(ILI9341_RAMWR);
   
}

//====================================================================================================

void video_init(PIO pio) {
    ov7670_pio = pio;

    // Init OV7670 camera input using I2C and PIO
    // Init SPI TFT LCD as output using SPI


    // OV7670 needs XCLK before it will function. Typical speed 24MHz
    gpio_set_function(PIN_OV7670_XCLK, GPIO_FUNC_PWM);
    uint pwm_slice = pwm_gpio_to_slice_num(PIN_OV7670_XCLK);
    uint pwm_chan = pwm_gpio_to_channel(PIN_OV7670_XCLK);
    //    const float div = (float)clock_get_hz(clk_sys) / (24 * 1000 * 1000);
    const float div = (float)clock_get_hz(clk_sys) / (8 * 1000 * 1000); //DEBUG 8Mhz so logic analyser can see it better
    pwm_set_clkdiv(pwm_slice, div/2);
    // Set period of 2 cycles 
    pwm_set_wrap(pwm_slice, 1);
    pwm_set_chan_level(pwm_slice, pwm_chan, 1); //1 cycle high
    pwm_set_enabled(pwm_slice, true);

    // non-PIO pins that are read by PIO as WAIT GPIO
    gpio_init(PIN_OV7670_HREF);
    gpio_set_dir(PIN_OV7670_HREF, GPIO_IN);
    gpio_init(PIN_OV7670_VSYNC);
    gpio_set_dir(PIN_OV7670_VSYNC, GPIO_IN);
    gpio_init(PIN_OV7670_PCLK);
    gpio_set_dir(PIN_OV7670_PCLK, GPIO_IN);

    // Load OV7670 pixel loader
    ov7670_program_load(pio, PIN_OV7670_PCLK, PIN_OV7670_D0);


    // Map I2C0 to GPIO pins 20 & 21 */
    i2c_inst_t *i2c = i2c0;

    //Initialize I2C port at 100 kHz (required to be called before other functions)
    i2c_init(i2c, 100 * 1000);

    // Assign I2C pins
    gpio_set_function(PIN_OV7670_SIO_D, GPIO_FUNC_I2C);
    gpio_set_function(PIN_OV7670_SIO_C, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_OV7670_SIO_D);
    gpio_pull_up(PIN_OV7670_SIO_C);


    init_ov7670(i2c);
    init_pixel_dma(pio);


    // Map SPI0 to GPIO pins 16..19 */
    spi_inst_t *spi = spi0;

    //Initialize SPI port at 4MHz  (required to be called before other functions)
    spi_init(spi, 4 * 1000 * 1000);

    // Assign SPI hardware pins
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);

    // Assign our manual SPI related pins

    gpio_init(PIN_LCD_DC);
    gpio_set_dir(PIN_LCD_DC, GPIO_OUT);
    gpio_put(PIN_LCD_DC, 0);

    gpio_init(PIN_LCD_CS);
    gpio_set_dir(PIN_LCD_CS, GPIO_OUT);
    gpio_put(PIN_LCD_CS, 0);

    gpio_init(PIN_SDCARD_CS);
    gpio_set_dir(PIN_SDCARD_CS, GPIO_OUT);
    gpio_put(PIN_SDCARD_CS, 0);


    init_lcd(spi);
    //TODO: init_sd_card();
}


// Continuously stream the OV7670 data to the SPI TFT-LCD display
// We add data overlays as we go.
// It does not matter that DMA is overwriting data as we go, as video pixels are mostly the same each frame (a bit of tearing may occur)
void video_stream() {

    // request row 0
    pixel_dma_restart();
    pio_sm_put_blocking(ov7670_pio, sm_ov7670, 0);

    while (true) {

        for (uint y=0; y<VIDEO_ROWS; ++y) {
            //wait for data
            dma_channel_wait_for_finish_blocking(dma_pixel_chan);
            memcpy(video_buffer_lcd, video_buffer, sizeof(video_buffer));

            //set up early for the next transfer we need
            uint next_row = y + 1;
            if (y >= VIDEO_ROWS) { y = 0; }
            pixel_dma_restart();
            pio_sm_put_blocking(ov7670_pio, sm_ov7670, next_row);

            lcd_set_row(y);
            for (uint x=0; x<VIDEO_COLUMNS; ++x) {
                uint16_t rgb565 = video_buffer_lcd[x];
                lcd_write_data(&rgb565, 2); //TODO translate little/big endian?
            }
        }

    }
}
