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


// OV7670: QVGA: 320x240
// Also TFT LCD: 240x320 (will need rotation during output)
#define VIDEO_COLUMNS 320
#define VIDEO_ROWS    240

// Using RGB565 encoding (16 bits).
// But we are low on RAM. So instead of holding a full display here, we hold 2 DMA buffers, 
// which are populated with whole rows from the PIO program.
// Each row starts with 16bits holding the row number, then Columns*16bits of pixel data.
uint16_t video_buffer[2][1 + VIDEO_COLUMNS];
uint16_t video_buffer_lcd[1 + VIDEO_COLUMNS];
_Atomic uint8_t last_video_buf;

PIO ov7670_pio;

uint dma_pixel_chan_0;
uint dma_pixel_chan_1;

i2c_inst_t *ov7670_i2c;
spi_inst_t *lcd_spi;

//====================================================================================================

void pixel_dma_handler() {
    // which DMA channel triggered IRQ 1?
    if (dma_channel_get_irq1_status(dma_pixel_chan_0)) {

        //channel 0
        last_video_buf = 0;
        // clear the interrupt
        dma_channel_acknowledge_irq1(dma_pixel_chan_0);
        // Reset buffer, ready for next time we are triggered/chained
        dma_channel_set_write_addr(dma_pixel_chan_0, video_buffer[0], false);

    } else {

        //channel 1
        last_video_buf = 1;
        // clear the interrupt
        dma_channel_acknowledge_irq1(dma_pixel_chan_1);
        // Reset buffer, ready for next time we are triggered/chained
        dma_channel_set_write_addr(dma_pixel_chan_1, video_buffer[1], false);
    }
}


static void init_pixel_dma(PIO pio) {
    // We want to use DMA to gather all the pixel data in alternate row buffers
    dma_pixel_chan_0 = dma_claim_unused_channel(true);
    dma_pixel_chan_1 = dma_claim_unused_channel(true);

    dma_channel_config cfg0 = dma_channel_get_default_config(dma_pixel_chan_0);
    channel_config_set_transfer_data_size(&cfg0, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg0, false);
    channel_config_set_write_increment(&cfg0, true);
    channel_config_set_dreq(&cfg0, pio_get_dreq(pio, sm_ov7670, false));
    channel_config_set_chain_to(&cfg0, dma_pixel_chan_1); //chain to 1
    dma_channel_configure(dma_pixel_chan_0, &cfg0,
        video_buffer[0],        // Destination pointer
        &pio->rxf[sm_ov7670],   // Source pointer
        1 + VIDEO_COLUMNS,      // Number of transfers
        false                   // Start later
    );

    dma_channel_config cfg1 = dma_channel_get_default_config(dma_pixel_chan_1);
    channel_config_set_transfer_data_size(&cfg1, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg1, false);
    channel_config_set_write_increment(&cfg1, true);
    channel_config_set_dreq(&cfg1, pio_get_dreq(pio, sm_ov7670, false));
    channel_config_set_chain_to(&cfg1, dma_pixel_chan_0); //back to 0
    dma_channel_configure(dma_pixel_chan_1, &cfg1,
        video_buffer[1],        // Destination pointer
        &pio->rxf[sm_ov7670],   // Source pointer
        1 + VIDEO_COLUMNS,      // Number of transfers
        false                   // Start later
    );

    // Tell the DMA to raise IRQ 1 when a channel finishes a block (audio.c uses IRQ 0)
    dma_channel_set_irq1_enabled(dma_pixel_chan_0, true);
    dma_channel_set_irq1_enabled(dma_pixel_chan_1, true);

    irq_set_exclusive_handler(DMA_IRQ_1, pixel_dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);

    //start chain
    dma_channel_start(dma_pixel_chan_0);
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
    while (true) {

        // Grab copy of latest row - then send it

        // Wait for either of the vide IRQ DMA's to trigger (should happen pretty soon at 30fps)
        while ((dma_hw->intr & ((1u << dma_pixel_chan_0) | (1u << dma_pixel_chan_1))) == 0) {
            tight_loop_contents();
        }
        // Then wait for it to be serviced (so we can trust last_video_buf is up-to-date)
        while ((dma_hw->intr & ((1u << dma_pixel_chan_0) | (1u << dma_pixel_chan_1))) != 0) {
            tight_loop_contents();
        }
        // grab
        memcpy(video_buffer_lcd, &video_buffer[last_video_buf], sizeof(video_buffer_lcd));

        //restrict output to this row, then stream bytes to fill it
        //the LCD is rotated (240x320) so we actually fill a 320 high column, 1 row
        uint16_t row = video_buffer_lcd[0];

        // column address set
        lcd_write_command(ILI9341_CASET);
        lcd_write_byte(0x00);
        lcd_write_byte(row);  // start column
        lcd_write_byte(0x00);
        lcd_write_byte(row);  // end column -> 239

        // page address set
        lcd_write_command(ILI9341_PASET);
        lcd_write_byte(0x00);
        lcd_write_byte(0x00);  // start page
        lcd_write_byte(0x01);
        lcd_write_byte(0x3f);  // end page -> 319

        lcd_write_command(ILI9341_RAMWR);

        for (uint x=0; x<VIDEO_COLUMNS; ++x) {
            uint16_t rgb565 = video_buffer_lcd[x+1];
            lcd_write_data(&rgb565, 2); //TODO translate little/big endian?
        }

    }
}
