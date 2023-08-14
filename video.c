#include <stdio.h>
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


// OV7670 Initialisation sequence

typedef struct {
  uint8_t reg;   ///< Register address
  uint8_t value; ///< Value to store
} OV7670_command;

static const OV7670_command
    OV7670_init[] = {
        // Initialisation sequences from Adafruit: https://github.com/adafruit/Adafruit_OV7670/blob/master/src/ov7670.c
        {OV7670_REG_COM7, OV7670_COM7_RESET},

        // Manual output format, RGB, use RGB565 and full 0-255 output range
        {OV7670_REG_COM7, OV7670_COM7_SIZE_VGA | OV7670_COM7_RGB},
        {OV7670_REG_RGB444, 0},
        {OV7670_REG_COM15, OV7670_COM15_RGB565 | OV7670_COM15_R10F0},

        {OV7670_REG_TSLB, OV7670_TSLB_YLAST},    // No auto window
        //{OV7670_REG_COM10, OV7670_COM10_VS_NEG}, // -VSYNC (req by SAMD PCC)

        {OV7670_REG_SLOP, 0x20},
        {OV7670_REG_GAM_BASE, 0x1C},
        {OV7670_REG_GAM_BASE + 1, 0x28},
        {OV7670_REG_GAM_BASE + 2, 0x3C},
        {OV7670_REG_GAM_BASE + 3, 0x55},
        {OV7670_REG_GAM_BASE + 4, 0x68},
        {OV7670_REG_GAM_BASE + 5, 0x76},
        {OV7670_REG_GAM_BASE + 6, 0x80},
        {OV7670_REG_GAM_BASE + 7, 0x88},
        {OV7670_REG_GAM_BASE + 8, 0x8F},
        {OV7670_REG_GAM_BASE + 9, 0x96},
        {OV7670_REG_GAM_BASE + 10, 0xA3},
        {OV7670_REG_GAM_BASE + 11, 0xAF},
        {OV7670_REG_GAM_BASE + 12, 0xC4},
        {OV7670_REG_GAM_BASE + 13, 0xD7},
        {OV7670_REG_GAM_BASE + 14, 0xE8},

        {OV7670_REG_COM8, OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP | OV7670_COM8_BANDING},
        {OV7670_REG_GAIN, 0x00},
        {OV7670_COM2_SSLEEP, 0x00},
        {OV7670_REG_COM4, 0x00},
        {OV7670_REG_COM9, 0x20}, // Max AGC value
        {OV7670_REG_BD50MAX, 0x05},
        {OV7670_REG_BD60MAX, 0x07},
        {OV7670_REG_AEW, 0x75},
        {OV7670_REG_AEB, 0x63},
        {OV7670_REG_VPT, 0xA5},
        {OV7670_REG_HAECC1, 0x78},
        {OV7670_REG_HAECC2, 0x68},
        {0xA1, 0x03},              // Reserved register?
        {OV7670_REG_HAECC3, 0xDF}, // Histogram-based AEC/AGC setup
        {OV7670_REG_HAECC4, 0xDF},
        {OV7670_REG_HAECC5, 0xF0},
        {OV7670_REG_HAECC6, 0x90},
        {OV7670_REG_HAECC7, 0x94},
        {OV7670_REG_COM8, OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP |
                              OV7670_COM8_BANDING | OV7670_COM8_AGC |
                              OV7670_COM8_AEC},
        {OV7670_REG_COM5, 0x61},
        {OV7670_REG_COM6, 0x4B},
        {0x16, 0x02},            // Reserved register?
        {OV7670_REG_MVFP, 0x07}, // 0x07,
        {OV7670_REG_ADCCTR1, 0x02},
        {OV7670_REG_ADCCTR2, 0x91},
        {0x29, 0x07}, // Reserved register?
        {OV7670_REG_CHLF, 0x0B},
        {0x35, 0x0B}, // Reserved register?
        {OV7670_REG_ADC, 0x1D},
        {OV7670_REG_ACOM, 0x71},
        {OV7670_REG_OFON, 0x2A},
        {OV7670_REG_COM12, 0x78},
        {0x4D, 0x40}, // Reserved register?
        {0x4E, 0x20}, // Reserved register?
        {OV7670_REG_GFIX, 0x69}, //was 0x5D
        {OV7670_REG_REG74, 0x19},
        {0x8D, 0x4F}, // Reserved register?
        {0x8E, 0x00}, // Reserved register?
        {0x8F, 0x00}, // Reserved register?
        {0x90, 0x00}, // Reserved register?
        {0x91, 0x00}, // Reserved register?
        {OV7670_REG_DM_LNL, 0x00},
        {0x96, 0x00}, // Reserved register?
        {0x9A, 0x80}, // Reserved register?
        {0xB0, 0x84}, // Reserved register?
        {OV7670_REG_ABLC1, 0x0C},
        {0xB2, 0x0E}, // Reserved register?
        {OV7670_REG_THL_ST, 0x82},
        {0xB8, 0x0A}, // Reserved register?
        {OV7670_REG_AWBC1, 0x14},
        {OV7670_REG_AWBC2, 0xF0},
        {OV7670_REG_AWBC3, 0x34},
        {OV7670_REG_AWBC4, 0x58},
        {OV7670_REG_AWBC5, 0x28},
        {OV7670_REG_AWBC6, 0x3A},
        {0x59, 0x88}, // Reserved register?
        {0x5A, 0x88}, // Reserved register?
        {0x5B, 0x44}, // Reserved register?
        {0x5C, 0x67}, // Reserved register?
        {0x5D, 0x49}, // Reserved register?
        {0x5E, 0x0E}, // Reserved register?
        {OV7670_REG_LCC3, 0x04},
        {OV7670_REG_LCC4, 0x20},
        {OV7670_REG_LCC5, 0x05},
        {OV7670_REG_LCC6, 0x04},
        {OV7670_REG_LCC7, 0x08},
        {OV7670_REG_AWBCTR3, 0x0A},
        {OV7670_REG_AWBCTR2, 0x55},
        {OV7670_REG_MTX1, 0x80},
        {OV7670_REG_MTX2, 0x80},
        {OV7670_REG_MTX3, 0x00},
        {OV7670_REG_MTX4, 0x22},
        {OV7670_REG_MTX5, 0x5E},
        {OV7670_REG_MTX6, 0x80}, // 0x40?
        {OV7670_REG_AWBCTR1, 0x11},
        {OV7670_REG_AWBCTR0, 0x9E}, // Or use 0x9E for advance AWB
        {OV7670_REG_BRIGHT, 0x40}, //was 0x00
        {OV7670_REG_CONTRAS, 0x40},
        {OV7670_REG_CONTRAS_CENTER, 0x80}, // 0x40?


        //Switch to QVGA RGB565
        //From https://github.com/ComputerNerd/ov7670-simple/blob/master/main.c
    	{OV7670_REG_COM3,4},	// REG_COM3 
        {OV7670_REG_COM14, 0x19},
		{0x72, 0x11},
		{0x73, 0xf1},
		{OV7670_REG_HSTART, 0x15},
		{OV7670_REG_HSTOP,  0x01},
		{OV7670_REG_HREF,   0x24},		
		{OV7670_REG_VSTART, 0x02},
		{OV7670_REG_VSTOP,  0x7a},
		{OV7670_REG_VREF,   0x0a},

        // Colour balance
        {OV7670_REG_GAIN, 0x00},
        {OV7670_REG_RED, 0x60},
        {OV7670_REG_BLUE, 0xF0},
        {OV7670_REG_GGAIN, 0x80},

        {OV7670_REG_EDGE, 0x02}, //Edge enhancement factor 

        {OV7670_REG_COM11, 0xFF}, //night mode all options on : automatic

#undef OV7670_SHOW_TEST_PATTERN
#ifdef OV7670_SHOW_TEST_PATTERN
        // Colour bars
        // {OV7670_REG_SCALING_XSC, 0x3A}, // Enable pattern
        // {OV7670_REG_SCALING_YSC, 0x35 | 0x80}, // Enable pattern
        {OV7670_REG_SCALING_XSC, 0x00}, // Enable pattern
        {OV7670_REG_SCALING_YSC, 0x80}, // Enable pattern
        // Shifting 1
        // {OV7670_REG_SCALING_XSC, 0x80}, // Enable pattern
        // {OV7670_REG_SCALING_YSC, 0x00}, // Enable pattern
        // Fade to grey
        // {OV7670_REG_SCALING_XSC, 0x80}, // Enable pattern
        // {OV7670_REG_SCALING_YSC, 0x80}, // Enable pattern
#else
        {OV7670_REG_SCALING_XSC, 0},
        {OV7670_REG_SCALING_YSC, 0},
#endif

        //{OV7670_REG_CLKRC, 0b10000000}, //double clock
        //half and then double the pclk. This seems to tidy up the data quality
        {OV7670_REG_CLKRC, 0x1},
        {OV7670_REG_DBLV, 0x4A},

        {0xFF, 0xFF},       // End-of-data marker
};

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
        &pio->rxf[sm_ov7670],   // Source pointer, lower 16bits of the RX FIFO
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
        &pio->rxf[sm_ov7670],   // Source pointer, lower 16bits of the RX FIFO
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
	i2c_write_blocking(ov7670_i2c, OV7670_ADDR, &reg, 1, false);
	i2c_read_blocking(ov7670_i2c, OV7670_ADDR, &data, 1, false);
    return data;
}

static void init_ov7670(i2c_inst_t *i2c) {
    ov7670_i2c = i2c;
    const OV7670_command *cmd = OV7670_init;
    while (cmd->reg != 0xFF) {
        ov7670_write_reg(cmd->reg, cmd->value);
        cmd++;
    }
}

//====================================================================================================

/**
 * @brief select the LCD using the CS pin.
 */
static void lcd_cs_select() {
    gpio_put(PIN_LCD_CS, 0); //active low
}

/**
 * @brief deselect the LCD using the CS pin.
 */
static void lcd_cs_deselect() {
    gpio_put(PIN_LCD_CS, 1); //active low
}

static void lcd_write_commandX(uint8_t cmd, size_t num_data_bytes, uint8_t* data) {
    lcd_cs_select();
    gpio_put(PIN_LCD_DC, 0); // command is active low
    spi_write_blocking(lcd_spi, &cmd, 1);
    gpio_put(PIN_LCD_DC, 1);
    if (num_data_bytes != 0) {
        spi_write_blocking(lcd_spi, data, num_data_bytes);
    }
    lcd_cs_deselect();
}

static void lcd_write_command1(uint8_t cmd, uint8_t data) {
    lcd_write_commandX(cmd, 1, &data);
}

static void lcd_write_command0(uint8_t cmd) {
    lcd_write_commandX(cmd, 0, NULL);
}


static void init_lcd(spi_inst_t *spi) {
    lcd_spi = spi;

    // ILI9341 ??

    lcd_write_command0(ILI9341_SWRESET);
    sleep_ms(100);

    lcd_write_command1(ILI9341_GAMMASET, 0x01);

    // positive gamma correction
    lcd_write_commandX(ILI9341_GMCTRP1, 15, (uint8_t[15]){ 0x0f, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1, 0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00 });

    // negative gamma correction
    lcd_write_commandX(ILI9341_GMCTRN1, 15, (uint8_t[15]){ 0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xc1, 0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f });

    // memory access control
    //lcd_write_command1(ILI9341_MADCTL, 0x48);
    //lcd_write_command1(ILI9341_MADCTL, 0x40); //top-to-bottom, RGB
    lcd_write_command1(ILI9341_MADCTL, 0b11000000); //top-to-bottom, right-to-left, RGB

    // pixel format
    lcd_write_command1(ILI9341_PIXFMT, 0x55);  // 16-bits per pixel LCD & SPI

    // frame rate; default, 70 Hz
    lcd_write_commandX(ILI9341_FRMCTR1, 2, (uint8_t[2]){ 0x00, 0x1B });

    // exit sleep
    lcd_write_command0(ILI9341_SLPOUT);
    sleep_ms(120);

    // display on
    lcd_write_command0(ILI9341_DISPON);

    //

    // column address set
    lcd_write_commandX(ILI9341_CASET, 4, (uint8_t[4]){
        0x00, 0x00,    // start column
        0x00, 0xef});  // end column -> 239

    // page address set
    lcd_write_commandX(ILI9341_PASET, 4, (uint8_t[4]){
        0x00, 0x00,    // start page
        0x01, 0x3f});  // end page -> 319

    lcd_write_command0(ILI9341_RAMWR);
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
    //const float div = (float)clock_get_hz(clk_sys) / (24 * 1000 * 1000);
    const float div = (float)clock_get_hz(clk_sys) / (12 * 1000 * 1000);
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
    ov7670_program_load(pio, PIN_OV7670_VSYNC, PIN_OV7670_D0);

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
//    spi_init(spi, 4 * 1000 * 1000);
//    spi_init(spi, 10 * 1000 * 1000); //LCD can handle 40Mhz?!
    spi_init(spi, 65 * 1000 * 1000); //LCD can handle 65Mhz?! 

    //spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

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

static void lcd_reg_diag(const char * name, uint8_t cmd, uint8_t num_returned_params) {
    uint8_t bytes[num_returned_params];

    printf("%02x %s = ", cmd, name);

    lcd_cs_select();
    gpio_put(PIN_LCD_DC, 0); // command is active low
    spi_write_blocking(lcd_spi, &cmd, 1);
    spi_read_blocking(lcd_spi, 0xff, bytes, num_returned_params);
    gpio_put(PIN_LCD_DC, 1);
    lcd_cs_deselect();

    //skip param 1 - it is always junk
    for (uint i = 1; i < num_returned_params; ++i) {
        printf(" %02x", bytes[i]);
    }
    printf("\n");
}

// Continuously stream the OV7670 data to the SPI TFT-LCD display
// We add data overlays as we go.
// It does not matter that DMA is overwriting data as we go, as video pixels are mostly the same each frame (a bit of tearing may occur)
void video_stream() {

    lcd_reg_diag("ILI9341_RDDID", 0x04, 4);
    lcd_reg_diag("ILI9341_RDDST", 0x09, 5);
    lcd_reg_diag("ILI9341_RDDPM", 0x0A, 2);
    lcd_reg_diag("ILI9341_RDDMADCTL", 0x0B, 2);
    lcd_reg_diag("ILI9341_RDDCOLMOD", 0x0C, 2);
    lcd_reg_diag("ILI9341_RDDIM", 0x0D, 2);
    lcd_reg_diag("ILI9341_RDDSM", 0x0E, 2);
    lcd_reg_diag("ILI9341_RDDSDR", 0x0F, 2);
    lcd_reg_diag("ILI9341_RDDSDR", 0x0F, 2);

    // lcd_reg_diag("ILI9488_RDSELFDIAG", 0x0F, 1);
    // lcd_reg_diag("ILI9488_DFUNCTR", 0xb6, 5);
    // lcd_reg_diag("ILI9488_PWCTR1", 0xC0, 3);
    // lcd_reg_diag("ILI9488_VMCTR1", 0xC5, 3);
    // lcd_reg_diag("ILI9488_VMCTR2", 0xC7, 2);
    // lcd_reg_diag("NVM Stats    ", 0xD2, 3);
    // lcd_reg_diag("ID4          ", 0xD3, 4);
    // lcd_reg_diag("ILI9488_RDID1", 0xDA, 2);
    // lcd_reg_diag("ILI9488_RDID2", 0xDB, 2);
    // lcd_reg_diag("ILI9488_RDID3", 0xDC, 2);
    // lcd_reg_diag("GAMMAP       ", 0xE0, 16);
    // lcd_reg_diag("GAMMAN       ", 0xE1, 16);
    // lcd_reg_diag("INTERFACE    ", 0xf6, 4);

    printf("OV7670 PID %02x\n", ov7670_read_reg(OV7670_REG_PID));
    printf("OV7670 VER %02x\n", ov7670_read_reg(OV7670_REG_VER));

    uint8_t colour[2];
    // colour=0x0000; //WHITE?
    // colour=0xFFFF; //BLACK?

    //RED
    colour[0]=0xF8; colour[1]=0x00; 
    lcd_write_command0(ILI9341_RAMWR);
    lcd_cs_select();
    for(uint i=0; i<(320*240); ++i) {
        spi_write_blocking(lcd_spi, colour, 2);
    }
    lcd_cs_deselect();
    sleep_ms(20);

    //GREEN
    colour[0]=0x07; colour[1]=0xE0; 
    lcd_write_command0(ILI9341_RAMWR);
    lcd_cs_select();
    for(uint i=0; i<(320*240); ++i) {
        spi_write_blocking(lcd_spi, colour, 2);
    }
    lcd_cs_deselect();
    sleep_ms(20);

    //BLUE
    colour[0]=0x00; colour[1]=0x3F; 
    lcd_write_command0(ILI9341_RAMWR);
    lcd_cs_select();
    for(uint i=0; i<(320*240); ++i) {
        spi_write_blocking(lcd_spi, colour, 2);
    }
    lcd_cs_deselect();
    sleep_ms(20);

    // while (getchar() == EOF) {
    //     tight_loop_contents();
    // }
    // printf("RUN\n");

    // while (true) {
    //     printf("%d x%x\tx%x\n", last_video_buf, video_buffer[0][0], video_buffer[1][0]);
    //     sleep_ms(91);
    // }

    while (true) {

        // Grab copy of latest row - then send it

        // Wait for either of the video DMA IRQs to trigger (should happen pretty soon at 30fps)
        while ((dma_hw->intr & ((1u << dma_pixel_chan_0) | (1u << dma_pixel_chan_1))) == 0) {
            tight_loop_contents();
        }
        // Then wait for it to be serviced (so we can trust last_video_buf is up-to-date)
        while ((dma_hw->intr & ((1u << dma_pixel_chan_0) | (1u << dma_pixel_chan_1))) != 0) {
            tight_loop_contents();
        }
        // grab a copy -- because the SPI transfer is slower and so the DMA buffer could get overwritten while we write
#define SWAP_VIDEO_DATA_ENDIANNESS
#ifdef SWAP_VIDEO_DATA_ENDIANNESS
        video_buffer_lcd[0] = video_buffer[last_video_buf][0]; //row
        for (uint i=1; i<=VIDEO_COLUMNS; ++i) {
            uint16_t tmp = video_buffer[last_video_buf][i];
            video_buffer_lcd[i] = (tmp >> 8) | (tmp << 8);
        }
#else
        memcpy(video_buffer_lcd, video_buffer[last_video_buf], sizeof(video_buffer_lcd));
#endif

        //restrict output to this row, then stream bytes to fill it
        //the LCD is rotated (240x320) so we actually fill a 320 high column, 1 row
        uint16_t row = video_buffer_lcd[0];
        //DEBUG
        //printf("ROW %u\n", row);
        // {//if (row == 1) {
        //     printf("%d %u:%02x\t", last_video_buf, row, row);
        //     for (uint i=1; i<VIDEO_COLUMNS+1; ++i) {
        //         printf(" %04x", video_buffer_lcd[i]);
        //     }
        //     printf("\n");
        //     sleep_ms(791);
        // }

        // column address set
        lcd_write_commandX(ILI9341_CASET, 4, (uint8_t[4]){
            (row >> 8) & 0xFF, row & 0xFF,    // start column
            (row >> 8) & 0xFF, row & 0xFF});  // end column

        // page address set
        lcd_write_commandX(ILI9341_PASET, 4, (uint8_t[4]){
            0x00, 0x00,    // start page
            0x01, 0x3f});  // end page -> 319

        // pixel data
        lcd_write_commandX(ILI9341_RAMWR, 2*VIDEO_COLUMNS, (uint8_t*)(&video_buffer_lcd[1]));
    }
}
