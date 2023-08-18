#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/spi.h"

#include "pins.h"
#include "video.h"

#include "ili9341.h"
#include "ili9341reg.h"


spi_inst_t *lcd_spi;

uint lcd_tx_dma_chan;


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


/**
 * @brief Init SPI TFT LCD as output using SPI
 */
void lcd_init(spi_inst_t *spi) {
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


    // Setup for SPI DMA
    lcd_tx_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(lcd_tx_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(lcd_spi, true));
    dma_channel_configure(lcd_tx_dma_chan, &c,
                          &spi_get_hw(lcd_spi)->dr, // write address
                          NULL,                     // read address - populated when drawing starts
                          VIDEO_COLUMNS*2,          // 2x because 2 bytes per pixel
                          false);                   // don't start yet
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

void lcd_diag() {
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
}


void lcd_draw_row(bool async, uint16_t row, uint16_t* buffer) {
    //restrict output to this row, then stream bytes to fill it
    //the LCD is rotated (240x320) so we actually fill a 320 row high single column

    // column address set
    lcd_write_commandX(ILI9341_CASET, 4, (uint8_t[4]){
        (row >> 8) & 0xFF, row & 0xFF,    // start column
        (row >> 8) & 0xFF, row & 0xFF});  // end column

    // page address set
    lcd_write_commandX(ILI9341_PASET, 4, (uint8_t[4]){
        0x00, 0x00,    // start page
        0x01, 0x3f});  // end page -> 319

    // pixel data (16 bit buffer as 8bit bytes)
    if (async) {
        lcd_write_commandX(ILI9341_RAMWR, 2*VIDEO_COLUMNS, (uint8_t*)buffer);
    } else {
        dma_channel_set_read_addr(lcd_tx_dma_chan, (uint8_t*)buffer, true); // set buffer & trigger
    }
}

bool lcd_is_busy() {
    return dma_channel_is_busy(lcd_tx_dma_chan) || spi_is_busy(lcd_spi);
}
