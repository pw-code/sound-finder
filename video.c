#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"

#include "pins.h"
#include "video.h"
#include "audio.h" //for capture analysis data

#include "ili9341.h"
#include "ov7670.h"


// Using RGB565 encoding (16 bits).
// But we are low on RAM. So instead of holding a full display here, we hold 2 DMA buffers, 
// which are populated with whole rows from the PIO program.
// Each row starts with 16bits holding the row number, then Columns*16bits of pixel data.
uint16_t video_buffer[2][1 + VIDEO_COLUMNS];
_Atomic uint8_t last_video_buf;

// Translated pixels, with out overlays included, for LCD output
uint16_t video_buffer_lcd[VIDEO_COLUMNS];


//====================================================================================================

void video_init(PIO pio) {

    // Map I2C0 to GPIO pins 20 & 21 */
    i2c_inst_t *i2c = i2c0;

    //Initialize I2C port at 100 kHz (required to be called before other functions)
    i2c_init(i2c, 100 * 1000);

    // Assign I2C pins
    gpio_set_function(PIN_OV7670_SIO_D, GPIO_FUNC_I2C);
    gpio_set_function(PIN_OV7670_SIO_C, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_OV7670_SIO_D);
    gpio_pull_up(PIN_OV7670_SIO_C);


    ov7670_init(pio, i2c);
    ov7670_diag();


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


    lcd_init(spi);
    lcd_diag();

    //TODO: init_sd_card();
}


static void plot_audio_marker(int row, int marker_x, int marker_y) {

    // Draw a square around the given spot
    static const int square_size = 15;

    if (row >= (marker_y - square_size) && row < (marker_y + square_size)) {
        int minx = marker_x - square_size;
        if (minx < 0) { minx = 0; }
        int maxx = marker_x + square_size;
        if (maxx > VIDEO_COLUMNS) { maxx = VIDEO_COLUMNS; }
        for (int x = minx; x < maxx; ++x) {
            //replace RED (endianness means it's in the middle 5 bits)
            // rrrrrggggggbbbbb =>  gggbbbbbrrrrrggg
            //                    0b1110011100000001 
            //also remove the top 2 bits of G & B too (fade away)
            video_buffer_lcd[x] = (video_buffer_lcd[x] & 0b1110011100000001) | 0b0000000011111000;
        }
    }
}

static void plot_audio_markers(int row) {
    //TODO: use data from audio module sound intensity data
    // extern uint32_t capture_magnitudes[SAMPLE_OFFSET_WIDTH][SAMPLE_OFFSET_DEPTH][SAMPLE_OFFSET_HEIGHT];
    // extern uint32_t screen_offsets[SAMPLE_OFFSET_WIDTH][SAMPLE_OFFSET_DEPTH][SAMPLE_OFFSET_HEIGHT];
    for (uint w = 0; w < SAMPLE_OFFSET_WIDTH; ++w) {
        for (uint d = 0; d < SAMPLE_OFFSET_DEPTH; ++d) {
            for (uint h = 0; h < SAMPLE_OFFSET_HEIGHT; ++h) {
                int x = w<<5; //screen_offsets[w][d][h].x;
                int y = h<<6; //screen_offsets[w][d][h].y;
                plot_audio_marker(row, x, y);
            }
        }
    }
    // plot_audio_marker(row, 160, 100);
    // plot_audio_marker(row, 315, 10);
    // plot_audio_marker(row, 5, 235);
}

// Continuously stream the OV7670 data to the SPI TFT-LCD display
// We add data overlays as we go.
// It does not matter that DMA is overwriting data as we go, as video pixels are mostly the same each frame (a bit of tearing may occur)
void video_stream() {
    while (true) {
        // Grab copy of latest row - then send it

        ov7670_wait_for_buffer();

        // grab a copy -- because the SPI transfer is slower and so the DMA buffer could get overwritten while we write
        // swap data endianness as we go (PICO and OV7670 are opposite)
        uint16_t row = video_buffer[last_video_buf][0];
        for (uint i=0; i<VIDEO_COLUMNS; ++i) {
            uint16_t tmp = video_buffer[last_video_buf][i+1];
            video_buffer_lcd[i] = (tmp >> 8) | (tmp << 8);
        }

        // Overlay our audio information on the signal
        plot_audio_markers(row);

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

        // draw async (dma). this allows us to process a new row while the last is writing
        lcd_draw_row(true, row, video_buffer_lcd);
    }
}
