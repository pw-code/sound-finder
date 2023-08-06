#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/pio.h"

#include "audio.h"
#include "offsets.h"
#include "pins.h"
#include "video.h"



int main() {
    bi_decl(bi_program_description("sound-finder waveforming sound locating tool for 6 I2S microphones"));
    bi_decl(bi_1pin_with_name(PIN_LED, "On-board LED"));

    bi_decl(bi_1pin_with_name(PIN_I2S_CLK, "I2S Clock"));
    bi_decl(bi_1pin_with_name(PIN_I2S_WS, "I2S Word-Select"));
    bi_decl(bi_1pin_with_name(PIN_I2S_DATA0, "I2S Data 0"));
    bi_decl(bi_1pin_with_name(PIN_I2S_DATA1, "I2S Data 1"));
    bi_decl(bi_1pin_with_name(PIN_I2S_DATA2, "I2S Data 2"));

    bi_decl(bi_1pin_with_name(PIN_OV7670_SIO_D, "OV7670 SIO_D"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_SIO_C, "OV7670 SIO_C"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_XCLK, "OV7670 XCLK"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_PCLK, "OV7670 PCLK"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_VSYNC, "OV7670 VSYNC"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_HREF, "OV7670 HREF"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_D0, "OV7670 D0"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_D1, "OV7670 D1"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_D2, "OV7670 D2"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_D3, "OV7670 D3"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_D4, "OV7670 D4"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_D5, "OV7670 D5"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_D6, "OV7670 D6"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_D7, "OV7670 D7"));
    bi_decl(bi_1pin_with_name(PIN_OV7670_D8, "OV7670 D8"));

    bi_decl(bi_1pin_with_name(PIN_SPI_SCK,  "LCD SCK"));
    bi_decl(bi_1pin_with_name(PIN_SPI_MOSI, "LCD MOSI"));
    bi_decl(bi_1pin_with_name(PIN_SPI_MISO, "LCD MISO"));
    bi_decl(bi_1pin_with_name(PIN_LCD_CS,   "LCD SPI CS"));
    bi_decl(bi_1pin_with_name(PIN_LCD_DC,   "LCD DS"));
    bi_decl(bi_1pin_with_name(PIN_SDCARD_CS, "SD Card SPI CS"));


    //stdio_init_all();
    stdio_usb_init(); //use use UART pins for other tasks, so we can only log via USB


    // PIO 0 setup for i2s capture
    audio_dma_init(pio0);

    // PIO 1 setup for OV7670 video capture
    video_init(pio1);
    //trigger video streaming on the second core
    multicore_launch_core1(video_stream);


    // LED flashing
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    // main loop is just audio capture and analysis
    while(true) {
        audio_capture_analyse();
    }
    
    return 0;
}
