#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/pio.h"

#include "audio.h"
#include "offsets.h"
#include "pins.h"



extern void core1_main(); //for core1 to run on 

int main() {
    bi_decl(bi_program_description("sound-finder waveforming sound locating tool for 6 I2S microphones"));
    bi_decl(bi_1pin_with_name(PIN_LED, "On-board LED"));
    bi_decl(bi_1pin_with_name(PIN_I2S_CLK, "I2S Clock"));
    bi_decl(bi_1pin_with_name(PIN_I2S_WS, "I2S Word-Select"));
    bi_decl(bi_1pin_with_name(PIN_I2S_DATA0, "I2S Data 0"));
    bi_decl(bi_1pin_with_name(PIN_I2S_DATA1, "I2S Data 1"));
    bi_decl(bi_1pin_with_name(PIN_I2S_DATA2, "I2S Data 2"));

    stdio_init_all();


    // PIO 0 setup for i2s capture
    audio_dma_init(pio0);

    // PIO 1 setup for OV7670 video capture
//    camera_init(pio1);

    //trigger analysis on the second core
//    multicore_launch_core1(core1_main);

    // LED flashing
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);


    while(true) {
        audio_capture_analyse();
    }
    
    return 0;
}