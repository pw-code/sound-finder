#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "i2s.pio.h"

// GP25
const int LED_PIN = PICO_DEFAULT_LED_PIN;

// Clock and Word-Select shared for all devices
// Then 3x Data pins for addressing 6x I2S microphones (2 channels per 3x pins)
// All INPUT pins (data 0,1,2 & WS) must be consequtive GPIO's for PIO to address.
// GP2 .. GP8
const int I2S_CLK_PIN   = 2;
//skip GP4,GP5 in case we want to use I2C0 (SDA/SCL)
const int I2S_DATA0_PIN = 6;
const int I2S_DATA1_PIN = 7;
const int I2S_DATA2_PIN = 8;
const int I2S_WS_PIN    = 9;

const int TEST_PIN      = 16;


const int AUDIO_SAMPLE_RATE_HZ = 48000; //48kHz (PIO clock divider ends up at 44.2khz, close enough)


int main() {
    bi_decl(bi_program_description("sound-finder waveforming sound locating tool for 6 I2S microphones"));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    bi_decl(bi_1pin_with_name(I2S_CLK_PIN, "I2S Clock"));
    bi_decl(bi_1pin_with_name(I2S_WS_PIN, "I2S Word-Select"));
    bi_decl(bi_1pin_with_name(I2S_DATA0_PIN, "I2S Data 0"));
    bi_decl(bi_1pin_with_name(I2S_DATA1_PIN, "I2S Data 1"));
    bi_decl(bi_1pin_with_name(I2S_DATA2_PIN, "I2S Data 2"));

    stdio_init_all();

    // PIO setup for i2s
    PIO pio = pio0;
    i2s_program_load(pio, AUDIO_SAMPLE_RATE_HZ, I2S_CLK_PIN, I2S_WS_PIN, I2S_DATA0_PIN, I2S_DATA1_PIN, I2S_DATA2_PIN);

    // LED flashing
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);

    for (int i=0; ;++i) {
        gpio_put(LED_PIN, 1);
        gpio_put(TEST_PIN, 1);
        printf("Hello, world! %d\n", i);
        sleep_ms(50);

        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
        gpio_put(TEST_PIN, 0);
        sleep_ms(1000);
    }
    
    return 0;
}