#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/dma.h"

#include "offsets.h"

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


#define AUDIO_SAMPLE_RATE_HZ 30000     /* With nyquist limit this should give is up to 15khz sounds. Good enough with too much data. */

#if AUDIO_SAMPLE_RATE_HZ != SAMPLE_OFFSET_HZ
# error ("Mismatched sample rates " AUDIO_SAMPLE_RATE_NZ " != " SAMPLE_OFFSET_HZ)
#endif

// Audio buffers. These must fit in ram (only 260kb total)
#define AUDIO_SAMPLE_MILLISECONDS 250   /* 250ms per capture */
#define AUDIO_CHANNEL_BUF_SIZE (AUDIO_SAMPLE_RATE_HZ * 2 * AUDIO_SAMPLE_MILLISECONDS) / 1000

uint32_t capture_buf_data0[AUDIO_CHANNEL_BUF_SIZE];
uint32_t capture_buf_data1[AUDIO_CHANNEL_BUF_SIZE];
uint32_t capture_buf_data2[AUDIO_CHANNEL_BUF_SIZE];

extern void analyse_last_capture(uint capture_length, uint32_t* capture_buf_data0, uint32_t* capture_buf_data1, uint32_t* capture_buf_data2);

uint handler_dma_channel, dma_chan0, dma_chan1, dma_chan2;

volatile uint32_t capture_count = 0;
volatile bool capture_ready = false;

void i2s_dma_handler() {
    capture_count++;
    capture_ready = true;

    // clear the interrupt
    dma_hw->ints0 = 1u << handler_dma_channel;

    // All DMA channels should have finished at once (the PIO's are synchronised)
    // We we can and need to re-trigger all 3

    // Give the channel a new buffer pointer, and re-trigger it
    dma_channel_set_write_addr(dma_chan0, capture_buf_data0, true);
    dma_channel_set_write_addr(dma_chan1, capture_buf_data1, true);
    dma_channel_set_write_addr(dma_chan2, capture_buf_data2, true);
}


static void i2s_dma_setup_sm(PIO pio, uint sm, uint32_t* capture_buf, bool add_interrupt) {

    uint dma_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);

    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
        capture_buf,            // Destination pointer
        &pio->rxf[sm],          // Source pointer
        AUDIO_CHANNEL_BUF_SIZE, // Number of transfers
        true                    // Start immediately
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block?
    if (add_interrupt) {
        dma_channel_set_irq0_enabled(dma_chan, true);

        irq_set_exclusive_handler(DMA_IRQ_0, i2s_dma_handler);
        irq_set_enabled(DMA_IRQ_0, true);

        handler_dma_channel = dma_chan;
    }
}

static void i2s_dma_setup(PIO pio) {
    // 1 DMA per buffer / PIO state machine
    i2s_dma_setup_sm(pio, sm_data0,  capture_buf_data0, true);
    i2s_dma_setup_sm(pio, sm_data1,  capture_buf_data1, false);
    i2s_dma_setup_sm(pio, sm_data2,  capture_buf_data2, false);
}


extern void core1_main(); //for core1 to run on 

int main() {
    bi_decl(bi_program_description("sound-finder waveforming sound locating tool for 6 I2S microphones"));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    bi_decl(bi_1pin_with_name(I2S_CLK_PIN, "I2S Clock"));
    bi_decl(bi_1pin_with_name(I2S_WS_PIN, "I2S Word-Select"));
    bi_decl(bi_1pin_with_name(I2S_DATA0_PIN, "I2S Data 0"));
    bi_decl(bi_1pin_with_name(I2S_DATA1_PIN, "I2S Data 1"));
    bi_decl(bi_1pin_with_name(I2S_DATA2_PIN, "I2S Data 2"));

    stdio_init_all();


    // PIO setup for i2s capture
    PIO pio = pio0;
    i2s_program_load(pio, AUDIO_SAMPLE_RATE_HZ, I2S_CLK_PIN, I2S_WS_PIN, I2S_DATA0_PIN, I2S_DATA1_PIN, I2S_DATA2_PIN);

    // DMA Setup
    i2s_dma_setup(pio);

    // Start PIO (**at the same time** and in sync)
    pio_enable_sm_mask_in_sync(pio, 0x0f); //all 4 at once!!

    //trigger analysis on the second core
//    multicore_launch_core1(core1_main);

    // LED flashing
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for (int i=0; ;++i) {
        gpio_put(LED_PIN, 1);
        printf("Hello, world! %d, cnt %d, bufs %ld\n", i, capture_count, sizeof(capture_buf_data0)*3);
        sleep_ms(50);

        gpio_put(LED_PIN, 0);
        sleep_ms(2000);

        if (capture_ready) {
            analyse_last_capture(AUDIO_CHANNEL_BUF_SIZE, capture_buf_data0, capture_buf_data1, capture_buf_data2);
        }
    }
    
    return 0;
}