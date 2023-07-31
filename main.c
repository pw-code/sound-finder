#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
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
const int I2S_WS_PIN    = 3;
//skip GP4,GP5 in case we want to use I2C0 (SDA/SCL)
const int I2S_DATA0_PIN = 6;
const int I2S_DATA1_PIN = 7;
const int I2S_DATA2_PIN = 8;

const int TEST_PIN      = 11;


#define AUDIO_SAMPLE_RATE_HZ 44100

#if AUDIO_SAMPLE_RATE_HZ != SAMPLE_OFFSET_HZ
# error ("Mismatched sample rates " AUDIO_SAMPLE_RATE_NZ " != " SAMPLE_OFFSET_HZ)
#endif

// Audio buffers. These must fit in ram (only 260kb total)
#define AUDIO_SAMPLE_MILLISECONDS 100   /* ms per capture */
#define AUDIO_CHANNEL_BUF_SIZE (AUDIO_SAMPLE_RATE_HZ * 2 * AUDIO_SAMPLE_MILLISECONDS) / 1000

//double-buffer between DMA samples
uint32_t capture_buf_data0[2][AUDIO_CHANNEL_BUF_SIZE];
uint32_t capture_buf_data1[2][AUDIO_CHANNEL_BUF_SIZE];
uint32_t capture_buf_data2[2][AUDIO_CHANNEL_BUF_SIZE];

extern void analyse_last_capture(uint capture_length, uint32_t* capture_buf_data0, uint32_t* capture_buf_data1, uint32_t* capture_buf_data2);

uint handler_dma_channel_0, handler_dma_channel_1;
uint dma_chan0_0, dma_chan0_1;
uint dma_chan1_0, dma_chan1_1;
uint dma_chan2_0, dma_chan2_1;

volatile uint32_t capture_count = 0;

void i2s_dma_handler_0() {
    capture_count++;

    // clear the interrupt
    dma_hw->ints0 = 1u << handler_dma_channel_0;

    // All DMA channels should have finished at once (the PIO's are synchronised)

    // Reset buffer, ready for next time we are triggered/chained
    dma_channel_set_write_addr(dma_chan0_0, capture_buf_data0[0], false);
    dma_channel_set_write_addr(dma_chan1_0, capture_buf_data1[0], false);
    dma_channel_set_write_addr(dma_chan2_0, capture_buf_data2[0], false);
}

void i2s_dma_handler_1() {
    capture_count++;

    // clear the interrupt
    dma_hw->ints0 = 1u << handler_dma_channel_1;

    // All DMA channels should have finished at once (the PIO's are synchronised)

    // Reset buffer, ready for next time we are triggered/chained
    dma_channel_set_write_addr(dma_chan0_1, capture_buf_data0[1], false);
    dma_channel_set_write_addr(dma_chan1_1, capture_buf_data1[1], false);
    dma_channel_set_write_addr(dma_chan2_1, capture_buf_data2[1], false);
}


static void i2s_dma_setup_sm(PIO pio, uint sm, uint32_t *capture_buf0, uint32_t *capture_buf1, uint* pChan0, uint* pChan1, bool add_interrupt) {

    // Two alternative DMA transfers, with different buffers (reloaded in interrupt handler)

    *pChan0 = dma_claim_unused_channel(true);
    *pChan1 = dma_claim_unused_channel(true);

    dma_channel_config c0 = dma_channel_get_default_config(*pChan0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
    channel_config_set_read_increment(&c0, false);
    channel_config_set_write_increment(&c0, true);
    channel_config_set_dreq(&c0, pio_get_dreq(pio, sm, false));
    channel_config_set_chain_to(&c0, *pChan1);
    dma_channel_configure(*pChan0, &c0,
        capture_buf0,           // Destination pointer
        &pio->rxf[sm],          // Source pointer
        AUDIO_CHANNEL_BUF_SIZE, // Number of transfers
        false                   // Start later
    );

    dma_channel_config c1 = dma_channel_get_default_config(*pChan1);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, false);
    channel_config_set_write_increment(&c1, true);
    channel_config_set_dreq(&c1, pio_get_dreq(pio, sm, false));
    channel_config_set_chain_to(&c1, *pChan0);
    dma_channel_configure(*pChan1, &c1,
        capture_buf1,           // Destination pointer
        &pio->rxf[sm],          // Source pointer
        AUDIO_CHANNEL_BUF_SIZE, // Number of transfers
        false                   // Start later, when chained
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block?
    if (add_interrupt) {
        dma_channel_set_irq0_enabled(*pChan0, true);
        irq_set_exclusive_handler(DMA_IRQ_0, i2s_dma_handler_0);
        irq_set_enabled(DMA_IRQ_0, true);
        handler_dma_channel_0 = *pChan0;

        dma_channel_set_irq1_enabled(*pChan1, true);
        irq_set_exclusive_handler(DMA_IRQ_1, i2s_dma_handler_1);
        irq_set_enabled(DMA_IRQ_1, true);
        handler_dma_channel_1 = *pChan1;
    }

    // Trigger channel 0 now, it will chain to channel 1
    dma_channel_start(*pChan0);
}

static void i2s_dma_setup(PIO pio) {
    // 1 DMA per buffer / PIO state machine
    i2s_dma_setup_sm(pio, sm_data0, capture_buf_data0[0], capture_buf_data0[1], &dma_chan0_0, &dma_chan0_1, true);
    i2s_dma_setup_sm(pio, sm_data1, capture_buf_data1[0], capture_buf_data1[1], &dma_chan1_0, &dma_chan1_1, false);
    i2s_dma_setup_sm(pio, sm_data2, capture_buf_data2[0], capture_buf_data2[1], &dma_chan2_0, &dma_chan2_1, false);
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
    i2s_program_load(pio, AUDIO_SAMPLE_RATE_HZ, I2S_CLK_PIN, I2S_WS_PIN, I2S_DATA0_PIN, I2S_DATA1_PIN, I2S_DATA2_PIN, TEST_PIN);

    // DMA Setup
    i2s_dma_setup(pio);

    // Start PIO (**at the same time** and in sync)
    pio_enable_sm_mask_in_sync(pio, (1u << sm_clocks) | (1u << sm_data0) | (1u << sm_data1) | (1u << sm_data2));

    //trigger analysis on the second core
//    multicore_launch_core1(core1_main);

    // LED flashing
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    while(true) {
        // wait for alternate DMA buffers to fill, and then process each

        dma_channel_wait_for_finish_blocking(dma_chan0_0);
        gpio_put(LED_PIN, 1);
        analyse_last_capture(AUDIO_CHANNEL_BUF_SIZE, capture_buf_data0[0], capture_buf_data1[0], capture_buf_data2[0]);
        gpio_put(LED_PIN, 0);

        dma_channel_wait_for_finish_blocking(dma_chan0_1);
        gpio_put(LED_PIN, 1);
        analyse_last_capture(AUDIO_CHANNEL_BUF_SIZE, capture_buf_data0[1], capture_buf_data1[1], capture_buf_data2[1]);
        gpio_put(LED_PIN, 0);
   }
    
    return 0;
}