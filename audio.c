#include <stdio.h>
#include "hardware/dma.h"

#include "audio.h"
#include "offsets.h"
#if AUDIO_SAMPLE_RATE_HZ != SAMPLE_OFFSET_HZ
# error ("Mismatched sample rates " AUDIO_SAMPLE_RATE_NZ " != " SAMPLE_OFFSET_HZ)
#endif

#include "pins.h"
#include "i2s.pio.h"


//double-buffer between DMA samples
uint32_t capture_buf_data0[2][AUDIO_CHANNEL_BUF_LEN];
uint32_t capture_buf_data1[2][AUDIO_CHANNEL_BUF_LEN];
uint32_t capture_buf_data2[2][AUDIO_CHANNEL_BUF_LEN];


uint handler_dma_channel_0, handler_dma_channel_1;
uint dma_chan0_0, dma_chan0_1;
uint dma_chan1_0, dma_chan1_1;
uint dma_chan2_0, dma_chan2_1;

volatile uint32_t capture_count = 0;


const char HEX_DIGITS[16] = "0123456789ABCDEF";




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
        AUDIO_CHANNEL_BUF_LEN,  // Number of transfers
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
        AUDIO_CHANNEL_BUF_LEN,  // Number of transfers
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


/**
 * @brief Initialise i2s audio capture using PIO and DMA
 * 
 */
void audio_dma_init(PIO pio) {
    i2s_program_load(pio, AUDIO_SAMPLE_RATE_HZ, PIN_I2S_CLK, PIN_I2S_WS, PIN_I2S_DATA0, PIN_I2S_DATA1, PIN_I2S_DATA2, PIN_TEST);

    // DMA Setup
    i2s_dma_setup(pio);

    // Start PIO (**at the same time** and in sync)
    pio_enable_sm_mask_in_sync(pio, (1u << sm_clocks) | (1u << sm_data0) | (1u << sm_data1) | (1u << sm_data2));
}



static void analyse_capture(uint buffer_num) {

    // dump capture buffer 0, left channel, as hex
    printf("CAPTURE %d: %d samples\n", capture_count, AUDIO_CHANNEL_BUF_LEN);
    //fflush(stdout);
    char buf[128];
    int p = 0;
    for (uint i=0; i<AUDIO_CHANNEL_BUF_LEN; i++) {
        //low byte tells us which channel
        if ((capture_buf_data0[buffer_num][i] & 1) == 0) {
            //output in blocks to improve speed (buffered), plus skip printf as it is slow too.
            //pico is little endian, so we are writing little-endian data 
            uint8_t * pVal = (uint8_t*)(&capture_buf_data0[buffer_num][i]);

            buf[p++] = HEX_DIGITS[(pVal[3]>>4) & 0xF];
            buf[p++] = HEX_DIGITS[(pVal[3])    & 0xF];
            buf[p++] = HEX_DIGITS[(pVal[2]>>4) & 0xF];
            buf[p++] = HEX_DIGITS[(pVal[2])    & 0xF];
            buf[p++] = HEX_DIGITS[(pVal[1]>>4) & 0xF];
            buf[p++] = HEX_DIGITS[(pVal[1])    & 0xF];
            buf[p++] = HEX_DIGITS[(pVal[0]>>4) & 0xF];
            buf[p++] = HEX_DIGITS[(pVal[0])    & 0xF];

            buf[p++] = ' ';
            if (p >= (sizeof(buf) - sizeof(uint32_t)*2)) {
                buf[p] = 0;
                fputs(buf, stdout);
                p = 0;
            }
        }
    }
    buf[p++] = '\n';
    buf[p++] = '\n';
    buf[p] = 0;
    fputs(buf, stdout);
    //fflush(stdout);
}


/**
 * @brief Capture a set of DMA samples and analyse them
 * 
 */
void audio_capture_analyse() {
    // wait for alternate DMA buffers to fill, and then process each

    dma_channel_wait_for_finish_blocking(dma_chan0_0);
    gpio_put(PIN_LED, 1);
    analyse_capture(0);
    gpio_put(PIN_LED, 0);

    dma_channel_wait_for_finish_blocking(dma_chan0_1);
    gpio_put(PIN_LED, 1);
    analyse_capture(1);
    gpio_put(PIN_LED, 0);
}
