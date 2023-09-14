#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hardware/dma.h"

#include "audio.h"
#include "offsets.h"

//Validate audio.h and generated offsets.h agree on sample rate (would affect positioning calculations)
#if AUDIO_SAMPLE_RATE_HZ != SAMPLE_OFFSET_HZ
# error ("Mismatched sample rates " AUDIO_SAMPLE_RATE_NZ " != " SAMPLE_OFFSET_HZ)
#endif

#include "pins.h"
#include "i2s.pio.h"


//double-buffer between DMA samples
int32_t capture_buf_data0[2][AUDIO_CHANNEL_BUF_LEN];
int32_t capture_buf_data1[2][AUDIO_CHANNEL_BUF_LEN];
int32_t capture_buf_data2[2][AUDIO_CHANNEL_BUF_LEN];
_Atomic uint8_t last_capture_buf;


int from_index;
int to_index;

magnitude_info_t best_magnitudes[NUM_BEST_MAGNITUDES];
uint64_t audio_magnitudes[SAMPLE_OFFSET_COUNT];


uint handler_dma_channel_0, handler_dma_channel_1;
uint dma_chan0_0, dma_chan0_1;
uint dma_chan1_0, dma_chan1_1;
uint dma_chan2_0, dma_chan2_1;

volatile uint32_t capture_count = 0;


const char HEX_DIGITS[16] = "0123456789ABCDEF";



void i2s_dma_handler() {
    capture_count++;

    // which DMA channel triggered IRQ 0?
    if (dma_channel_get_irq0_status(handler_dma_channel_0)) {

        //channel 0
        last_capture_buf = 0;

        // clear the interrupt
        dma_channel_acknowledge_irq0(handler_dma_channel_0);

        // All DMA channels should have finished at once (the PIO's are synchronised)

        // Reset buffer, ready for next time we are triggered
        dma_channel_set_write_addr(dma_chan0_0, capture_buf_data0[0], false);
        dma_channel_set_write_addr(dma_chan1_0, capture_buf_data1[0], false);
        dma_channel_set_write_addr(dma_chan2_0, capture_buf_data2[0], false);

    } else {

        // channel 1
        last_capture_buf = 1;

        // clear the interrupt
        dma_channel_acknowledge_irq0(handler_dma_channel_1);

        // All DMA channels should have finished at once (the PIO's are synchronised)

        // Reset buffer, ready for next time we are triggered
        dma_channel_set_write_addr(dma_chan0_1, capture_buf_data0[1], false);
        dma_channel_set_write_addr(dma_chan1_1, capture_buf_data1[1], false);
        dma_channel_set_write_addr(dma_chan2_1, capture_buf_data2[1], false);
    }
}


static void i2s_dma_setup_sm(PIO pio, uint sm, void *capture_buf0, void *capture_buf1, uint* pChan0, uint* pChan1, bool add_interrupt) {

    // Two alternative DMA transfers, with different buffers (reloaded in interrupt handler)

    *pChan0 = dma_claim_unused_channel(true);
    *pChan1 = dma_claim_unused_channel(true);

    dma_channel_config c0 = dma_channel_get_default_config(*pChan0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);
    channel_config_set_read_increment(&c0, false);
    channel_config_set_write_increment(&c0, true);
    channel_config_set_dreq(&c0, pio_get_dreq(pio, sm, false));
    //channel_config_set_chain_to(&c0, *pChan1); //Now triggered in audio loop so we ensure buffer contents
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
    //channel_config_set_chain_to(&c1, *pChan0); //Now triggered in audio loop so we ensure buffer contents
    dma_channel_configure(*pChan1, &c1,
        capture_buf1,           // Destination pointer
        &pio->rxf[sm],          // Source pointer
        AUDIO_CHANNEL_BUF_LEN,  // Number of transfers
        false                   // Start later, when chained
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block (ov7670.c uses IRQ 1 for video)
    if (add_interrupt) {
        dma_channel_set_irq0_enabled(*pChan0, true);
        dma_channel_set_irq0_enabled(*pChan1, true);

        handler_dma_channel_0 = *pChan0;
        handler_dma_channel_1 = *pChan1;

        irq_set_exclusive_handler(DMA_IRQ_0, i2s_dma_handler);
        irq_set_enabled(DMA_IRQ_0, true);
    }

    // Trigger channel 0 now, it will chain to channel 1
    //dma_channel_start(*pChan0); //Now triggered in audio loop so we ensure buffer contents
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

//====================================================================================================


// Whether to dump audio buffers for debugging
#undef DEBUG_DUMP_AUDIO_BUFFER

#ifdef DEBUG_DUMP_AUDIO_BUFFER

// Which audio buffer to debug-dump (0..2)
#define DEBUG_DUMP_AUDIO_BUF capture_buf_data1
// channel, 0==left, 1==right
#define DEBUG_DUMP_AUDIO_CHANNEL 0

static void capture_dump(uint buffer_num) {

    printf("CAPTURE %d: %d samples\n", capture_count, AUDIO_CHANNEL_BUF_LEN);
    //fflush(stdout);

    char buf[128];
    int p = 0;
    for (uint i=0; i<AUDIO_CHANNEL_BUF_LEN; i++) {
        //low byte tells us which channel
        {//if ((DEBUG_DUMP_AUDIO_BUF[buffer_num][i] & 1) == DEBUG_DUMP_AUDIO_CHANNEL) {
            //output in blocks to improve speed (buffered), plus skip printf as it is slow too.
            uint8_t * pVal = (uint8_t*)(&DEBUG_DUMP_AUDIO_BUF[buffer_num][i]);

            // big-endian 32bit pcm encoding
            // example decoding: xxd -p -r <minicom.cap >audio.au
            // load in audacity as import raw data: signed 32bit pcm, big-endian, 1 channel, 44100hz
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
#endif


// Whether to audio magnitudes for debugging
#undef DEBUG_DUMP_AUDIO_MAGNITUDES

#ifdef DEBUG_DUMP_AUDIO_MAGNITUDES

static void magnitudes_dump() {

    printf("MAGNITUDES\n");
    printf(" BEST: ");
    for (uint m=0; m<NUM_BEST_MAGNITUDES; m++) {
        printf(" %d:%llu", best_magnitudes[m].offset, best_magnitudes[m].magnitude);
    }
    printf("\n ALL: ");
    for (uint s=0; s<SAMPLE_OFFSET_COUNT; s++) {
        char marker = ' ';
        for (uint m=0; m<NUM_BEST_MAGNITUDES; m++) {
            if (best_magnitudes[m].offset == s) {
                marker = '*';
                break;
            }
        }
        printf(" %c%llu", marker, audio_magnitudes[s]);
    }
    putchar('\n');
}
#endif

//====================================================================================================


/**
 * @brief adjust the capture buffer pointer, so that it is pointing at a left(0) or right(1) sample
 */
static inline uint32_t * adjust_left_right(int32_t wanted, int32_t * buffer) {
    // We can tell left/rigth channel, as the last bit (LSB) is the channel indicator
    if ((*buffer & 1) != (wanted & 1)) {
        buffer++;
    }
    return buffer;
}


static void analyse_capture(uint buffer_num) {
//!!!!!
// TODO: ensure that these left/right channels, for each of the 3 I2S strings, actually matches the mic offsets in offsets.h
// mics are arrange (viewed from the back of the device), starting center right, and moving anticlockwise
//!!!!!
    uint32_t * buffer[SAMPLE_OFFSET_NUM_CHANNELS] = {
        adjust_left_right(0, &capture_buf_data1[buffer_num][0]),
        adjust_left_right(0, &capture_buf_data0[buffer_num][0]),
        adjust_left_right(1, &capture_buf_data0[buffer_num][1]),
        adjust_left_right(0, &capture_buf_data2[buffer_num][0]),
        adjust_left_right(1, &capture_buf_data2[buffer_num][1]),
        adjust_left_right(1, &capture_buf_data1[buffer_num][1]) };

    const uint num_samples = (to_index - from_index) / 2;

    // Analyse stereo sample at each mic position:
    // * Sum all the audio samples from the various mic offsets 
    // * They will add together if in sync, and partially cancel each other out otherwise.
    // * Record the average amplitude of the audio at this mic offset
    // * Keep the top 3 positions for display later

    memset(best_magnitudes, 0, sizeof(best_magnitudes));

    // For each of the sample offsets, calculate the merged audio's average magnitude and record it for later
    for (uint offset_num = 0; offset_num < SAMPLE_OFFSET_COUNT; ++offset_num) {

        // Analyse stereo sample at this position:
        // * Sum all the audio at various offsets (they will add if in sync, and partially cancel each other out otherwise)
        // * Record the average amplitude of the audio at this mic offset
        // 
        uint64_t sum = 0;
        for (uint i = from_index; i < to_index; i += 2) {

            int32_t sample = 0;
            for (uint channel = 0; channel < SAMPLE_OFFSET_NUM_CHANNELS; ++channel) {
                int o = sample_offsets[offset_num][channel];
                sample += buffer[channel][i - (o * 2)];
            }

            //sum += (abs(sample) / SAMPLE_OFFSET_NUM_CHANNELS);
            sum += (abs(sample) >> 3); // div 8 is close enough for us, and much faster than an actual divide (by 6 channels)
        }

        uint64_t capture_magnitude = sum / num_samples;
        audio_magnitudes[offset_num] = capture_magnitude;

        // insert into best_magnitudes, keeping the best at the top
        uint m;
        for (m = 0; m < NUM_BEST_MAGNITUDES; ++m) {
            if (capture_magnitude > best_magnitudes[m].magnitude) {
                // insert here

                // make room
                for (uint x = NUM_BEST_MAGNITUDES - 1; x > m; x--) {
                    best_magnitudes[x] = best_magnitudes[x - 1];
                }

                // insert
                best_magnitudes[m].magnitude = capture_magnitude;
                best_magnitudes[m].offset_num = offset_num;

                break;
            }
        }
        // it goes last?
        if (m >= NUM_BEST_MAGNITUDES) {
            best_magnitudes[NUM_BEST_MAGNITUDES - 1].magnitude = capture_magnitude;
            best_magnitudes[NUM_BEST_MAGNITUDES - 1].offset_num = offset_num;
        }
    }

#ifdef DEBUG_DUMP_AUDIO_BUFFER
    capture_dump(buffer_num);
#endif

#ifdef DEBUG_DUMP_AUDIO_MAGNITUDES
    magnitudes_dump();
#endif
}


/**
 * @brief Capture a set of DMA samples and analyse them
 * 
 */
void audio_capture_analyse() {
    // audio channels are analysed offset from each other
    // calculate 'from' & 'to' sample indexes that are always safe to use with all possible offsets
    from_index = 0;
    to_index = AUDIO_CHANNEL_BUF_LEN - 1;
    for (uint s = 0; s < SAMPLE_OFFSET_COUNT; ++s) {
        for (uint c = 0; c < SAMPLE_OFFSET_NUM_CHANNELS; ++c) {
            int o = sample_offsets[s][c];
            int index = o * 2; //because there are two samples per buffer

            // inefficient, but I don't care, it only runs once, and this is easier to think about
            while ((from_index+index) < 0) {
                from_index += 2;
            }
            while ((to_index+index) >= (AUDIO_CHANNEL_BUF_LEN-2)) {
                to_index -= 2;
            }
        }
    }

    // start capture cycle
    dma_start_channel_mask((1u << dma_chan0_0) | (1u << dma_chan1_0) | (1u << dma_chan2_0));

    // main audio loop, capture and analyse audio samples
    while(true) {
        // wait for alternate DMA buffers to fill, and then process each whilst triggering the next

        dma_channel_wait_for_finish_blocking(dma_chan0_0);
        dma_start_channel_mask((1u << dma_chan0_1) | (1u << dma_chan1_1) | (1u << dma_chan2_1)); //trigger the other, while we analyse this
        gpio_put(PIN_LED, 0);
        analyse_capture(0);


        dma_channel_wait_for_finish_blocking(dma_chan0_1);
        dma_start_channel_mask((1u << dma_chan0_0) | (1u << dma_chan1_0) | (1u << dma_chan2_0)); //trigger the other, while we analyse this
        gpio_put(PIN_LED, 1);
        analyse_capture(1);
    }
}
