#pragma once

#include "pico/stdlib.h"

#include "offsets.h"

#define AUDIO_SAMPLE_RATE_HZ 48000
#define AUDIO_NUM_BUFFERS    2   /*double-buffered DMA*/

// Audio buffers. These must fit in ram (only 260kb total)
#define AUDIO_SAMPLE_MILLISECONDS 100   /* ms per capture */
#define AUDIO_NUM_CHANNELS        2     /* stereo from i2s */
#define AUDIO_CHANNEL_BUF_LEN     (AUDIO_SAMPLE_RATE_HZ * AUDIO_NUM_CHANNELS * AUDIO_SAMPLE_MILLISECONDS) / 1000

#define NUM_BEST_MAGNITUDES  3

typedef struct magnitude_info {
    uint offset_num;
    uint64_t magnitude;
} magnitude_info_t;
extern magnitude_info_t best_magnitudes[NUM_BEST_MAGNITUDES];
extern uint64_t audio_magnitudes[SAMPLE_OFFSET_COUNT];

extern float averaged_best_x;
extern float averaged_best_y;

extern void audio_dma_init();
extern void audio_capture_analyse();
