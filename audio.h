#pragma once

#include "pico/stdlib.h"

#include "offsets.h"

#define AUDIO_SAMPLE_RATE_HZ 44100
#define AUDIO_NUM_BUFFERS    2   /*double-buffered DMA*/

// Audio buffers. These must fit in ram (only 260kb total)
#define AUDIO_SAMPLE_MILLISECONDS 100   /* ms per capture */
#define AUDIO_NUM_CHANNELS        2     /* stereo from i2s */
#define AUDIO_CHANNEL_BUF_LEN     (AUDIO_SAMPLE_RATE_HZ * AUDIO_NUM_CHANNELS * AUDIO_SAMPLE_MILLISECONDS) / 1000

extern uint32_t capture_magnitudes[SAMPLE_OFFSET_COUNT];

extern void audio_dma_init();
extern void audio_capture_analyse();
