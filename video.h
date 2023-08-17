#pragma once

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"


// OV7670: QVGA: 320x240
// Also TFT LCD: 240x320 (will need rotation during output)
#define VIDEO_COLUMNS 320
#define VIDEO_ROWS    240


extern uint16_t video_buffer[2][1 + VIDEO_COLUMNS];
extern _Atomic uint8_t last_video_buf;


extern void video_init();
extern void video_stream();
