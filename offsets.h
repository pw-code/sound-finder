#pragma once

/* Mic offsets:
[(0.105, 0, 0.0),
 (0.05250000000000001, 0, 0.09093266739736605),
 (-0.05249999999999998, 0, 0.09093266739736607),
 (-0.105, 0, 1.2858791391047208e-17),
 (-0.05250000000000005, 0, -0.09093266739736602),
 (0.05249999999999993, 0, -0.0909326673973661)]
*/

#define SAMPLE_OFFSET_COUNT 90
#define SAMPLE_OFFSET_NUM_CHANNELS 6
#define SAMPLE_OFFSET_HZ 48000
#define SQUARE_SIZE 24

/* Sample Offsets[point,chan] */
extern const int sample_offsets[SAMPLE_OFFSET_COUNT][SAMPLE_OFFSET_NUM_CHANNELS];

/* Screen Offsets[point,chan] */
typedef struct screen_point {
    int x, y;
} screen_point_t;
extern const screen_point_t screen_offsets[SAMPLE_OFFSET_COUNT];
