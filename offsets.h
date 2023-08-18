#pragma once

/* Mic offsets:
[(0.12, 0, 0.0),
 (0.06000000000000001, 0, 0.10392304845413262),
 (-0.05999999999999997, 0, 0.10392304845413264),
 (-0.12, 0, 1.4695761589768237e-17),
 (-0.06000000000000005, 0, -0.1039230484541326),
 (0.059999999999999915, 0, -0.10392304845413268)]
*/

#define SAMPLE_OFFSET_WIDTH 8 /* x axis */
#define SAMPLE_OFFSET_DEPTH 6 /* y axis */
#define SAMPLE_OFFSET_HEIGHT 3 /* z axis */
#define SAMPLE_OFFSET_NUM_CHANNELS 6
#define SAMPLE_OFFSET_HZ 44100

/* Sample Offsets[x,y,z,chan] */
extern const int sample_offsets[SAMPLE_OFFSET_WIDTH][SAMPLE_OFFSET_DEPTH][SAMPLE_OFFSET_HEIGHT][SAMPLE_OFFSET_NUM_CHANNELS];
