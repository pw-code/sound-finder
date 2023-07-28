#include <stdio.h>
#include "pico/stdlib.h"

extern volatile uint32_t capture_count;
extern volatile bool capture_ready;

void analyse_last_capture(uint capture_length, uint32_t* capture_buf_data0, uint32_t* capture_buf_data1, uint32_t* capture_buf_data2) {

    // dump capture buffer 0, left channel, as hex
    printf("CAPTURE %d\n", capture_count);
    for (uint i=0; i<capture_length; ++i) {
        //low byte tells us which channel
        //TODO: restring
        printf("%08lx\t", capture_buf_data0[i]);
    }
    printf("=====================\n\n\n");

    capture_ready = false; //waiting for new data
}