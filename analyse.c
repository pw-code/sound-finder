#include <stdio.h>
#include <stdatomic.h>
#include "pico/stdlib.h"

extern volatile uint32_t capture_count;

const char HEX_DIGITS[16] = "0123456789ABCDEF";

void analyse_last_capture(uint capture_length, uint32_t* capture_buf_data0, uint32_t* capture_buf_data1, uint32_t* capture_buf_data2) {

    // dump capture buffer 0, left channel, as hex
    printf("CAPTURE %d: %d samples\n", capture_count, capture_length);
    //fflush(stdout);
    char buf[128];
    int p = 0;
    for (uint i=0; i<capture_length; i++) {
        //low byte tells us which channel
        if ((capture_buf_data0[i] & 1) == 0) {
            //output in blocks to improve speed (buffered), plus skip printf as it is slow too.
            //pico is little endian, so we are writing little-endian data 
            uint8_t * pVal = (uint8_t*)(&capture_buf_data0[i]);
            for (int byt=0; byt<4; ++byt) {
                buf[p++] = HEX_DIGITS[(*pVal>>4) & 0xF];
                buf[p++] = HEX_DIGITS[(*pVal)    & 0xF];
                pVal++;
            }
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