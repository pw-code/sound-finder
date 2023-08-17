#pragma once

#include "hardware/i2c.h"
#include "hardware/pio.h"

extern void ov7670_init(PIO pio, i2c_inst_t *i2c);
extern void ov7670_diag();
extern void ov7670_wait_for_buffer();
