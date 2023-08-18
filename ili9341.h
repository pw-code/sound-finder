#pragma once

#include "hardware/spi.h"

extern void lcd_init(spi_inst_t *spi);
extern void lcd_diag();
extern void lcd_draw_row(bool async, uint16_t row, uint16_t* buffer);
extern bool lcd_is_busy();
