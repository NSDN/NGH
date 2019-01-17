#ifndef __LTDC_H_
#define __LTDC_H_


#include "halinc.h"
#include "dram.h"

#define GBUF_ADDR SDRAM_ADDR
#define GBUF_WIDTH 854
#define GBUF_HEIGH 480
#define GBUF_SIZE (GBUF_WIDTH * GBUF_HEIGH * 4)

void pos(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void write(uint32_t data);
void writes(uint32_t* data, uint32_t len);
void flash(uint32_t data, uint32_t n);


#endif
