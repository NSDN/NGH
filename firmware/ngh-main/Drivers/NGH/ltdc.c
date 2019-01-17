#include "./Include/ltdc.h"

typedef struct {
	uint16_t x, y;
	uint16_t w, h;
} Rect;

__IO Rect rect = { 0 };
__IO uint32_t _ptr = 0;

#define PIXEL(x, y, data) (*(uint32_t*) (GBUF_ADDR + ((x) + GBUF_WIDTH * (y)) * 4) = (data))

void pos(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	rect.x = x1; rect.y = y1;
	rect.w = x2 - x1 + 1; rect.h = y2 - y1 + 1;
	_ptr = 0;
}

void write(uint32_t data) {
	if (_ptr < rect.w * rect.h) {
		PIXEL(rect.x + _ptr % rect.w, rect.y + _ptr / rect.w, data);
		_ptr += 1;
	} else _ptr = 0;
}

void writes(uint32_t* data, uint32_t len) {
	while (_ptr < rect.w * rect.h) {
		PIXEL(rect.x + _ptr % rect.w, rect.y + _ptr / rect.w, data[_ptr]);
		_ptr += 1;
		if (_ptr >= len) break;
	}
}

void flash(uint32_t data, uint32_t n) {
	while (_ptr < rect.w * rect.h) {
		PIXEL(rect.x + _ptr % rect.w, rect.y + _ptr / rect.w, data);
		_ptr += 1;
		if (_ptr >= n) break;
	}
}
