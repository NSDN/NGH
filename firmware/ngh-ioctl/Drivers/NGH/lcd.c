#include "./Include/lcd.h"
#include "./Include/reglcd.h"

#define SDA_PIN CTS_SDI_GPIO_Port,CTS_SDI_Pin
#define SCK_PIN CTS_SCK_GPIO_Port,CTS_SCK_Pin
#define NCS_PIN CTS_NCS_GPIO_Port,CTS_NCS_Pin

__IO uint8_t dcs = 0;

void _spi_start() {
	HAL_GPIO_WritePin(NCS_PIN, GPIO_PIN_RESET);
}

void _spi_dcs(uint8_t data) {
	dcs = data;
}

void _spi_stop() {
	HAL_GPIO_WritePin(NCS_PIN, GPIO_PIN_SET);
}

void _spi_send(uint8_t byte) {
#define __setpin(p, v) HAL_GPIO_WritePin(p, (v) ? GPIO_PIN_SET : GPIO_PIN_RESET)
	__setpin(SDA_PIN, dcs);
	__setpin(SCK_PIN, 0);
	__setpin(SCK_PIN, 1);
	for (uint8_t i = 0; i < 8; i++) {
		__setpin(SDA_PIN, byte & (0x80 >> i));
		__setpin(SCK_PIN, 0);
		__setpin(SCK_PIN, 1);
	}
#undef __setpin
}

void LCDInit() {
	uint32_t i = 0;
	uint16_t r = 0, len = 0, x = 0;
	uint16_t size = sizeof(_regValues) / sizeof(unsigned short);
	while(i < size) {
		r = _regValues[i++];
		len = _regValues[i++];
		if(r == LCD_DELAY) {
			HAL_Delay(len);
		} else {
			_spi_start();
			_spi_dcs(0);
			_spi_send(r & 0xFF);
			_spi_stop();

			_spi_start();
			_spi_dcs(1);
			for (uint16_t d = 0; d < len; d++) {
				x = _regValues[i++];
				_spi_send(x & 0xFF);
			}
			_spi_stop();
		}
    }
}
