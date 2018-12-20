#include "./Include/key.h"

#include "main.h"

uint16_t tmpState;

uint16_t scanKeys() {
	uint16_t value = 0;



	return value;
}

uint8_t checkKey(uint16_t key) {
	return (scanKeys() & key) != 0;
}

uint8_t checkKeyUp(uint16_t key) {
	if (checkKey(key)) {
		tmpState |= key;
		return 0;
	} else if (tmpState & key) {
		tmpState &= ~key;
		return 1;
	}
	return 0;
}

uint8_t waitKey(uint16_t key) {
	if (checkKey(key)) return 0;
	while (!checkKey(key));
	return 1;
}

uint8_t waitKeyUp(uint16_t key) {
	if (!checkKey(key)) return 0;
	while (checkKey(key));
	return 1;
}
