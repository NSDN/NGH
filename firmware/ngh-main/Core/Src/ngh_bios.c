#include "ngh_bios.h"
#include "stm32h7xx_hal.h"
#include "fatfs.h"

#include <setjmp.h>
#include <string.h>

#include "nshel.h"
#include "nsio.h"
#include "logo.h"
#include "dram.h"
#include "lcd.h"

#define NGH_SYS_VERSION "190117"

LCD* lcd;
FILTYPE file;
jmp_buf rstPos;
uint8_t FS_OK = 0;
HAL_SD_CardInfoTypeDef cardInfo;
extern SD_HandleTypeDef hsd1;

void greenScreen(const char* head) {
	lcd->colorb(lcd->p, 0x007F00);
	lcd->colorf(lcd->p, 0xFFFFFF);
	lcd->font(lcd->p, Small);
	lcd->clear(lcd->p);
	lcd->printfc(lcd->p, 64, "%s", head);
	lcd->printfc(lcd->p, 100, "WE DO NOT KNOW");
	lcd->printfc(lcd->p, 110, "WHAT HAD HAPPENED");
}

//extern void __DRAM_SET_();
//extern void __DRAM_RESET_();

extern void tick();

void delay(int t) { HAL_Delay(t); }

void processEvent() {  }

void ngh_setup() {
	lcd = LCDInit();
	setjmp(rstPos);
	//__DRAM_RESET_();

	lcd->init(lcd->p);
	delay(100);
	lcd->rotate(lcd->p, LCD_LANDSCAPE);
	lcd->font(lcd->p, Small);
	lcd->colorb(lcd->p, 0xFFFFFF);
	lcd->colorf(lcd->p, 0x000000);
	lcd->clear(lcd->p);
	delay(1000);

	lcd->bitmapsc(lcd->p, lcd->p->width / 2, 140, 64, 64, getLogo());
	lcd->printfc(lcd->p, 180, "nyagame hydro");
	delay(1000);
	lcd->clear(lcd->p);

	lcd->font(lcd->p, Big);

	print("NyaGame Hydro Factory System\n");
	print("Version: %s\n\n", NGH_SYS_VERSION);
	delay(1000);

	/* Initialize device */
	print("Init ARM(R) Avalon(R) external bus...\n");
	print("\n");
	delay(1000);

	print("Reset NSDN(C) NGH-IOCTL chip...\n");
	print("Reset NSDN(C) NGH-AUDIO chip...\n");
	print("\n");
	delay(1000);

	print("Test SDRAM chip...\n");
	__IO uint16_t* ptr = (uint16_t*) SDRAM_ADDR;
	uint32_t gramOffset = 854 * 480 * 4;
	uint32_t intOffset = 0x32, strOffset = 0x64;
	*(ptr + gramOffset + intOffset) = 0x3232;
	strcpy((char*) (ptr + gramOffset + strOffset), "Hello, world!\0");
	print("Output: %x\n", *(ptr + gramOffset + intOffset));
	print("Output: %s\n", ptr + gramOffset + strOffset);
	print("\n");
	delay(1000);

	if (HAL_SD_Init(&hsd1) == HAL_OK) {
		print("Init SD card... OK\n");
		HAL_SD_GetCardInfo(&hsd1, &cardInfo);
		print("Card size: %dMB\n", (cardInfo.BlockNbr * cardInfo.BlockSize) >> 20);
	} else {
		print("Init SD card... ERR\n");
	}
	print("\n");
	delay(1000);

	uint8_t result = 0;
	print("Mount file system...\n");
	result = f_mount(&SDFatFS, SDPath, 1);
	delay(1000);
	if(result == FR_OK) {
		char path[] = "NGV_INFO.TXT";
		f_open(&file, path, FA_WRITE | FA_CREATE_ALWAYS);
		f_printf(&file, "NyaGame Hydro v1.0 with STM32H743ZIT6 and STM32F0/F1/F3 Co-CPUs\n");
		f_printf(&file, "by NyaSama Developer Network\n");
		f_printf(&file, "Firmware Version: %s\n", NGH_SYS_VERSION);
		f_close(&file);
		print("Test file system... OK\n");
		FS_OK = 1;
	} else {
		print("Test file system... ERR: %02X\n", result);
		FS_OK = 0;
	}
	print("\n");
	delay(1000);

	print("Test SDRAM chip again...\n");
	print("Output: %x\n", *(ptr + gramOffset + intOffset));
	print("Output: %s\n", ptr + gramOffset + strOffset);
	print("\n");
	delay(1000);

	print("\n");
	/* Initialize end */

	delay(1000);

	print("Loading NSHEL...\n");

	print("\n");

	//__DRAM_SET_();

	const char* args[] = { "nshel", "init.d" };
	nshel(2, (char**) args);
	print("\n");
	delay(5000);
}

volatile uint8_t cnt = 0;
void ngh_loop() {
	lcd->colorb(lcd->p, 0xFF0000);
	lcd->clear(lcd->p);
	lcd->colorb(lcd->p, 0x00FF00);
	lcd->clear(lcd->p);
	lcd->colorb(lcd->p, 0x0000FF);
	lcd->clear(lcd->p);
	if (cnt < 128) cnt += 1;
	else {
		cnt = 0;
		greenScreen("END OF SYSTEM");
		delay(3000);
		longjmp(rstPos, 0);
	}
}
