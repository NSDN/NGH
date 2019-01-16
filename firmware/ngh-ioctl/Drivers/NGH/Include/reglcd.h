#ifndef __REGLCD_H_
#define __REGLCD_H_


#define LCD_INVON		0x20
#define LCD_INVOFF		0x21
#define LCD_CADDR		0x2A
#define LCD_PADDR		0x2B
#define LCD_MEMWR		0x2C
#define LCD_MADCTL		0x36

#define LCD_MADCTL_MY  0x80
#define LCD_MADCTL_MX  0x40
#define LCD_MADCTL_MV  0x20
#define LCD_MADCTL_ML  0x10
#define LCD_MADCTL_RGB 0x00
#define LCD_MADCTL_BGR 0x08
#define LCD_MADCTL_MH  0x04
#define LCD_MADCTL_HF  0x02
#define LCD_MADCTL_VF  0x01

#define LCD_DELAY 0xFFFF
const unsigned short _regValues[] = {
	0xFF, 3, 0xFF, 0x98, 0x06,  // EXTC Command Set Enable Register
	0xBA, 1, 0xE0, // SPI Interface Setting
	0xBC, 21, 0x03, 0x0F, 0x63, 0x69, 0x01, 0x01, 0x1B, 0x11, 0x70, 0x73, 0xFF, 0xFF, 0x08, 0x09, 0x05, 0x00, 0xEE, 0xE2, 0x01, 0x00, 0xC1, // GIP 1
	0xBD, 8, 0x01, 0x23, 0x45, 0x67, 0x01, 0x23, 0x45, 0x67, // GIP 2
	0xBE, 9, 0x00, 0x22, 0x27, 0x6A, 0xBC, 0xD8, 0x92, 0x22, 0x22, // GIP 3
	0xC7, 1, 0x1E, // VCom
	0xED, 3, 0x7F, 0x0F, 0x00, // Enable Volt Register
	0xC0, 3, 0xE3, 0x0B, 0x00, // Power Control 1
	0xFC, 1, 0x08, // LVGL Voltage Setting
	0xDF, 6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, // Engineering Setting
	0xF3, 1, 0x74, // DVDD Voltage Setting
	0xB4, 3, 0x00, 0x00, 0x00, // Display Inversion Control
	0xF7, 1, 0x81, // Panel Resolution Selection Set (480x854)
	0xB1, 3, 0x00, 0x10, 0x14, // Frame Rate
	0xF1, 3, 0x29, 0x8A, 0x07, // Panel Timing Control
	0xF2, 4, 0x40, 0xD2, 0x50, 0x28, // Panel Timing Control
	0xC1, 4, 0x17, 0x85, 0x85, 0x20, // Power Control 2
	0xE0, 16, 0x00, 0x0C, 0x15, 0x0D, 0x0F, 0x0C, 0x07, 0x05, 0x07, 0x0B, 0x10, 0x10, 0x0D, 0x17, 0x0F, 0x00, // Positive Gamma Control
	0xE1, 16, 0x00, 0x0D, 0x15, 0x0E, 0x10, 0x0D, 0x08, 0x06, 0x07, 0x0C, 0x11, 0x11, 0x0E, 0x17, 0x0F, 0x00, // Negative Gamma Correction
	0x36, 1, 0x22, // Memory Access Control, 00/22
	0x3A, 1, 0x70, // Interface Pixel Format (RGB 24 bit/pixel)
	0xB6, 1, 0x22, // Display Function Control, A2/22
	0xB0, 1, 0x00, // Interface Mode Control
	0x2A, 4, 0x00, 0x00, 0x03, 0x55,
	0x2B, 4, 0x00, 0x00, 0x01, 0xDF,
	0x11, 0, // Sleep Out
	LCD_DELAY, 120,
	0x29, 0, // Display ON
};


#endif
