#include "base.h"

#include "./Drivers/DxLib/DxLib.h"

#include <cstdlib>
#include <cstdio>
#include <ctime>

#include <string>

void processEvent();
uint32_t colorBuf;
struct {
    uint16_t x, y;
    uint16_t w, h;
} rect;

const uint16_t LCD_WIDTH = 854, LCD_HEIGHT = 480;
static uint16_t LCD_SCALE = 1;

static uint32_t LCD_PTR;
static uint8_t LCD_SPD = 0;

extern uint8_t initFlag, endFlag, reset;
extern uint16_t keyValue;

#include <setjmp.h>
extern jmp_buf rstPos;

static uint8_t _progress = 0;
static uint16_t _count = 0;
void progress() {
    char c;
    if (_count < 100) { _count += 1; return; }
    else {
        _count = 0;

        switch (_progress) {
            case 0: c = '-';    _progress += 1; break;
            case 1: c = '\\';   _progress += 1; break;
            case 2: c = '|';    _progress += 1; break;
            case 3: c = '/';    _progress  = 0; break;
        }
        printf("%c\b", c);
    }
}

void delay(uint32_t ms) { 
    clock_t begin = clock();
    while ((uint32_t) ((double) (clock() - begin) / CLK_TCK * 1000) < ms)
        processEvent();
}

#define pixel(x, y) DxLib::DrawPixel((x), (y), colorBuf)

void color(uint32_t color) {
    colorBuf = color;
}

void pos(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    rect.x = x1; rect.y = y1;
	rect.w = x2 - x1 + 1; rect.h = y2 - y1 + 1;
	LCD_PTR = 0;
}

void write(uint32_t data) {
    if (LCD_PTR < rect.w * rect.h) {
        color(data);
        pixel(rect.x + LCD_PTR % rect.w, rect.y + LCD_PTR / rect.w);
        LCD_PTR += 1;
    } else LCD_PTR = 0;
    DxLib::ScreenFlip();
    processEvent();
}

void writes(uint32_t* data, uint32_t len) {
    while (LCD_PTR < rect.w * rect.h) {
        color(data[LCD_PTR]);
        pixel(rect.x + LCD_PTR % rect.w, rect.y + LCD_PTR / rect.w);
        LCD_PTR += 1;
        if (LCD_PTR >= len) break;
    }
    DxLib::ScreenFlip();
    processEvent();
}

void flash(uint32_t data, uint32_t n) {
    color(data);
    while (LCD_PTR < rect.w * rect.h) {
        pixel(rect.x + LCD_PTR % rect.w, rect.y + LCD_PTR / rect.w);
        LCD_PTR += 1;
        if (LCD_PTR >= n) break;
    }
    DxLib::ScreenFlip();
    processEvent();
}

#undef pixel

void playMusic(std::string path) {
    TCHAR* buff = (TCHAR*) malloc((path.length() + 1) * sizeof(TCHAR));
    for (int i = 0; i < path.length(); i++)
        buff[i] = path[i];
    buff[path.length()] = '\0';
    DxLib::PlayMusic(buff, DX_PLAYTYPE_LOOP);
    free(buff);
}

void stopMusic() {
    DxLib::StopMusic();
}

void initBase() {
    DxLib::SetWindowText(_T("ngh-emu"));
    DxLib::SetOutApplicationLogValidFlag(0);
    DxLib::ChangeWindowMode(1);
    DxLib::SetWindowStyleMode(5);

    DxLib::SetFullScreenScalingMode(DX_FSSCALINGMODE_NEAREST);
    DxLib::SetGraphMode(LCD_WIDTH, LCD_HEIGHT, 32);
    DxLib::SetWindowSize(LCD_WIDTH, LCD_HEIGHT);
    DxLib::SetDrawScreen(DX_SCREEN_BACK);
    DxLib::SetAlwaysRunFlag(1);

    DxLib::DxLib_Init();
}

void deinitBase() {
    DxLib::DxLib_End();
}

void resizeWindow(uint16_t scale) {
    LCD_SCALE = scale;
    int maxW = 0, maxH = 0;
    DxLib::GetDisplayMaxResolution(&maxW, &maxH);
    int width = LCD_WIDTH * LCD_SCALE, height = LCD_HEIGHT * LCD_SCALE;
    DxLib::SetWindowSize(width, height);
    DxLib::SetWindowPosition((maxW - width) / 2, (maxH - height) / 2);
}

#include "util.h"

void processEvent() {
    if (DxLib::ProcessMessage() == -1) {
        if (initFlag == 0) {
            deinitBase();
            exit(0);
        } else endFlag = 1;
    }

    if (DxLib::CheckHitKey(Keys::KeyESCAPE)) {
        if (initFlag == 0) {
            deinitBase();
            exit(0);
        } else endFlag = 1;
    } else if (DxLib::CheckHitKey(Keys::KeyBACK)) {
        if (initFlag == 0)
            longjmp(rstPos, 0);
        else reset = 1;
    }

#define __SCAN_KEY(code, value) do { \
        if (DxLib::CheckHitKey(code)) \
            keyValue |= value; \
        else \
            keyValue &= ~value; \
    } while (0)

    __SCAN_KEY(Keys::KeyW, LPAD_UP);
    __SCAN_KEY(Keys::KeyS, LPAD_DOWN);
    __SCAN_KEY(Keys::KeyA, LPAD_LEFT);
    __SCAN_KEY(Keys::KeyD, LPAD_RIGHT);
    __SCAN_KEY(Keys::KeyUP, RPAD_UP);
    __SCAN_KEY(Keys::KeyDOWN, RPAD_DOWN);
    __SCAN_KEY(Keys::KeyLEFT, RPAD_LEFT);
    __SCAN_KEY(Keys::KeyRIGHT, RPAD_RIGHT);

    __SCAN_KEY(Keys::KeyF1, KEY_F1);
    __SCAN_KEY(Keys::KeyF2, KEY_F2);
    __SCAN_KEY(Keys::KeyF3, KEY_F3);
    __SCAN_KEY(Keys::KeyF4, KEY_F4);
    __SCAN_KEY(Keys::KeyF5, KEY_F5);
    __SCAN_KEY(Keys::KeyF6, KEY_F6);
    __SCAN_KEY(Keys::KeyF7, KEY_F7);
    __SCAN_KEY(Keys::KeyF8, KEY_F8);

#undef __SCAN_KEY
}
