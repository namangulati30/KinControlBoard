#ifndef _LED_DRIVER_SK6812
#define _LED_DRIVER_SK6812

#include <stdint.h>
#include "main.h"



void loopledSetRgb(uint8_t group, uint8_t r, int8_t g,int8_t b,int8_t w, uint8_t n);
void loopblink(uint8_t group, uint8_t r, int8_t g,int8_t b,int8_t w, uint8_t n, uint16_t time);
void ledinit(void);
void ColorChange(void);
void BlinkLed(uint8_t r, uint8_t g, uint8_t b);
void Breath(uint8_t r, uint8_t g, uint8_t b);
uint32_t hsl_to_rgb(uint8_t h, uint8_t s, uint8_t l);
void led_set_RGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void led_set_RGBW(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void led_set_all_RGB(uint8_t r, uint8_t g, uint8_t b);
void led_set_all_RGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void led_render(void);
extern float bred,bgreen,bblue;
#endif
