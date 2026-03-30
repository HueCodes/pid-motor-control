#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

#define SSD1306_ADDR    0x3C
#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64

int  ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_update(void);
void ssd1306_set_pixel(uint8_t x, uint8_t y, uint8_t on);
void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str);
void ssd1306_draw_float(uint8_t x, uint8_t y, const char *label, float val);
void ssd1306_draw_bar(uint8_t x, uint8_t y, uint8_t width, float fraction);

#endif
