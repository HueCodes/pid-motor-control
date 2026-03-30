/*
 * SSD1306 128x64 OLED driver over I2C
 * Displays: Kp/Ki/Kd values, setpoint, error, output bar graph
 */

#include "ssd1306.h"
#include "hal.h"
#include <string.h>
#include <stdio.h>

static uint8_t framebuf[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/* 5x7 font, ASCII 32-127 (space through ~) */
/* Minimal subset: digits, uppercase, lowercase, common symbols */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* space */
    {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
    {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x36,0x49,0x56,0x20,0x50}, /* & */
    {0x00,0x08,0x07,0x03,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x2A,0x1C,0x7F,0x1C,0x2A}, /* * */
    {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x80,0x70,0x30,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x00,0x60,0x60,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x72,0x49,0x49,0x49,0x46}, /* 2 */
    {0x21,0x41,0x49,0x4D,0x33}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x31}, /* 6 */
    {0x41,0x21,0x11,0x09,0x07}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x46,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x00,0x14,0x00,0x00}, /* : */
    {0x00,0x40,0x34,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41}, /* < */
    {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x00,0x41,0x22,0x14,0x08}, /* > */
    {0x02,0x01,0x59,0x09,0x06}, /* ? */
};

static void ssd1306_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    /* Manual I2C write of 2 bytes to command register */
    i2c_write_reg(SSD1306_ADDR, 0x00, cmd);
    (void)buf;
}

int ssd1306_init(void) {
    delay_ms(100); /* Wait for OLED power-up */

    ssd1306_cmd(0xAE); /* Display off */
    ssd1306_cmd(0xD5); ssd1306_cmd(0x80); /* Clock divide */
    ssd1306_cmd(0xA8); ssd1306_cmd(0x3F); /* Multiplex 64 */
    ssd1306_cmd(0xD3); ssd1306_cmd(0x00); /* Display offset 0 */
    ssd1306_cmd(0x40);                     /* Start line 0 */
    ssd1306_cmd(0x8D); ssd1306_cmd(0x14); /* Charge pump on */
    ssd1306_cmd(0x20); ssd1306_cmd(0x00); /* Horizontal addressing */
    ssd1306_cmd(0xA1);                     /* Segment remap */
    ssd1306_cmd(0xC8);                     /* COM scan decrement */
    ssd1306_cmd(0xDA); ssd1306_cmd(0x12); /* COM pins */
    ssd1306_cmd(0x81); ssd1306_cmd(0xCF); /* Contrast */
    ssd1306_cmd(0xD9); ssd1306_cmd(0xF1); /* Pre-charge */
    ssd1306_cmd(0xDB); ssd1306_cmd(0x40); /* VCOMH deselect */
    ssd1306_cmd(0xA4);                     /* Display from RAM */
    ssd1306_cmd(0xA6);                     /* Normal display */
    ssd1306_cmd(0xAF);                     /* Display on */

    ssd1306_clear();
    ssd1306_update();
    return 0;
}

void ssd1306_clear(void) {
    memset(framebuf, 0, sizeof(framebuf));
}

void ssd1306_update(void) {
    ssd1306_cmd(0x21); ssd1306_cmd(0); ssd1306_cmd(127); /* Column range */
    ssd1306_cmd(0x22); ssd1306_cmd(0); ssd1306_cmd(7);   /* Page range */

    /* Send framebuffer in chunks via I2C data writes */
    for (uint16_t i = 0; i < sizeof(framebuf); i++) {
        i2c_write_reg(SSD1306_ADDR, 0x40, framebuf[i]);
    }
}

void ssd1306_set_pixel(uint8_t x, uint8_t y, uint8_t on) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    uint16_t idx = x + (y / 8) * SSD1306_WIDTH;
    if (on)
        framebuf[idx] |= (1 << (y & 7));
    else
        framebuf[idx] &= ~(1 << (y & 7));
}

void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str) {
    while (*str && x + 5 < SSD1306_WIDTH) {
        uint8_t c = *str;
        if (c >= ' ' && c < ' ' + 32) {
            const uint8_t *glyph = font5x7[c - ' '];
            for (int col = 0; col < 5; col++) {
                for (int row = 0; row < 7; row++) {
                    ssd1306_set_pixel(x + col, y + row, (glyph[col] >> row) & 1);
                }
            }
        }
        x += 6;
        str++;
    }
}

void ssd1306_draw_float(uint8_t x, uint8_t y, const char *label, float val) {
    char buf[22];
    snprintf(buf, sizeof(buf), "%s%.2f", label, (double)val);
    ssd1306_draw_string(x, y, buf);
}

void ssd1306_draw_bar(uint8_t x, uint8_t y, uint8_t width, float fraction) {
    if (fraction < 0) fraction = 0;
    if (fraction > 1.0f) fraction = 1.0f;
    uint8_t filled = (uint8_t)(fraction * width);

    for (uint8_t i = 0; i < width; i++) {
        for (uint8_t r = 0; r < 6; r++) {
            ssd1306_set_pixel(x + i, y + r, i < filled);
        }
    }
}
