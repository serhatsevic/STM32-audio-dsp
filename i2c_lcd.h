#ifndef __I2C_LCD_H__
#define __I2C_LCD_H__

#include "main.h"   // gives HAL + I2C_HandleTypeDef (CubeIDE projects)


#define LCD_I2C_ADDR   (0x27 << 1)   // = 0x4E

// If your I2C peripheral handle is not hi2c1, change this extern accordingly.
extern I2C_HandleTypeDef hi2c1;

void lcd_init(void);
void lcd_clear(void);

void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);

void lcd_put_cur(uint8_t row, uint8_t col);
void lcd_send_string(char *str);

void lcd_clearLine(uint8_t row);

// Optional helpers
void lcd_backlight_on(void);
void lcd_backlight_off(void);

#endif
