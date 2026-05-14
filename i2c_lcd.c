#include "i2c_lcd.h"

// PCF8574 -> LCD pin mapping used by ControllersTech:
// P0=RS, P1=RW, P2=EN, P3=Backlight, P4..P7 = D4..D7  :contentReference[oaicite:1]{index=1}
#define LCD_RS        0x01
#define LCD_EN        0x04
#define LCD_BACKLIGHT 0x08

static uint8_t bl_state = LCD_BACKLIGHT;

static void expander_write(uint8_t data)
{
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, &data, 1, 100);
}

static void lcd_send_internal(uint8_t byte, uint8_t rs)
{
    // Upper and lower nibbles on D4..D7 (P4..P7)
    uint8_t hi = (byte & 0xF0);
    uint8_t lo = (uint8_t)((byte << 4) & 0xF0);

    uint8_t mode = bl_state | (rs ? LCD_RS : 0x00);

    // Same idea as ControllersTech: pulse EN for each nibble using 4 bytes. :contentReference[oaicite:2]{index=2}
    uint8_t buf[4];
    buf[0] = hi | mode | LCD_EN;  // EN=1
    buf[1] = hi | mode;          // EN=0
    buf[2] = lo | mode | LCD_EN;  // EN=1
    buf[3] = lo | mode;          // EN=0

    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, buf, 4, 100);
}

void lcd_send_cmd(uint8_t cmd)
{
    lcd_send_internal(cmd, 0);
}

void lcd_send_data(uint8_t data)
{
    lcd_send_internal(data, 1);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);   // clear display
    HAL_Delay(2);
}

void lcd_put_cur(uint8_t row, uint8_t col)
{
    // ControllersTech method: row0 base 0x80, row1 base 0xC0. :contentReference[oaicite:3]{index=3}
    if (row == 0) lcd_send_cmd(0x80 | col);
    else          lcd_send_cmd(0xC0 | col);
}

void lcd_send_string(char *str)
{
    // ControllersTech: send char-by-char. :contentReference[oaicite:4]{index=4}
    while (*str) lcd_send_data((uint8_t)(*str++));
}

void lcd_backlight_on(void)
{
    bl_state = LCD_BACKLIGHT;
    expander_write(bl_state);  // update expander output
}

void lcd_backlight_off(void)
{
    bl_state = 0x00;
    expander_write(bl_state);
}
void lcd_clearLine(uint8_t row)
{
  lcd_put_cur(row, 0);
  lcd_send_string("                "); // 16 spaces for 16x2
}

void lcd_init(void)
{
    // Same init flow as ControllersTech (HD44780 4-bit init). :contentReference[oaicite:5]{index=5}
    HAL_Delay(50);

    lcd_send_cmd(0x30);
    HAL_Delay(5);

    lcd_send_cmd(0x30);
    HAL_Delay(1);

    lcd_send_cmd(0x30);
    HAL_Delay(10);

    lcd_send_cmd(0x20);   // set 4-bit mode
    HAL_Delay(10);

    lcd_send_cmd(0x28);   // function set: 4-bit, 2-line, 5x8
    HAL_Delay(1);

    lcd_send_cmd(0x08);   // display off
    HAL_Delay(1);

    lcd_clear();

    lcd_send_cmd(0x06);   // entry mode: increment, no shift
    HAL_Delay(1);

    lcd_send_cmd(0x0C);   // display on, cursor off, blink off
}
