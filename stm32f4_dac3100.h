#ifndef TLV320DAC3100_H
#define TLV320DAC3100_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/*
  TLV320DAC3100 (Adafruit breakout) minimal driver for STM32 HAL.

  Assumptions (default init):
  - STM32 provides I2S clocks (BCLK + LRCLK/WS), codec is I2S SLAVE.
  - 16-bit I2S Philips format.
  - 48 kHz nominal.
  - Prefer MCLK present (256*Fs). If not, optional BCLK-as-PLL-input mode.

  Wiring reminder (STM32 I2S2 example):
    PB10 = I2S2_CK  -> DAC BCLK
    PB12 = I2S2_WS  -> DAC LRCLK/WS
    PC3  = I2S2_SD  -> DAC DIN
    PA6  = I2S2_MCK -> DAC MCK (recommended)
    I2C SDA/SCL -> DAC SDA/SCL
    GND common
*/

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  TLV320_PLL_INPUT_MCLK = 0,   // Recommended if you connected MCK
  TLV320_PLL_INPUT_BCLK = 1    // Use if you did NOT connect MCK (may require tweaks)
} tlv320_pll_input_t;

typedef struct {
  uint8_t i2c_addr_7bit;        // default 0x18 for Adafruit board
  tlv320_pll_input_t pll_input; // MCLK or BCLK
  uint32_t fs_hz;               // e.g. 48000
  uint8_t hp_vol_reg;           // raw reg value written to HPL/HPR volume regs (page1)
                               // example "0 dB unmute" from TI scripts often ~0x06
} tlv320dac3100_cfg_t;

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint16_t i2c_addr_8bit; // HAL expects 8-bit address (7-bit << 1)
  tlv320dac3100_cfg_t cfg;
} tlv320dac3100_t;

/* Core API */
HAL_StatusTypeDef TLV320DAC3100_Init(tlv320dac3100_t *dev,
                                     I2C_HandleTypeDef *hi2c,
                                     const tlv320dac3100_cfg_t *cfg);

HAL_StatusTypeDef TLV320DAC3100_SetHpVolRaw(tlv320dac3100_t *dev, uint8_t regval);
HAL_StatusTypeDef TLV320DAC3100_MuteHp(tlv320dac3100_t *dev, bool mute);

/* Optional: raw register access (useful for debugging) */
HAL_StatusTypeDef TLV320DAC3100_WriteReg(tlv320dac3100_t *dev, uint8_t page, uint8_t reg, uint8_t val);
HAL_StatusTypeDef TLV320DAC3100_ReadReg(tlv320dac3100_t *dev, uint8_t page, uint8_t reg, uint8_t *val_out);

#ifdef __cplusplus
}
#endif

#endif // TLV320DAC3100_H
