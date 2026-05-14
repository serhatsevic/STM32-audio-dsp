#include "stm32f4_dac3100.h"

//Pages for register
#define TLV_REG_PAGE_SEL        0x00
#define TLV_REG_SW_RESET        0x01

// Page 0 clocking for  PLL
#define TLV_REG_CLKMUX          0x04
#define TLV_REG_PLL_P_R         0x05
#define TLV_REG_PLL_J           0x06
#define TLV_REG_PLL_D_MSB       0x07
#define TLV_REG_PLL_D_LSB       0x08

#define TLV_REG_NDAC            0x0B
#define TLV_REG_MDAC            0x0C
#define TLV_REG_DOSR_MSB        0x0D
#define TLV_REG_DOSR_LSB        0x0E

#define TLV_REG_AUDIO_IFACE     0x1B  // I2S format + word length + master/slave bits

#define TLV_REG_PRB_SELECT      0x3C
#define TLV_REG_DAC_PWR         0x3F
#define TLV_REG_DAC_DIG_VOL     0x40  // Digital volume or mute

// Page 1 analog routing/headphone (script-like defaults)
#define TLV_P1_HPDAC_ROUTE      0x23
#define TLV_P1_HPL_VOL          0x28
#define TLV_P1_HPR_VOL          0x29
#define TLV_P1_HP_DRIVER_PWR    0x1F
#define TLV_P1_DEPOP            0x21
#define TLV_P1_HP_ANALOG_VOL_L  0x24
#define TLV_P1_HP_ANALOG_VOL_R  0x25

// Page 8
#define TLV_P8_CFG              0x01

static HAL_StatusTypeDef tlv_set_page(tlv320dac3100_t *dev, uint8_t page)    //sets page
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr_8bit,
                             TLV_REG_PAGE_SEL, I2C_MEMADD_SIZE_8BIT,
                             &page, 1, 50);
}

HAL_StatusTypeDef TLV320DAC3100_WriteReg(tlv320dac3100_t *dev, uint8_t page, uint8_t reg, uint8_t val)   //Writes data to register
{
    HAL_StatusTypeDef st = tlv_set_page(dev, page);
    if (st != HAL_OK)
        return st; //setting page doesn't work, return error so read/write doesn't continue
    return HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr_8bit,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, 50);
}

HAL_StatusTypeDef TLV320DAC3100_ReadReg(tlv320dac3100_t *dev, uint8_t page, uint8_t reg, uint8_t *val_out)   //Reads data from register
{
    if (!val_out)
        return HAL_ERROR; //If no output value, return error code
    HAL_StatusTypeDef st = tlv_set_page(dev, page); //Check if page is set
    if (st != HAL_OK)
        return st; //setting page doesn't work, return error so read/write doesn't continue
    return HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr_8bit,
                            reg, I2C_MEMADD_SIZE_8BIT,
                            val_out, 1, 50);
}

static HAL_StatusTypeDef tlv_write16(tlv320dac3100_t *dev, uint8_t page, uint8_t reg_msb, uint16_t value)
{
    uint8_t msb = (uint8_t)((value >> 8) & 0xFF); //Divide 16 bit data into
    uint8_t lsb = (uint8_t)(value & 0xFF);
    HAL_StatusTypeDef st = TLV320DAC3100_WriteReg(dev, page, reg_msb, msb);
    if (st != HAL_OK)
        return st;
    return TLV320DAC3100_WriteReg(dev, page, (uint8_t)(reg_msb + 1), lsb);
}

/* ---- Public API ---- */

HAL_StatusTypeDef TLV320DAC3100_SetHpVolRaw(tlv320dac3100_t *dev, uint8_t regval)
{
    HAL_StatusTypeDef st = TLV320DAC3100_WriteReg(dev, 1, TLV_P1_HPL_VOL, regval);
    if (st != HAL_OK)
        return st;
    return TLV320DAC3100_WriteReg(dev, 1, TLV_P1_HPR_VOL, regval);
}

/*
  Mute/unmute headphone output by changing HPL/HPR volume registers.

    mute= set volume reg to 0x00
    unmute= restore cfg.hp_vol_reg
*/
HAL_StatusTypeDef TLV320DAC3100_MuteHp(tlv320dac3100_t *dev, bool mute)
{
    return TLV320DAC3100_SetHpVolRaw(dev, mute ? 0x00 : dev->cfg.hp_vol_reg);
} //Mute headphone

static HAL_StatusTypeDef tlv_config_clocks_48k(tlv320dac3100_t *dev)
{
    uint8_t clkmux = (dev->cfg.pll_input == TLV320_PLL_INPUT_MCLK) ? 0x03 : 0x07;
    HAL_StatusTypeDef st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_CLKMUX, clkmux);
    if (st != HAL_OK)
        return st;


    st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_PLL_J, 0x07);
    if (st != HAL_OK)
        return st;
    st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_PLL_D_MSB, 0x00);
    if (st != HAL_OK)
        return st;
    st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_PLL_D_LSB, 0x00);
    if (st != HAL_OK)
        return st;

    // Power up PLL, P=1, R=1
    st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_PLL_P_R, 0x91);
    if (st != HAL_OK)
        return st;

    // NDAC=7, MDAC=2, DOSR=128 ==> Fs = (MCLK*7)/(7*2*128) = MCLK/256
    st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_NDAC, (uint8_t)(0x80 | 7));
    if (st != HAL_OK)
        return st;

    st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_MDAC, (uint8_t)(0x80 | 2));
    if (st != HAL_OK)
        return st;

    st = tlv_write16(dev, 0, TLV_REG_DOSR_MSB, 128);
    return st;
}


static HAL_StatusTypeDef tlv_config_i2s_16bit_slave(tlv320dac3100_t *dev)
{

    return TLV320DAC3100_WriteReg(dev, 0, TLV_REG_AUDIO_IFACE, 0x00);
}

static HAL_StatusTypeDef tlv_power_and_route_headphone(tlv320dac3100_t *dev)
{
    HAL_StatusTypeDef st;

    /* Select a processing block (script-style default used widely) */
    st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_PRB_SELECT, 0x0B);
    if (st != HAL_OK)
        return st;

    /* Page 8 small config used in many reference sequences */
    st = TLV320DAC3100_WriteReg(dev, 8, TLV_P8_CFG, 0x04);
    if (st != HAL_OK)
        return st;

    /* Route DAC to headphone path on page 1 */
    st = TLV320DAC3100_WriteReg(dev, 1, TLV_P1_DEPOP, 0x4E);
    if (st != HAL_OK)
        return st;

    /* LDAC->HPL, RDAC->HPR (script-style) */
    st = TLV320DAC3100_WriteReg(dev, 1, TLV_P1_HPDAC_ROUTE, 0x44);
    if (st != HAL_OK)
        return st;

    /* Power up headphone drivers (script-style) */
    st = TLV320DAC3100_WriteReg(dev, 1, TLV_P1_HP_DRIVER_PWR, 0xC2);
    if (st != HAL_OK)
        return st;

    /* Enable analog headphone volume controls (script-style) */
    st = TLV320DAC3100_WriteReg(dev, 1, TLV_P1_HP_ANALOG_VOL_L, 0x80);
    if (st != HAL_OK)
        return st;
    st = TLV320DAC3100_WriteReg(dev, 1, TLV_P1_HP_ANALOG_VOL_R, 0x80);
    if (st != HAL_OK)
        return st;

    /* Set HP volume (raw reg value) */
    st = TLV320DAC3100_SetHpVolRaw(dev, dev->cfg.hp_vol_reg);
    if (st != HAL_OK)
        return st;

    /* Power up DAC L/R (script-style) */
    st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_DAC_PWR, 0xD4);
    if (st != HAL_OK)
        return st;

    /* Unmute / set digital volume default (script-style 0x00) */
    st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_DAC_DIG_VOL, 0x00);
    return st;
}

HAL_StatusTypeDef TLV320DAC3100_Init(tlv320dac3100_t *dev,
                                     I2C_HandleTypeDef *hi2c,
                                     const tlv320dac3100_cfg_t *cfg)
{
    if (!dev || !hi2c || !cfg)
        return HAL_ERROR;

    dev->hi2c = hi2c;
    dev->cfg = *cfg;
    dev->i2c_addr_8bit = (uint16_t)(cfg->i2c_addr_7bit << 1);

    /* Page 0 + software reset */
    HAL_StatusTypeDef st = TLV320DAC3100_WriteReg(dev, 0, TLV_REG_SW_RESET, 0x01);
    if (st != HAL_OK)
        return st;
    HAL_Delay(5);

    /* Clocking (48k profile) */
    if (cfg->fs_hz == 48000)
    {
        st = tlv_config_clocks_48k(dev);
        if (st != HAL_OK)
            return st;
    }
    else
    {
        /* Only 48k profile included to keep it minimal */
        return HAL_ERROR;
    }

    /* I2S 16-bit, codec slave */
    st = tlv_config_i2s_16bit_slave(dev);
    if (st != HAL_OK)
        return st;

    /* Power/routing */
    st = tlv_power_and_route_headphone(dev);
    return st;
}
