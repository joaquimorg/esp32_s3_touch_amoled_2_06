#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// AXP2101 basic register map (subset used)
#define AXP2101_I2C_ADDR_PRIMARY     0x34
#define AXP2101_I2C_ADDR_ALTERNATE   0x35

#define AXP2101_REG_STATUS1          0x00
#define AXP2101_REG_STATUS2          0x01
#define AXP2101_REG_IC_TYPE          0x03

#define AXP2101_REG_ADC_CH_CTRL      0x30
#define AXP2101_REG_ADC_RES0         0x34
#define AXP2101_REG_ADC_RES1         0x35
#define AXP2101_REG_ADC_RES2         0x36
#define AXP2101_REG_ADC_RES3         0x37
#define AXP2101_REG_ADC_RES4         0x38
#define AXP2101_REG_ADC_RES5         0x39
#define AXP2101_REG_ADC_RES6         0x3A
#define AXP2101_REG_ADC_RES7         0x3B
#define AXP2101_REG_ADC_RES8         0x3C
#define AXP2101_REG_ADC_RES9         0x3D

#define AXP2101_REG_INTEN1           0x40
#define AXP2101_REG_INTEN2           0x41
#define AXP2101_REG_INTEN3           0x42
#define AXP2101_REG_INTSTS1          0x48
#define AXP2101_REG_INTSTS2          0x49
#define AXP2101_REG_INTSTS3          0x4A

#define AXP2101_REG_LDO_ONOFF_CTRL0  0x90
#define AXP2101_REG_LDO_ONOFF_CTRL1  0x91

#define AXP2101_REG_BAT_DET_CTRL     0x68
#define AXP2101_REG_BAT_PERCENT      0xA4
#define AXP2101_REG_CHG_GAUGE_WDT    0x18

// ADC temperature conversion (from XPowersLib)
#define AXP2101_TEMP_C_FROM_RAW(raw)  (22.0f + ((7274.0f - (float)(raw)) / 20.0f))

// Bit helpers
static inline esp_err_t axp2101_set_bits(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t mask)
{
    uint8_t v;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, &v, 1, 1000);
    if (err != ESP_OK) return err;
    uint8_t buf[2] = { reg, (uint8_t)(v | mask) };
    return i2c_master_transmit(dev, buf, sizeof(buf), 1000);
}

static inline esp_err_t axp2101_clear_bits(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t mask)
{
    uint8_t v;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, &v, 1, 1000);
    if (err != ESP_OK) return err;
    uint8_t buf[2] = { reg, (uint8_t)(v & (uint8_t)(~mask)) };
    return i2c_master_transmit(dev, buf, sizeof(buf), 1000);
}

static inline uint16_t axp2101_read_h5_l8(i2c_master_dev_handle_t dev, uint8_t reg_h, uint8_t reg_l)
{
    uint8_t rh = reg_h, rl = reg_l, vh = 0, vl = 0;
    if (i2c_master_transmit_receive(dev, &rh, 1, &vh, 1, 1000) != ESP_OK) return 0;
    if (i2c_master_transmit_receive(dev, &rl, 1, &vl, 1, 1000) != ESP_OK) return 0;
    return (uint16_t)(((vh & 0x1F) << 8) | vl);
}

static inline uint16_t axp2101_read_h6_l8(i2c_master_dev_handle_t dev, uint8_t reg_h, uint8_t reg_l)
{
    uint8_t rh = reg_h, rl = reg_l, vh = 0, vl = 0;
    if (i2c_master_transmit_receive(dev, &rh, 1, &vh, 1, 1000) != ESP_OK) return 0;
    if (i2c_master_transmit_receive(dev, &rl, 1, &vl, 1, 1000) != ESP_OK) return 0;
    return (uint16_t)(((vh & 0x3F) << 8) | vl);
}

#ifdef __cplusplus
}
#endif

