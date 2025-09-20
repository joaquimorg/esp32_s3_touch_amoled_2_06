#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "esp_event.h"
#include "driver/i2c_master.h"
#include "bsp/esp32_s3_touch_amoled_2_06.h"
#include "axp2101.h"

static const char *TAG = "AXP2101";

static i2c_master_dev_handle_t s_pmu_dev = NULL;
static uint8_t s_pmu_addr = AXP2101_I2C_ADDR_PRIMARY;
static bool s_ready = false;
static TaskHandle_t s_mon_task = NULL;
static uint32_t s_poll_ms = 500;
static bsp_power_event_cb_t s_cb = NULL;
static void *s_cb_user = NULL;

static esp_err_t pmu_add_on_bus(void)
{
    if (s_pmu_dev) return ESP_OK;
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init");
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) return ESP_FAIL;

    const uint8_t candidates[] = { AXP2101_I2C_ADDR_PRIMARY, AXP2101_I2C_ADDR_ALTERNATE };
    esp_err_t perr = ESP_FAIL;
    for (size_t i = 0; i < sizeof(candidates); ++i) {
        perr = i2c_master_probe(bus, candidates[i], I2C_MASTER_TIMEOUT_MS);
        if (perr == ESP_OK) {
            s_pmu_addr = candidates[i];
            break;
        }
    }
    ESP_RETURN_ON_ERROR(perr, TAG, "probe failed");

    // Resolve I2C speed
#ifdef CONFIG_BSP_I2C_CLK_SPEED_HZ
    const uint32_t i2c_speed = CONFIG_BSP_I2C_CLK_SPEED_HZ;
#elif defined(CONFIG_I2C_MASTER_FREQUENCY)
    const uint32_t i2c_speed = CONFIG_I2C_MASTER_FREQUENCY;
#else
    const uint32_t i2c_speed = 400000;
#endif

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_pmu_addr,
        .scl_speed_hz = i2c_speed,
        .scl_wait_us = 0,
        .flags = { .disable_ack_check = 0 },
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &s_pmu_dev), TAG, "add dev");
    ESP_LOGI(TAG, "AXP210x detected at 0x%02X, I2C %lu Hz", s_pmu_addr, (unsigned long)i2c_speed);
    return ESP_OK;
}

static esp_err_t pmu_config_adc_and_gauge(void)
{
    // Enable ADC main gate + channels: Temp(4), Sys(3), VBUS(2), VBAT(0)
    ESP_RETURN_ON_ERROR(axp2101_set_bits(s_pmu_dev, AXP2101_REG_ADC_CH_CTRL, (1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<0)), TAG, "adc ch");
    // Disable TS pin measurement (bit1)
    ESP_RETURN_ON_ERROR(axp2101_clear_bits(s_pmu_dev, AXP2101_REG_ADC_CH_CTRL, (1<<1)), TAG, "ts off");
    // Enable battery detection (bit0 of 0x68)
    ESP_RETURN_ON_ERROR(axp2101_set_bits(s_pmu_dev, AXP2101_REG_BAT_DET_CTRL, (1<<0)), TAG, "bat det");
    // Enable fuel gauge (bit3 of 0x18)
    ESP_RETURN_ON_ERROR(axp2101_set_bits(s_pmu_dev, AXP2101_REG_CHG_GAUGE_WDT, (1<<3)), TAG, "gauge");

    // Enable PWR key short-press IRQ based on Kconfig selection
#ifdef CONFIG_BSP_POWER_PKEY_IRQ_STATUS_REG_CH
    uint8_t irq_reg_ch = CONFIG_BSP_POWER_PKEY_IRQ_STATUS_REG_CH; // 1..3
#else
    uint8_t irq_reg_ch = 2; // default INTSTS2/INTEN2
#endif
#ifdef CONFIG_BSP_POWER_PKEY_SHORT_BIT
    uint8_t pkey_bit = CONFIG_BSP_POWER_PKEY_SHORT_BIT; // 0..7
#else
    uint8_t pkey_bit = 1; // sensible default
#endif
    uint8_t inten_addr = (uint8_t)(AXP2101_REG_INTEN1 + (irq_reg_ch - 1));
    ESP_RETURN_ON_ERROR(axp2101_set_bits(s_pmu_dev, inten_addr, (1u << pkey_bit)), TAG, "pkey inten");

    // Clear any pending IRQ status for that bit
    uint8_t intsts_addr = (uint8_t)(AXP2101_REG_INTSTS1 + (irq_reg_ch - 1));
    uint8_t clr_buf[2] = { intsts_addr, (uint8_t)(1u << pkey_bit) };
    (void)i2c_master_transmit(s_pmu_dev, clr_buf, sizeof(clr_buf), I2C_MASTER_TIMEOUT_MS);
    return ESP_OK;
}

static uint8_t pmu_read_u8(uint8_t reg)
{
    uint8_t v = 0;
    (void)i2c_master_transmit_receive(s_pmu_dev, &reg, 1, &v, 1, I2C_MASTER_TIMEOUT_MS);
    return v;
}

static esp_err_t pmu_set_aldo_state(uint8_t bit, bool enable)
{
    if (!s_ready || !s_pmu_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t mask = (uint8_t)(1u << bit);
    if (enable) {
        return axp2101_set_bits(s_pmu_dev, AXP2101_REG_LDO_ONOFF_CTRL0, mask);
    }
    return axp2101_clear_bits(s_pmu_dev, AXP2101_REG_LDO_ONOFF_CTRL0, mask);
}

static void pmu_emit_evt(bsp_power_event_t evt)
{
    if (!s_ready) return;
    if (s_cb) s_cb(evt, s_cb_user);
    bsp_power_event_payload_t pl = {
        .battery_percent = bsp_power_get_battery_percent(),
        .charging = bsp_power_is_charging(),
        .vbus_in = bsp_power_is_vbus_in(),
        .charger_status = 0,
    };
    (void)esp_event_post(BSP_POWER_EVENT_BASE, evt, &pl, sizeof(pl), 0);
}

static void pmu_mon_task(void *arg)
{
    bool prev_vbus = bsp_power_is_vbus_in();
    bool prev_chg = bsp_power_is_charging();
    for (;;) {
        bool vbus = bsp_power_is_vbus_in();
        if (vbus != prev_vbus) {
            pmu_emit_evt(vbus ? BSP_POWER_EVT_VBUS_INSERT : BSP_POWER_EVT_VBUS_REMOVE);
            prev_vbus = vbus;
        }
        bool chg = bsp_power_is_charging();
        if (!prev_chg && chg) {
            pmu_emit_evt(BSP_POWER_EVT_CHG_START);
        } else if (prev_chg && !chg) {
            pmu_emit_evt(BSP_POWER_EVT_CHG_DONE);
        }
        prev_chg = chg;
        vTaskDelay(pdMS_TO_TICKS(s_poll_ms));
    }
}

esp_err_t bsp_power_init(void)
{
    ESP_RETURN_ON_ERROR(pmu_add_on_bus(), TAG, "dev");
    // Read IC type for info
    uint8_t ic = pmu_read_u8(AXP2101_REG_IC_TYPE);
    ESP_LOGI(TAG, "AXP210x IC_TYPE(0x03)=0x%02X", (int)ic);
    ESP_RETURN_ON_ERROR(pmu_config_adc_and_gauge(), TAG, "cfg");
    s_ready = true;
    // Log initial values
    ESP_LOGI(TAG, "AXP2101 initialized: Batt %d%%, Vbat %dmV, Vbus %dmV, Vsys %dmV",
             bsp_power_get_battery_percent(), bsp_power_get_batt_voltage_mv(),
             bsp_power_get_vbus_voltage_mv(), bsp_power_get_system_voltage_mv());
    // Start monitor task by default
    bsp_power_start_monitor(250);
    return ESP_OK;
}

void bsp_power_isr_handler(void)
{
    // No dedicated IRQ pin; polling task handles events.
}

int bsp_power_get_battery_percent(void)
{
    if (!s_ready) return -1;
    return (int)pmu_read_u8(AXP2101_REG_BAT_PERCENT);
}

int bsp_power_get_batt_voltage_mv(void)
{
    if (!s_ready) return -1;
    uint16_t raw = axp2101_read_h5_l8(s_pmu_dev, AXP2101_REG_ADC_RES0, AXP2101_REG_ADC_RES1);
    return (int)raw;
}

int bsp_power_get_vbus_voltage_mv(void)
{
    if (!s_ready) return -1;
    uint16_t raw = axp2101_read_h6_l8(s_pmu_dev, AXP2101_REG_ADC_RES4, AXP2101_REG_ADC_RES5);
    return (int)raw;
}

int bsp_power_get_system_voltage_mv(void)
{
    if (!s_ready) return -1;
    uint16_t raw = axp2101_read_h6_l8(s_pmu_dev, AXP2101_REG_ADC_RES6, AXP2101_REG_ADC_RES7);
    return (int)raw;
}

float bsp_power_get_temperature_c(void)
{
    if (!s_ready) return 0.0f;
    uint16_t raw = axp2101_read_h6_l8(s_pmu_dev, AXP2101_REG_ADC_RES8, AXP2101_REG_ADC_RES9);
    return AXP2101_TEMP_C_FROM_RAW(raw);
}

bool bsp_power_is_battery_connected(void)
{
    if (!s_ready) return false;
    // Fallback: VBAT ADC non-zero considered connected
    uint8_t reg = AXP2101_REG_STATUS1, v = 0;
    if (i2c_master_transmit_receive(s_pmu_dev, &reg, 1, &v, 1, I2C_MASTER_TIMEOUT_MS) == ESP_OK) {
        if (v & (1<<3)) return true;
    }
    return bsp_power_get_batt_voltage_mv() > 0;
}

bool bsp_power_is_charging(void)
{
    if (!s_ready) return false;
    uint8_t reg = AXP2101_REG_STATUS2, v = 0;
    if (i2c_master_transmit_receive(s_pmu_dev, &reg, 1, &v, 1, I2C_MASTER_TIMEOUT_MS) != ESP_OK) return false;
    return ((v >> 5) & 0x07) == 0x01;
}

bool bsp_power_is_vbus_in(void)
{
    if (!s_ready) return false;
    uint8_t r1 = AXP2101_REG_STATUS1, v1 = 0;
    if (i2c_master_transmit_receive(s_pmu_dev, &r1, 1, &v1, 1, I2C_MASTER_TIMEOUT_MS) != ESP_OK) return false;
    return (v1 & (1<<5)) != 0; // VBUS good
}

// Minimal rail control stubs to preserve API
esp_err_t bsp_power_set_dc1_voltage_mv(uint16_t mv)
{
    return s_ready ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t bsp_power_enable_dc1(bool enable)
{
    return s_ready ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t bsp_power_set_aldo1_voltage_mv(uint16_t mv)
{
    return s_ready ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t bsp_power_enable_aldo1(bool enable)
{
    return pmu_set_aldo_state(0, enable);
}

esp_err_t bsp_power_set_aldo2_voltage_mv(uint16_t mv)
{
    return s_ready ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t bsp_power_enable_aldo2(bool enable)
{
    return pmu_set_aldo_state(1, enable);
}

esp_err_t bsp_power_enable_aldo3(bool enable)
{
    return pmu_set_aldo_state(2, enable);
}

esp_err_t bsp_power_enable_aldo4(bool enable)
{
    return pmu_set_aldo_state(3, enable);
}

bool bsp_power_poll_pwr_button_short(void)
{
    if (!s_ready) return false;
#ifdef CONFIG_BSP_POWER_PKEY_IRQ_STATUS_REG_CH
    uint8_t irq_reg_ch = CONFIG_BSP_POWER_PKEY_IRQ_STATUS_REG_CH; // 1..3
#else
    uint8_t irq_reg_ch = 2; // default INTSTS2
#endif
#ifdef CONFIG_BSP_POWER_PKEY_SHORT_BIT
    uint8_t pkey_bit = CONFIG_BSP_POWER_PKEY_SHORT_BIT; // 0..7
#else
    uint8_t pkey_bit = 1; // default bit
#endif
    uint8_t reg = (uint8_t)(AXP2101_REG_INTSTS1 + (irq_reg_ch - 1));
    uint8_t v = 0;
    if (i2c_master_transmit_receive(s_pmu_dev, &reg, 1, &v, 1, I2C_MASTER_TIMEOUT_MS) != ESP_OK) {
        return false;
    }
    uint8_t mask = (uint8_t)(1u << pkey_bit);
    bool pressed = (v & mask) != 0;
    if (pressed) {
        uint8_t clr_buf[2] = { reg, mask };
        (void)i2c_master_transmit(s_pmu_dev, clr_buf, sizeof(clr_buf), I2C_MASTER_TIMEOUT_MS);
    }
    return pressed;
}

void bsp_power_register_event_cb(bsp_power_event_cb_t cb, void *user)
{
    s_cb = cb;
    s_cb_user = user;
}

void bsp_power_start_monitor(uint32_t ms)
{
    if (!s_ready) return;
    if (ms) s_poll_ms = ms;
    if (s_mon_task) return;
    xTaskCreate(pmu_mon_task, "pmu_mon", 3072, NULL, 3, &s_mon_task);
}
