// C++ implementation of AXP2101 power integration for the BSP
#include <stdio.h>
#include <cstring>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "bsp/esp32_s3_touch_amoled_2_06.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

static const char *TAG = "AXP2101";

static XPowersPMU PMU;
static i2c_master_dev_handle_t s_pmu_i2c_dev = nullptr;
static bool s_pmu_ready = false;

static esp_err_t pmu_i2c_dev_init()
{
    if (s_pmu_i2c_dev) {
        return ESP_OK;
    }
    // Ensure I2C bus exists
    esp_err_t err_init = bsp_i2c_init();
    if (err_init != ESP_OK) {
        return err_init;
    }
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    if (!bus) {
        return ESP_FAIL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP2101_SLAVE_ADDRESS,
        .scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ,
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, &s_pmu_i2c_dev);
}

extern "C" int pmu_register_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    (void)devAddr; // Address is set in device handle
    if (pmu_i2c_dev_init() != ESP_OK) {
        return -1;
    }
    esp_err_t err = i2c_master_transmit_receive(s_pmu_i2c_dev, &regAddr, 1, data, len, 1000);
    return (err == ESP_OK) ? 0 : -1;
}

extern "C" int pmu_register_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    (void)devAddr; // Address is set in device handle
    if (pmu_i2c_dev_init() != ESP_OK) {
        return -1;
    }
    uint8_t tmp[1 + 32];
    if (len > 32) {
        len = 32;
    }
    tmp[0] = regAddr;
    if (len > 0 && data != nullptr) {
        memcpy(&tmp[1], data, len);
    }
    esp_err_t err = i2c_master_transmit(s_pmu_i2c_dev, tmp, 1 + len, 1000);
    return (err == ESP_OK) ? 0 : -1;
}

static esp_err_t pmu_configure_defaults()
{
    // Turn off unused power channels first
    /*PMU.disableDC2();
    PMU.disableDC3();
    PMU.disableDC4();
    PMU.disableDC5();

    PMU.disableALDO1();
    PMU.disableALDO2();
    PMU.disableALDO3();
    PMU.disableALDO4();
    PMU.disableBLDO1();
    PMU.disableBLDO2();

    PMU.disableCPUSLDO();
    PMU.disableDLDO1();
    PMU.disableDLDO2();*/

    // Board rails: set 3.3V where applicable
    //PMU.setDC1Voltage(3300);
    //PMU.enableDC1();

    // Panel 3.3V rail on this board is ALDO2
    //PMU.setALDO2Voltage(3300);
    //PMU.enableALDO2();

    // Measurements
    PMU.clearIrqStatus();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();
    PMU.enableTemperatureMeasure();

    // Disable TS pin measurement (if NTC not present)
    PMU.disableTSPinMeasure();

    // IRQ configuration
    PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    PMU.clearIrqStatus();
    PMU.enableIRQ(
        XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ |
        XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |
        XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ |
        XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ);

    // Charging parameters
    PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_400MA);
    PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
    PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

    ESP_LOGI(TAG, "AXP2101 initialized: Batt %d%%, Vbat %dmV, Vbus %dmV, Vsys %dmV",
             PMU.getBatteryPercent(), PMU.getBattVoltage(), PMU.getVbusVoltage(), PMU.getSystemVoltage());
    return ESP_OK;
}

extern "C" esp_err_t bsp_power_init(void)
{
    if (PMU.begin(AXP2101_SLAVE_ADDRESS, pmu_register_read, pmu_register_write_byte)) {
        ESP_LOGI(TAG, "Init PMU SUCCESS!");
        s_pmu_ready = true;
        return pmu_configure_defaults();
    }
    ESP_LOGE(TAG, "Init PMU FAILED!");
    return ESP_FAIL;
}

extern "C" void pmu_isr_handler(void)
{
    // Get PMU Interrupt Status Register
    PMU.getIrqStatus();

    ESP_LOGI(TAG, "Power Temperature: %.2f°C", PMU.getTemperature());
    ESP_LOGI(TAG, "isCharging: %s", PMU.isCharging() ? "YES" : "NO");
    ESP_LOGI(TAG, "isDischarge: %s", PMU.isDischarge() ? "YES" : "NO");
    ESP_LOGI(TAG, "isStandby: %s", PMU.isStandby() ? "YES" : "NO");
    ESP_LOGI(TAG, "isVbusIn: %s", PMU.isVbusIn() ? "YES" : "NO");
    ESP_LOGI(TAG, "isVbusGood: %s", PMU.isVbusGood() ? "YES" : "NO");

    uint8_t charge_status = PMU.getChargerStatus();
    switch (charge_status) {
        case XPOWERS_AXP2101_CHG_TRI_STATE: ESP_LOGI(TAG, "Charger Status: tri_charge"); break;
        case XPOWERS_AXP2101_CHG_PRE_STATE: ESP_LOGI(TAG, "Charger Status: pre_charge"); break;
        case XPOWERS_AXP2101_CHG_CC_STATE:  ESP_LOGI(TAG, "Charger Status: constant charge"); break;
        case XPOWERS_AXP2101_CHG_CV_STATE:  ESP_LOGI(TAG, "Charger Status: constant voltage"); break;
        case XPOWERS_AXP2101_CHG_DONE_STATE:ESP_LOGI(TAG, "Charger Status: charge done"); break;
        case XPOWERS_AXP2101_CHG_STOP_STATE:ESP_LOGI(TAG, "Charger Status: not charge"); break;
        default: break;
    }

    ESP_LOGI(TAG, "Vbat: %d mV, Vbus: %d mV, Vsys: %d mV", PMU.getBattVoltage(), PMU.getVbusVoltage(), PMU.getSystemVoltage());
    if (PMU.isBatteryConnect()) {
        ESP_LOGI(TAG, "Battery: %d %%", PMU.getBatteryPercent());
    }
    // Clear PMU Interrupt Status Register
    PMU.clearIrqStatus();
}

extern "C" void bsp_power_isr_handler(void)
{
    pmu_isr_handler();
}

// Simple getters
extern "C" int bsp_power_get_battery_percent(void) { return s_pmu_ready ? PMU.getBatteryPercent() : -1; }
extern "C" int bsp_power_get_batt_voltage_mv(void) { return s_pmu_ready ? PMU.getBattVoltage() : -1; }
extern "C" int bsp_power_get_vbus_voltage_mv(void) { return s_pmu_ready ? PMU.getVbusVoltage() : -1; }
extern "C" int bsp_power_get_system_voltage_mv(void) { return s_pmu_ready ? PMU.getSystemVoltage() : -1; }
extern "C" float bsp_power_get_temperature_c(void) { return s_pmu_ready ? PMU.getTemperature() : 0.0f; }
extern "C" bool bsp_power_is_battery_connected(void) { return s_pmu_ready ? PMU.isBatteryConnect() : false; }
extern "C" bool bsp_power_is_charging(void) { return s_pmu_ready ? PMU.isCharging() : false; }

// Basic rail control for common rails used on this board
extern "C" esp_err_t bsp_power_set_dc1_voltage_mv(uint16_t mv) { if (!s_pmu_ready) return ESP_ERR_INVALID_STATE; PMU.setDC1Voltage(mv); return ESP_OK; }
extern "C" esp_err_t bsp_power_enable_dc1(bool enable) { if (!s_pmu_ready) return ESP_ERR_INVALID_STATE; if (enable) PMU.enableDC1(); else PMU.disableDC1(); return ESP_OK; }
extern "C" esp_err_t bsp_power_set_aldo1_voltage_mv(uint16_t mv) { if (!s_pmu_ready) return ESP_ERR_INVALID_STATE; PMU.setALDO1Voltage(mv); return ESP_OK; }
extern "C" esp_err_t bsp_power_enable_aldo1(bool enable) { if (!s_pmu_ready) return ESP_ERR_INVALID_STATE; if (enable) PMU.enableALDO1(); else PMU.disableALDO1(); return ESP_OK; }
extern "C" esp_err_t bsp_power_set_aldo2_voltage_mv(uint16_t mv) { if (!s_pmu_ready) return ESP_ERR_INVALID_STATE; PMU.setALDO2Voltage(mv); return ESP_OK; }
extern "C" esp_err_t bsp_power_enable_aldo2(bool enable) { if (!s_pmu_ready) return ESP_ERR_INVALID_STATE; if (enable) PMU.enableALDO2(); else PMU.disableALDO2(); return ESP_OK; }
