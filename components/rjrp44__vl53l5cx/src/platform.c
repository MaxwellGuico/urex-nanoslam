/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "platform.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "vl53_io";
#ifdef CONFIG_VL53L5CX_MUXED_BUS
/* Shared by all per-channel driver instances; access only under bus_mutex. */
static i2c_master_dev_handle_t s_selected_mux;
static uint8_t s_selected_channel=UINT8_MAX;
void VL53L5CX_InvalidateMuxCache(void)
{ s_selected_mux=NULL; s_selected_channel=UINT8_MAX; }
#endif

/* The ST API exposes an eight-bit platform status, whereas ESP-IDF uses the
 * wider esp_err_t namespace (for example ESP_ERR_INVALID_RESPONSE is 0x108).
 * Log the full value before returning a generic non-zero status so a NACK can
 * never be mistaken for a sensor-internal GO2 code after truncation. */
static uint8_t vl53l5cx_platform_error(VL53L5CX_Platform *p_platform,
                                      const char *operation,
                                      uint16_t register_address,
                                      esp_err_t err)
{
    unsigned mux_channel = UINT8_MAX;
#ifdef CONFIG_VL53L5CX_MUXED_BUS
    mux_channel = p_platform->mux_channel;
#endif
    ESP_LOGE(TAG,
             "sensor=%u mux_channel=%u operation=%s register=0x%04x "
             "esp_err=0x%08lx esp_err_name=%s",
             (unsigned)p_platform->sensor_id,
             mux_channel,
             operation,
             (unsigned)register_address,
             (unsigned long)(uint32_t)err,
             esp_err_to_name(err));
    return UINT8_MAX;
}

//Use the timeout from the config, the default timeout is -1
#if CONFIG_VL53L5CX_I2C_TIMEOUT == false
#define VL53L5CX_I2C_TIMEOUT (-1)
#else
#define VL53L5CX_I2C_TIMEOUT CONFIG_VL53L5CX_I2C_TIMEOUT_VALUE
#endif

#ifdef CONFIG_VL53L5CX_MUXED_BUS
static uint8_t vl53l5cx_platform_begin(VL53L5CX_Platform *p_platform,
                                       const char *operation,
                                       uint16_t register_address)
{
    if (p_platform->bus_mutex == NULL ||
        p_platform->mux_handle == NULL ||
        p_platform->mux_channel >= 8U) {
        return vl53l5cx_platform_error(p_platform, operation,
                                       register_address,
                                       ESP_ERR_INVALID_STATE);
    }

    if (xSemaphoreTake(p_platform->bus_mutex, portMAX_DELAY) != pdTRUE) {
        return vl53l5cx_platform_error(p_platform, operation,
                                       register_address, ESP_FAIL);
    }

    if (s_selected_mux == p_platform->mux_handle &&
        s_selected_channel == p_platform->mux_channel) return 0U;

    const uint8_t channel_mask =
        (uint8_t)(1U << p_platform->mux_channel);
    const esp_err_t err = i2c_master_transmit(
        p_platform->mux_handle,
        &channel_mask,
        sizeof(channel_mask),
        VL53L5CX_I2C_TIMEOUT);
    if (err != ESP_OK) {
        VL53L5CX_InvalidateMuxCache();
        xSemaphoreGive(p_platform->bus_mutex);
        return vl53l5cx_platform_error(p_platform, operation,
                                       register_address, err);
    }

    s_selected_mux=p_platform->mux_handle;
    s_selected_channel=p_platform->mux_channel;
    return 0U;
}

static void vl53l5cx_platform_end(VL53L5CX_Platform *p_platform)
{
    xSemaphoreGive(p_platform->bus_mutex);
}
#endif

//Define the reset scheme
#ifdef CONFIG_VL53L5CX_RESET_PIN_HIGH
#define VL53L5CX_RESET_LEVEL 1
#elif CONFIG_VL53L5CX_RESET_PIN_LOW
#define VL53L5CX_RESET_LEVEL 0
#endif

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
#ifdef CONFIG_VL53L5CX_MUXED_BUS
    uint32_t offset = 0U;
    while (offset < size) {
        uint32_t chunk_size = size - offset;
        if (chunk_size > CONFIG_VL53L5CX_I2C_WRITE_CHUNK_BYTES) {
            chunk_size = CONFIG_VL53L5CX_I2C_WRITE_CHUNK_BYTES;
        }

        const uint16_t chunk_address =
            (uint16_t)(RegisterAdress + offset);
        uint8_t status = vl53l5cx_platform_begin(
            p_platform, "mux_select_for_write", chunk_address);
        if (status != 0U) {
            return status;
        }

        uint8_t i2c_address[] = {
            (uint8_t)(chunk_address >> 8),
            (uint8_t)(chunk_address & 0xFFU)
        };
        i2c_master_transmit_multi_buffer_info_t i2c_buffers[2] = {
            {
                .write_buffer = i2c_address,
                .buffer_size = sizeof(i2c_address),
            },
            {
                .write_buffer = p_values + offset,
                .buffer_size = chunk_size,
            },
        };

        const esp_err_t err = i2c_master_multi_buffer_transmit(
            p_platform->handle,
            i2c_buffers,
            2,
            VL53L5CX_I2C_TIMEOUT);
        if (err != ESP_OK) VL53L5CX_InvalidateMuxCache();
        vl53l5cx_platform_end(p_platform);
        if (err != ESP_OK) {
            return vl53l5cx_platform_error(p_platform, "write",
                                           chunk_address, err);
        }
        offset += chunk_size;
    }
    return 0U;
#else
    i2c_master_transmit_multi_buffer_info_t i2c_buffers[2];
    uint8_t i2c_address[] = {RegisterAdress >> 8, RegisterAdress & 0xFF};
    i2c_buffers[0].write_buffer = i2c_address;
    i2c_buffers[0].buffer_size = 2;
    i2c_buffers[1].write_buffer = p_values;
    i2c_buffers[1].buffer_size = size;
    const esp_err_t err = i2c_master_multi_buffer_transmit(
        p_platform->handle, i2c_buffers, 2, VL53L5CX_I2C_TIMEOUT);
    return err == ESP_OK ? 0U : vl53l5cx_platform_error(
        p_platform, "write", RegisterAdress, err);
#endif
}

uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {

    //Write a single byte
    return VL53L5CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {

    //Add index to the data
    uint8_t i2c_address[] = {RegisterAdress >> 8, RegisterAdress & 0xFF};

#ifdef CONFIG_VL53L5CX_MUXED_BUS
    uint8_t status = vl53l5cx_platform_begin(
        p_platform, "mux_select_for_read", RegisterAdress);
    if (status != 0U) {
        return status;
    }
    const esp_err_t err = i2c_master_transmit_receive(
        p_platform->handle,
        i2c_address,
        sizeof(i2c_address),
        p_values,
        size,
        VL53L5CX_I2C_TIMEOUT);
    if (err != ESP_OK) VL53L5CX_InvalidateMuxCache();
    vl53l5cx_platform_end(p_platform);
    return err == ESP_OK ? 0U : vl53l5cx_platform_error(
        p_platform, "read", RegisterAdress, err);
#else
    const esp_err_t err = i2c_master_transmit_receive(
        p_platform->handle, i2c_address, 2, p_values, size,
        VL53L5CX_I2C_TIMEOUT);
    return err == ESP_OK ? 0U : vl53l5cx_platform_error(
        p_platform, "read", RegisterAdress, err);
#endif
}

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {

    //Read a single byte
    return VL53L5CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform* p_platform)
{
    gpio_set_direction(p_platform->reset_gpio, GPIO_MODE_OUTPUT);

    gpio_set_level(p_platform->reset_gpio, VL53L5CX_RESET_LEVEL);
    VL53L5CX_WaitMs(p_platform, 100);

    gpio_set_level(p_platform->reset_gpio, !VL53L5CX_RESET_LEVEL);
    VL53L5CX_WaitMs(p_platform, 100);

    return ESP_OK;
}

void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
    uint32_t i;
    uint8_t tmp[4] = {0};

    for (i = 0; i < size; i = i + 4) {

        tmp[0] = buffer[i + 3];
        tmp[1] = buffer[i + 2];
        tmp[2] = buffer[i + 1];
        tmp[3] = buffer[i];

        memcpy(&(buffer[i]), tmp, 4);
    }
}

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs) {
    vTaskDelay(TimeMs / portTICK_PERIOD_MS);

    return ESP_OK;
}
