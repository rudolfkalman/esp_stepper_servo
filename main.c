#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_SDA 21
#define I2C_SCL 22

#define AS5600_ADDR 0x36
#define RAW_ANGLE_H 0x0C

#define TIMEOUT_MS 100

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;

static bool as5600_read_angle(float *out_deg)
{
    uint8_t reg = RAW_ANGLE_H;
    uint8_t buf[2] = {0};

    esp_err_t ret = i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        buf,
        2,
        TIMEOUT_MS
    );

    if (ret != ESP_OK) {
        printf("I2C error: %s\n", esp_err_to_name(ret));
        return false;
    }

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    raw &= 0x0FFF;

    *out_deg = (float)raw * 360.0f / 4096.0f;

    return true;
}

void app_main(void)
{
    printf("AS5600 simple reader start\n");

    i2c_master_bus_config_t bus_conf = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_APB,
        .glitch_ignore_cnt = 7,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus));

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_ADDR,
        .scl_speed_hz = 100000,   // 安定優先（まず100k）
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_conf, &dev));

    while (1) {
        float angle = 0.0f;

        if (as5600_read_angle(&angle)) {
            printf("Angle: %.2f deg\n", angle);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
