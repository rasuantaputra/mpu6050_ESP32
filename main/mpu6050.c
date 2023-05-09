#include <stdio.h>

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#define TAG "Data Sesor Accelerometer"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define I2C_ADDRESS 0x68 // I2C Address of MPU6050

// MPU6050 Register
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B

void i2c_master_init();
void mpu6050_task();
void mpu605_read_data();

void app_main(void)
{
    nvs_flash_init();
    i2c_master_init();
    mpu6050_task();
    mpu605_read_data();
}

void i2c_master_init()
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000};
    
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void mpu6050_task()
{
    i2c_cmd_handle_t cmd;

    // delay
    vTaskDelay(200/portTICK_PERIOD_MS);

    cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
    i2c_master_write_byte(cmd, 0, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void mpu605_read_data() // fungsi ini masih raw data, nantinya akan dimasukan sekalian algoritme pengolahannnya
{

    i2c_cmd_handle_t cmd;

    uint8_t data[14];
    short x, y, z;

    while (1)
    {
		// Tell the MPU6050 to position the internal register pointer to register
		// MPU6050_ACCEL_XOUT_H.
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (I2C_ADDRESS << 1| I2C_MASTER_READ), 1);

        i2c_master_read_byte(cmd, data, 0);
        i2c_master_read_byte(cmd, data + 1, 0);
        i2c_master_read_byte(cmd, data + 2, 0);
        i2c_master_read_byte(cmd, data + 3, 0);
        i2c_master_read_byte(cmd, data + 4, 0);
        i2c_master_read_byte(cmd, data + 5, 1);

        //i2c_master_read(cmd, data, sizeof(data), 1); // malloc ?
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        x = (data[0] << 8) | data[1];
        y = (data[2] << 8) | data[3];
        z = (data[4] << 8) | data[5];

        ESP_LOGI(TAG, "sumbu_x: %d, sumbu_y: %d, sumbu_z: %d", x, y, z);

        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}