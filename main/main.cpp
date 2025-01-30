/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <thread>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "sensors/mpu6050.h"

extern "C" void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    i2c::I2CBusMaster bus(i2c::SDAType(gpio_num_t(10)), i2c::SCLType(gpio_num_t(11)));
    auto r = bus.Open();
    if (!r)
    {
        FMT_PRINTLN("i2c bus master error: {}", r.error());
        return;
    }

    MPU6050 mpu6050 = *MPU6050::Open(bus);
    FMT_PRINTLN("MPU6050: Id={}", mpu6050.GetId());
    mpu6050.SetPwrMgmt({.clksel = MPU6050::ClockSource::PLL_Gyro_X, .temp_dis = 0, .cycle = 0, .sleep = 0});
    mpu6050.SetPwrMgmt2({.stby_zg = 0, .stby_yg = 0, .stby_xg = 0});
    FMT_PRINTLN("MPU6050: Initializing DMP");
    if (auto r = mpu6050.InitDMP(); !r)
    {
        FMT_PRINTLN("MPU6050 err InitDMP: {}", r.error());
        return;
    }
    while(true)
    {
        FMT_PRINTLN("MPU6050: {}", mpu6050.GetAllMeasurements());
        //auto allR = *mpu6050.GetAllRaw();
        //FMT_PRINTLN("MPU6050(R): {}", allR);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return;


    //for (int i = 10; i >= 0; i--) {
    //    printf("Restarting in %d seconds...\n", i);
    //    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //}
    //printf("Restarting now.\n");
    //fflush(stdout);
    //esp_restart();
}
