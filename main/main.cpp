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
#include "driver/gpio.h"

void mpu6050_int(void *param)
{
    QueueHandle_t &c = *static_cast<QueueHandle_t*>(param);
    uint8_t dummy = 0;
    xQueueSendFromISR(c, &dummy, nullptr);
}

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
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    i2c::I2CBusMaster bus(i2c::SDAType(gpio_num_t(10)), i2c::SCLType(gpio_num_t(11)));
    auto r = bus.Open();
    if (!r)
    {
        FMT_PRINTLN("i2c bus master error: {}", r.error());
        return;
    }

    MPU6050 mpu6050 = *MPU6050::Open(bus);
    FMT_PRINTLN("MPU6050: Id={}", mpu6050.GetId());
    mpu6050.SetPwrMgmt({.clksel = MPU6050::ClockSource::PLL_Gyro_X, .temp_dis = 1, .cycle = 0, .sleep = 0});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mpu6050.SetPwrMgmt2({.stby_zg = 1, .stby_yg = 1, .stby_xg = 1, .stby_ya = 1, .stby_xa = 1});
    //FMT_PRINTLN("MPU6050: Initializing DMP");
    //mpu6050.SetUserCtrl({ .fifo_reset = true, .dmp_reset = true});
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //if (auto r = mpu6050.InitDMP(); !r)
    //{
    //    FMT_PRINTLN("MPU6050 err InitDMP: {}", r.error());
    //    return;
    //}
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //mpu6050.SetUserCtrl({ .fifo_reset = true, .dmp_reset = true});
    //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //mpu6050.SetUserCtrl({ .fifo_enable = true, .dmp_enable = true});
    //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto xq = xQueueCreate(16, sizeof(uint8_t));
    gpio_config_t mpu6050_int_cfg = {
        .pin_bit_mask = 1ULL << 12,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
        .hys_ctrl_mode = gpio_hys_ctrl_mode_t{}
#endif
    };

    gpio_config(&mpu6050_int_cfg);
    gpio_isr_handler_add(gpio_num_t(12), mpu6050_int, &xq);

    mpu6050.SignalPathReset({});
    mpu6050.SetAccelConfig(MPU6050::AccelFullScaleRange::_2_g, MPU6050::AccelFilter::_5Hz);

    mpu6050.ConfigureMotionDetection(
            5, 
            10, 
            {
                .motion_counter_decrement_rate = MPU6050::DecrementRate::_1, 
                .free_fall_counter_decrement_rate = MPU6050::DecrementRate::_1, 
                //.accel_on_delay = 1
            }
        );
    mpu6050.ConfigureZeroMotionDetection(56, 10);

    if (auto r = mpu6050.ConfigureInterrupts({.int_latch = 1}); !r)
    {
        FMT_PRINTLN("MPU6050 err int cfg: {}", r.error());
        return;
    }

    if (auto r = mpu6050.EnableInterrupts({.dmp = 1, .zero_motion = 1, .motion = 1, .free_fall = 1}); !r)
    {
        FMT_PRINTLN("MPU6050 err int en: {}", r.error());
        return;
    }

    uint8_t dummy;
    while(true)
    {
        if (xQueueReceive(xq, &dummy, 1000 / portTICK_PERIOD_MS))
        {
            FMT_PRINTLN("MPU6050 Interrupt happened");
            //FMT_PRINTLN("MPU6050 Interrupt: {}", mpu6050.GetInterruptStatus());
            FMT_PRINTLN("MPU6050 Interrupt: {}", mpu6050.GetInterruptStatus());
            FMT_PRINTLN("MPU6050 Motion status: {}", mpu6050.GetMotionDetectionStatus());
            {
                FMT_PRINTLN("MPU6050: {}", mpu6050.GetAllMeasurements());
            }
        }

        //auto allR = *mpu6050.GetAllRaw();
        //FMT_PRINTLN("MPU6050(R): {}", allR);
        //std::this_thread::sleep_for(std::chrono::seconds(1));
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
