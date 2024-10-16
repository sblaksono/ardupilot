/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_S3CAM

// make sensor selection clearer
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_IMU_SPI2(driver, devname1, devname2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname1),hal.spi->get_device(devname2),##args))

#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))

#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))


//- these are missing from esp-idf......will not be needed later
#define RTC_WDT_STG_SEL_OFF             0
#define RTC_WDT_STG_SEL_INT             1
#define RTC_WDT_STG_SEL_RESET_CPU       2
#define RTC_WDT_STG_SEL_RESET_SYSTEM    3
#define RTC_WDT_STG_SEL_RESET_RTC       4

#define HAL_ESP32_BOARD_NAME "esp32s3cam"

#define TRUE						1
#define FALSE						0

// Inertial sensors
//#define HAL_INS_DEFAULT HAL_INS_NONE
#define HAL_INS_DEFAULT HAL_INS_MPU9250_SPI
#define HAL_INS_NAME "mpu9250"
#define HAL_INS_PROBE_LIST PROBE_IMU_SPI(Invensense, HAL_INS_NAME, ROTATION_NONE)

// BAROMETER
//#define HAL_BARO_DEFAULT HAL_BARO_BMP280_SPI
//#define HAL_BARO_NAME "bmp280"
//#define HAL_BARO_PROBE_LIST PROBE_BARO_SPI(BMP280, HAL_BARO_NAME)
#define HAL_BARO_ALLOW_INIT_NO_BARO 1

// COMPASS
//#define AP_COMPASS_ENABLE_DEFAULT 0
#define HAL_MAG_PROBE_LIST PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
#define HAL_PROBE_EXTERNAL_I2C_COMPASSES 1
#define AP_COMPASS_AK8963_ENABLED 1
#define ALLOW_ARM_NO_COMPASS 1

#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

// ADC -- use ADS1115
//#define HAL_DISABLE_ADC_DRIVER 1
//#define HAL_USE_ADC 1
//#define HAL_ESP32_ADC_PINS { {ADC1_GPIO1_CHANNEL, 1, 1} }
#define HAL_ESP32_ADC_ADS1115 1

// see boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

// WIFI
#define HAL_ESP32_WIFI 2  //1-TCP, 2-UDP, comment this line = without wifi
#define WIFI_SSID "ardupilot-esp32"
#define WIFI_PWD "ardupilot-esp32"
//#define WIFI_CHANNEL 5

// RCOUT -- use PCA9685
#define HAL_ESP32_RCOUT_PCA9685 1
#define HAL_ESP32_RCOUT_PCA9685_OE_PIN GPIO_NUM_1
//#define HAL_ESP32_RCOUT {}

// SPI Buses
#define HAL_ESP32_SPI_BUSES \
    {.host=SPI3_HOST, .dma_ch=SPI_DMA_CH_AUTO, .mosi=GPIO_NUM_41, .miso=GPIO_NUM_47, .sclk=GPIO_NUM_42}

// SPI Devices
#define HAL_ESP32_SPI_DEVICES \
    {.name=HAL_INS_NAME, .bus=0, .device=0, .cs=GPIO_NUM_21,  .mode=0, .lspeed=2*MHZ, .hspeed=8*MHZ}

// I2C Buses
#define HAL_ESP32_I2C_BUSES \
    {.port=I2C_NUM_0, .sda=GPIO_NUM_14, .scl=GPIO_NUM_3, .speed=400*KHZ, .internal=true}

// no RCIN
//#define HAL_ESP32_RCIN GPIO_NUM_4
//#define HAL_ESP32_RMT_RX_PIN_NUMBER GPIO_NUM_4

//HARDWARE UARTS
#define HAL_ESP32_UART_DEVICES \
    {.port=UART_NUM_0, .rx=GPIO_NUM_44, .tx=GPIO_NUM_43}

//LED
//#define DEFAULT_NTF_LED_TYPES Notify_LED_None
//#define HAL_GPIO_LED_ON 1
//#define AP_NOTIFY_GPIO_LED_1_ENABLED 1
//#define AP_NOTIFY_GPIO_LED_1_PIN 2
#define DEFAULT_NTF_LED_TYPES Notify_LED_Onboard_RGB
#define AP_NOTIFY_ONBOARD_RGB_LED_ENABLED 1
#define AP_NOTIFY_ONBOARD_RGB_LED_PIN GPIO_NUM_48

//BUZZER
#define HAL_BUZZER_PIN 2

//SDCARD
#define HAL_ESP32_SDCARD 1
#define HAL_ESP32_SDMMC
#define HAL_ESP32_SDMMC_SLOT_CONFIG() {\
    .clk = GPIO_NUM_39, \
    .cmd = GPIO_NUM_38, \
    .d0 = GPIO_NUM_40, \
    .d1 = GPIO_NUM_NC, \
    .d2 = GPIO_NUM_NC, \
    .d3 = GPIO_NUM_NC, \
    .d4 = GPIO_NUM_NC, \
    .d5 = GPIO_NUM_NC, \
    .d6 = GPIO_NUM_NC, \
    .d7 = GPIO_NUM_NC, \
    .cd = GPIO_NUM_NC, \
    .wp = GPIO_NUM_NC, \
    .width = 1, \
    .flags = 0, \
}

#define HAL_LOGGING_FILESYSTEM_ENABLED 1
#define HAL_LOGGING_DATAFLASH_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 0

#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"

#define HAL_LOGGING_BACKENDS_DEFAULT 1

#define AP_RCPROTOCOL_ENABLED 0

#define AP_FILESYSTEM_ESP32_ENABLED 0
#define AP_SCRIPTING_ENABLED 0
#define HAL_USE_EMPTY_STORAGE 1

