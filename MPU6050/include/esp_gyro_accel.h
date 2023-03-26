/*header for MPU6050 gyroscope and accelerometer*/

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "esp_task_wdt.h"
#include "sdkconfig.h"

#define PIN_SDA                 21
#define PIN_CLK                 22
#define I2C_ADDRESS             0x68      //I2C address of MPU6050
#define FREQ                    100000
#define ACCEL                   0
#define GYRO                    1

#define A_LSB                   16384     //default values, see the manual for MPU6050
#define G_LSB                   131

#define MPU6050_ACCEL_XOUT_H    0x3B      //see manual
#define MPU6050_GYRO_XOUT_H     0x43

#define MPU6050_PWR_MGMT_1      0x6B

typedef struct _accel_data{
    short x;
    short y;
    short z;
} accel_data;

typedef struct _gyro_data{
    short x;
    short y;
    short z;
} gyro_data;

typedef struct _pitch_and_roll{
    double pitch;
    double roll;
} pitch_and_roll;

/*
@brief initialize mpu6050 using all default settings
*/
void mpu6050_init();

/*
@brief read the accelerometer data
*/
accel_data mpu6050_get_accel();

/*
@brief read the gyroscope data
*/
gyro_data mpu6050_get_gyro();

/*
@brief calculate the unfiltered pitch and roll
@param accel   accelerometer data
*/
pitch_and_roll get_raw_pitch_and_roll(accel_data accel);

/*
@brief filter pitch and roll data based on gyroscope data, using complementary filter
@param accel      accelerometer data
@param gyro       gyroscope data
@param sampling_interval
@param alpha      paramater for complementary filter
@param last_obs   data to filter
*/
void compl_filter_pitch_and_roll(accel_data accel, gyro_data gyro, double sampling_interval, double alpha, pitch_and_roll *last_obs);