/*header for gy511, accelerometer and magnetometer*/

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "esp_task_wdt.h"
#include "sdkconfig.h"

#define PIN_SDA                 21
#define PIN_CLK                 22

#define MAGNETO_I2C             0x1E //i2c address for magnetometer and accelerometer on gy511. Note they are different;
#define ACCEL_I2C               0x19

#define FREQ                    100000

#define OUT_X_L_ACCEL           0x28 //register address for accelerometer/magnetometer data
#define OUT_X_L_MEGNETO         0x03

typedef struct _accel_data{
    short x;
    short y;
    short z;
} accel_data;

typedef struct _magneto_data{
    short x;
    short y;
    short z;
} magneto_data;

/*
@brief initialize gy511 using default settings. To use accelerometer or magnetometer, need gy511_accel_init or gy511_magneto_init.
*/
void gy511_init();

/*
@brief initialize gy511 to use magnetometer
*/
void gy511_magneto_init();

/*
@brief initialize gy511 to use accelerometer
*/
void gy511_accel_init();

/*
@brief get accelerometer data. Need gy511_accel_init.
*/
accel_data gy511_get_accel();

/*
@brief get magnetometer data. Need gy511_magneto_init.
*/
magneto_data gy511_get_magneto();
