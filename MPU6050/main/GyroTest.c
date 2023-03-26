/*tester for mpu6050*/

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "esp_gyro_accel.h"

const char *TAG = "MPU6050Test";

void mpu6050_init(){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,        
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = PIN_CLK,         
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = FREQ,  
        .clk_flags = 0,            
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1); //adjust the power mode
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd)); 
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

accel_data mpu6050_get_accel(){
    accel_data res;
    i2c_cmd_handle_t cmd;
    uint8_t accel[14];

    //indicate address of register to read later
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    //read:
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 1, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 2, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 3, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 4, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 5, 1)); //NACK for the last byte read. 0 - ACK, 1 - NACK

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    //enter into res and return
    res.x = (accel[0] << 8) | accel[1];
    res.y = (accel[2] << 8) | accel[3];
    res.z = (accel[4] << 8) | accel[5];

    return res;
}

gyro_data mpu6050_get_gyro(){
    gyro_data res;
    i2c_cmd_handle_t cmd;
    uint8_t gyro[14];

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_GYRO_XOUT_H, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, gyro, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, gyro + 1, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, gyro + 2, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, gyro + 3, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, gyro + 4, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, gyro + 5, 1));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    res.x = (gyro[0] << 8) | gyro[1];
    res.y = (gyro[2] << 8) | gyro[3];
    res.z = (gyro[4] << 8) | gyro[5];
    
    return res;
}

pitch_and_roll get_raw_pitch_and_roll(accel_data accel){
    double nx, ny, nz;
    nx = (double) accel.x / A_LSB; 
    ny = (double) accel.y / A_LSB;
    nz = (double) accel.z / A_LSB;

    double pitchAccel = atan2(nx, sqrt(ny * ny + nz * nz));
    double rollAccel = atan2(-ny, nz);

    pitch_and_roll res = {
        .pitch = pitchAccel,
        .roll = rollAccel,
    };
    return res;
}

void compl_filter_pitch_and_roll(accel_data accel, gyro_data gyro, double sampling_interval, double alpha, pitch_and_roll *last_obs){
    double nx, ny, nz;
    nx = (double) accel.x / A_LSB; 
    ny = (double) accel.y / A_LSB;
    nz = (double) accel.z / A_LSB;

    double pitchAccel = atan2(nx, sqrt(ny * ny + nz * nz));
    double rollAccel = atan2(-ny, nz);

    double pitchGyro = pitchAccel + (last_obs->pitch / G_LSB) * sampling_interval;
    double rollGyro = rollAccel + (last_obs->roll / G_LSB)* sampling_interval;

    // Combine accelerometer and gyroscope estimates using complementary filter
    double pitch = alpha * pitchGyro + (1 - alpha) * pitchAccel;
    double roll = alpha * rollGyro + (1 - alpha) * rollAccel;

    last_obs->pitch = pitch;
    last_obs->roll = roll;
}

void mpu6050(){
    mpu6050_init();
    accel_data dt = mpu6050_get_accel();
    pitch_and_roll pr = get_raw_pitch_and_roll(dt);

    while(1){
        accel_data dt1 = mpu6050_get_accel();
        //printf("accel data: %hd %hd %hd\n", dt1.x, dt1.y, dt1.z);

        gyro_data gt1 = mpu6050_get_gyro();
        //printf("gyro data: %hd %hd %hd\n", gt1.x, gt1.y, gt1.z);

        update_pitch_and_roll(dt1, gt1, 0.1, 0.98, &pr);
        printf("Filtered: %lf        %lf\n", pr.pitch, pr.roll);

        vTaskDelay(10);
    }
}

void app_main(void)
{
    //ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_task_wdt_deinit());
    mpu6050();
}
