/*tester for gy511, accelerometer and magnetometer.*/

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "esp_gy511.h"

void gy511_init(){
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
}

void gy511_magneto_init(){
    i2c_cmd_handle_t cmd;

    uint8_t data[2] = {0x02, 0x00};

    cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (MAGNETO_I2C << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write(cmd, data, sizeof(data), 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd)); 
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

void gy511_accel_init(){
    i2c_cmd_handle_t cmd;

    uint8_t data[2] = {0x02, 0x00};

    cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (ACCEL_I2C << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write(cmd, data, sizeof(data), 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd)); 
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

accel_data gy511_get_accel(){
    accel_data res;
    i2c_cmd_handle_t cmd;
    uint8_t accel[14];

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (ACCEL_I2C << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, OUT_X_L_ACCEL, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (ACCEL_I2C << 1) | I2C_MASTER_READ, 1));

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 1, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 2, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 3, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 4, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, accel + 5, 1));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    res.x = (accel[0] << 8) | accel[1];
    res.y = (accel[2] << 8) | accel[3];
    res.z = (accel[4] << 8) | accel[5];

    return res;
}

magneto_data gy511_get_magneto(){
    magneto_data res;
    i2c_cmd_handle_t cmd;
    uint8_t magneto[14];

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (MAGNETO_I2C << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, OUT_X_L_MEGNETO, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (MAGNETO_I2C << 1) | I2C_MASTER_READ, 1));

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, magneto, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, magneto + 1, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, magneto + 2, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, magneto + 3, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, magneto + 4, 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, magneto + 5, 1));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    res.x = (magneto[0] << 8) | magneto[1];
    res.z = (magneto[2] << 8) | magneto[3];
    res.y = (magneto[4] << 8) | magneto[5];
    
    return res;
}

void gy511(){
    init();
    while(1){
        magneto_init();
        
        magneto_data md = get_magneto();

        printf("magneto data: %hd %hd %hd\n", md.x, md.y, md.z);
        double heading = (atan2(md.y, md.x) * 180.0 + 13.6) / M_PI;
        if (heading < 0) {
            heading += 360.0;
        }

        printf("angle: %lf\n", heading);
        vTaskDelay(10);
    }
}

void app_main(){
    ESP_ERROR_CHECK(esp_task_wdt_deinit());
    gy511();
}