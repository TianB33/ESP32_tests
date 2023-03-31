#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/param.h>
#include <lwip/netdb.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "esp_task_wdt.h"
#include "sdkconfig.h"

#include "esp_raw_packet.h"

#define BEACON_SSID_OFFSET                   38
#define SRCADDR_OFFSET                       10
#define BSSID_OFFSET                         16
#define SEQNUM_OFFSET                        22
#define TOTAL_LINES                          (sizeof(msg) / sizeof(char *))

#define DELAY                                10000  //in microsecond
#define ALLDIR                               0
#define SCANNING_SPEED                       3.0    //changing x deg/DELAY

#define SCANNING_ANGLE                       (double) 30.0

uint8_t BROADCAST_ADDR[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t M_addr[6] = {0xa0, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb};
uint8_t MAC[6];

static const char* TAG = "M-Node";

void init_AP(){

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_WIFI_SSID,
            .ssid_len = strlen(CONFIG_WIFI_SSID),
            .channel = CONFIG_WIFI_CHANNEL,
            .password = CONFIG_WIFI_PASSWORD,
            .max_connection = CONFIG_WIFI_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .beacon_interval = 100,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    //esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
    esp_wifi_config_80211_tx_rate(WIFI_IF_AP, WIFI_PHY_RATE_54M);
    //esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_L);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));

    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, MAC));

    ESP_LOGI(TAG, "init_AP() finished. SSID: %s password: %s channel: %d\n", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD, CONFIG_WIFI_CHANNEL);
    ESP_LOGI(TAG, "WiFi AP MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n", MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]);
}

static void M_node_task(){
    //uint16_t seqnum[TOTAL_LINES] = { 0 };
    //vTaskDelay(10 / TOTAL_LINES);
    uint8_t beacon[256] = {0};

    if (!ALLDIR){
        M_msg msg = {
            .scanning_angle = SCANNING_ANGLE,
            .id = 1,
            .cmd = "PING",
        };

        prepare_raw_packet(beacon, M_addr, BROADCAST_ADDR, &msg, sizeof(msg));

        while (1){      
            //ESP_ERROR_CHECK(esp_wifi_80211_tx(WIFI_IF_AP, beacon, sizeof(ieee80211_raw_template) + strlen(msg), true));
            
            esp_wifi_80211_tx(WIFI_IF_AP, beacon, 256, true);
            esp_rom_delay_us(DELAY);
            
            //vTaskDelay(1);
        }
    } else {
        double current_angle = 0.0;

        while (1) {
            M_msg msg = {
                .scanning_angle = current_angle,
                .id = 1,
                .cmd = "PING",
            };

            prepare_raw_packet(beacon, M_addr, BROADCAST_ADDR, &msg, sizeof(msg));

            esp_wifi_80211_tx(WIFI_IF_AP, beacon, 256, true);
            current_angle = fmod(current_angle + SCANNING_SPEED, 360.0);
            esp_rom_delay_us(DELAY);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    init_AP();
    
    vTaskDelay(10);
    ESP_ERROR_CHECK(esp_task_wdt_deinit());
    xTaskCreate(M_node_task, "M_node_task", 4096, NULL, 10, NULL);
}
