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
#include "esp_timer.h"
#include "driver/uart.h"

#include "esp_raw_packet.h"

#define DEVICENUM    10
#define TIMEOUT      200

/*about rssi - frequency conversion*/
/*rssi vs number of packet sent in 100ms: >-10 - 10, -11~-20 - 7, -21~-30 - 5, -31~-40 - 2, -41~-50 - 1, <-50 - 0 */

const int PONG_FREQ[6] = {10, 7, 5, 2, 1, 0};       

const char *TAG = "W_SEND";

uint8_t MAC[6] = {0};
uint8_t W_addr[6] = {0xc0, 0x00, 0x00, 0x00, 0x00, 0x01};

uint8_t M1_addr[6] = {0xa0, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb};
uint8_t M2_addr[6] = {0xa0, 0xbb, 0xbb, 0xbb, 0xbb, 0xcc};

uint8_t ieee80211_raw[58];

int AP_compare(uint8_t A[6], uint8_t B[6]){
    if (memcmp(A, B, sizeof(uint8_t) * 6) == 0) return 1;
    return 0;
}

void uart_init(){
    uart_config_t uart_config = {
        .baud_rate = CONFIG_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(CONFIG_UART_PORT_NUM, 1024 * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_UART_PORT_NUM, CONFIG_UART_TXD, CONFIG_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

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

void W_task(){
    uart_msg recv;
    memset(&recv, 0, sizeof recv);
    ESP_ERROR_CHECK(uart_flush_input(CONFIG_UART_PORT_NUM));
    
    while (1){
        int byte_read = uart_read_bytes(CONFIG_UART_PORT_NUM, &recv, sizeof(recv), TIMEOUT);
        if (byte_read == 0) continue;
        if (strcmp(recv.cmd, "PONG") == 0){
 
            W_msg send = {
                .original_scanning_angle = recv.scanning_angle,
                .wid = 13,
                .id = 2,
                .cmd = "PONG",
            };

            uint8_t beacon[256] = {0};
            prepare_raw_packet(beacon, W_addr, recv.addr, &send, sizeof(send));

            int lvl = (-1 * recv.rssi) / 10;
            if (lvl > 5) lvl = 5;

            int64_t start = esp_timer_get_time();
            for (int i = 0; i < PONG_FREQ[lvl]; i++){
                esp_wifi_80211_tx(WIFI_IF_AP, beacon, 256, true);
            }
            
            printf("message sent %d times to M..., in %lld us\n", PONG_FREQ[lvl], esp_timer_get_time() - start);
            vTaskDelay(10);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    uart_init();
    init_AP();
    ESP_ERROR_CHECK(esp_task_wdt_deinit());
    
    xTaskCreate(W_task, "W_task", 4096, NULL, 10, NULL);
}