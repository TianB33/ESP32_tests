#include <stdio.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "driver/uart.h"
#include "esp_timer.h"

#include "esp_raw_packet.h"

#define ALL                0
#define REAL               0
#define TIMEOUT            20
#define DEVICENUM          128
#define WINDOWLEN          10

#define ORIENTATION        30.0
#define ANGLE_THRESHOLD    10.0            //inclusive

#define M1_threshold       -55

uint8_t M1_addr[6] = {0xa0, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb};

const char *TAG = "W_RECV";

int device_cnt = 0;
int devices_id[DEVICENUM] = {0};
int devices_obs[DEVICENUM] = {0};
int devices_rssi[DEVICENUM][WINDOWLEN] = {0};

int AP_compare(uint8_t A[6], uint8_t B[6]){
    if (memcmp(A, B, sizeof(uint8_t) * 6) == 0) return 1;
    return 0;
}

double abs_diff(double a, double b){
    if (a > b) return a - b;
    return b - a;
}

char *type2str(wifi_promiscuous_pkt_type_t type){
    switch(type){
        case WIFI_PKT_MGMT: return "MGMT";
	    case WIFI_PKT_DATA: return "DATA";
        case WIFI_PKT_CTRL: return "CTRL";
	    default:	
	    case WIFI_PKT_MISC: return "MISC";
    }
}

void IRAM_ATTR sniffer_handler(void *buf, wifi_promiscuous_pkt_type_t type){
    if (type == WIFI_PKT_MISC) return;

    wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *) buf;
	ieee_80211_payload *ipkt = (ieee_80211_payload *)ppkt->payload;
	ieee_80211_payload_header *hdr = &ipkt->hdr;

    if (ALL || AP_compare(M1_addr, hdr->addr2)){
        /*
        ESP_LOGI(TAG, "PACKET TYPE=%s, CHAN=%02d, RSSI=%02d,"
            " ADDR1=%02x:%02x:%02x:%02x:%02x:%02x,"
            " ADDR2=%02x:%02x:%02x:%02x:%02x:%02x,"
            " ADDR3=%02x:%02x:%02x:%02x:%02x:%02x\n",
            type2str(type),
            ppkt->rx_ctrl.channel,
            ppkt->rx_ctrl.rssi,
            hdr->addr1[0],hdr->addr1[1],hdr->addr1[2],
            hdr->addr1[3],hdr->addr1[4],hdr->addr1[5],
            hdr->addr2[0],hdr->addr2[1],hdr->addr2[2],
            hdr->addr2[3],hdr->addr2[4],hdr->addr2[5],
            hdr->addr3[0],hdr->addr3[1],hdr->addr3[2],
            hdr->addr3[3],hdr->addr3[4],hdr->addr3[5]
        );
        */

        M_msg recv_msg;
        memcpy(&recv_msg, (void *) ((char *)ipkt + CONTENT_OFFSET), sizeof(recv_msg));

        //for (int i = 0; i < 128; i++) printf("%02x ", *(uint8_t *)((char *)ipkt + i));
        //printf("%lf %d %s\n", *(double *)&recv_msg, *(int *)((char *)&recv_msg + 8), ((char *)&recv_msg + 12));
        if (strcmp(recv_msg.cmd, "PING") == 0){
            if (abs_diff(ORIENTATION, recv_msg.scanning_angle) <= ANGLE_THRESHOLD) {
                int rssi = ppkt->rx_ctrl.rssi;
                printf("RSSI: %d\n", rssi);
                if (rssi > M1_threshold){
                    
                    uart_msg asktosend = {
                        .scanning_angle = recv_msg.scanning_angle,
                        .cmd = "PONG",
                        .led_level = rssi,
                    };
                    /*
                    if (rssi > -20) asktosend.led_level = 1;
                    else if (rssi > -27) asktosend.led_level = 2;
                    else if (rssi > -35) asktosend.led_level = 3;
                    else if (rssi > -42) asktosend.led_level = 4;
                    */

                    memcpy(&asktosend.addr, hdr->addr2, sizeof(hdr->addr2));

                    int bytesend = uart_write_bytes(CONFIG_UART_PORT_NUM, &asktosend, sizeof(asktosend));
                } 
            }
        }
    }

}

void sniffer_init(void){
    ESP_ERROR_CHECK(nvs_flash_init());

    /*initialize TCP/IP stack and initialize a default event handler for system events*/
    ESP_ERROR_CHECK(esp_netif_init()); 
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /*initialize the WiFi*/
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /*indicate that modifications will be only in memory*/
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    /*set to NULL mode for sniffing, start WiFi, set to promiscuous mode and set the handler*/
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_rx_cb(&sniffer_handler);
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

void app_main(void)
{
    //uint8_t channel = 1;

    /*disable the watchdog*/
    ESP_ERROR_CHECK(esp_task_wdt_deinit());
    sniffer_init();
    uart_init();
    //xTaskCreate(respond, "respond_task", 4096, NULL, 10, NULL);
}
