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
#include "driver/gpio.h"
#include "esp_rom_gpio.h"

#include "esp_raw_packet.h"

#define LED_PIN CONFIG_LED_PIN
#define ALL 0

uint8_t W_addr[6] = {0xc0, 0x00, 0x00, 0x00, 0x00, 0x01};
int cur_led = -1;
const char *TAG = "M_recv";

int AP_compare(uint8_t A[6], uint8_t B[6]){
    if (memcmp(A, B, sizeof(uint8_t) * 6) == 0) return 1;
    return 0;
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

//blink the led with different frequencies
void led_blink(){
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  
    while (1){
        gpio_set_level(LED_PIN, 0);
        if (cur_led > 0){
            int tmp = cur_led * 40;
            vTaskDelay(tmp);
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(tmp);
        }
    }
}

void IRAM_ATTR sniffer_handler(void *buf, wifi_promiscuous_pkt_type_t type){
    if (type == WIFI_PKT_MISC) return;

    wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *) buf;
	ieee_80211_payload *ipkt = (ieee_80211_payload *)ppkt->payload;
	ieee_80211_payload_header *hdr = &ipkt->hdr;

    if (ALL || AP_compare(W_addr, hdr->addr2)){ //source addr is from W1 or W2 
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
        W_msg recv_msg;
        memcpy(&recv_msg, (void *) ((char *)ipkt + CONTENT_OFFSET), sizeof(recv_msg));
   
        if (strcmp(recv_msg.cmd, "PONG") == 0){
            cur_led = recv_msg.led_level;
            printf("level: %d\n", cur_led);
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

void app_main(){
    ESP_ERROR_CHECK(esp_task_wdt_deinit());

    sniffer_init();
    led_blink();
}