/*example of promiscuous mode*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define ALL 1

uint8_t APMAC[6] = {0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0};
uint8_t BROADMAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static const char* TAG = "promisc";

int AP_compare(uint8_t A[6], uint8_t B[6]){
    if (memcmp(A, B, sizeof(uint8_t) * 6) == 0) return 1;
    return 0;
}

typedef struct ieee80211_payload_header_{
    unsigned frame_control:16;       //2 byte
    unsigned duration_id:16;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
    unsigned sequence_ctrl:16;      
} ieee_80211_payload_header;

typedef struct ieee80211_payload_{
    ieee_80211_payload_header hdr;
    uint8_t payload[0];
} ieee_80211_payload;

char *type2str(wifi_promiscuous_pkt_type_t type){
    switch(type){
        case WIFI_PKT_MGMT: return "MGMT";
	    case WIFI_PKT_DATA: return "DATA";
        case WIFI_PKT_CTRL: return "CTRL";
	    default:	
	    case WIFI_PKT_MISC: return "MISC";
    }
}

void sniffer_handler(void *buf, wifi_promiscuous_pkt_type_t type){
    if (type == WIFI_PKT_MISC) return;

    wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *) buf;
	ieee_80211_payload *ipkt = (ieee_80211_payload *)ppkt->payload;
	ieee_80211_payload_header *hdr = &ipkt->hdr;

    if (ALL || AP_compare(APMAC, hdr->addr1) || AP_compare(APMAC, hdr->addr2) || AP_compare(APMAC, hdr->addr3)){
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

void app_main(void)
{
    //uint8_t channel = 1;

    /*disable the watchdog*/
    ESP_ERROR_CHECK(esp_task_wdt_deinit());
    sniffer_init();
}
