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

#define PORT           "9999"
#define BROADCAST      "255.255.255.255"
#define TESTHOST       "192.168.2.32"

static const char* TAG = "UDP-broadcaster";
char msg[8] = "message";
int msglen = 8;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void init_AP(){

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_WIFI_SSID,
            .ssid_len = strlen(CONFIG_WIFI_SSID),
            .channel = CONFIG_WIFI_CHANNEL,
            .password = CONFIG_WIFI_PASSWORD,
            .max_connection = CONFIG_WIFI_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    uint8_t MAC[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, MAC));

    ESP_LOGI(TAG, "init_AP() finished. SSID: %s password: %s channel: %d\n", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD, CONFIG_WIFI_CHANNEL);
    ESP_LOGI(TAG, "WiFi AP MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n", MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]);
}

static void udp_broadcast(){

    struct addrinfo hints;
    struct addrinfo *dest;
    struct addrinfo *p;
    int status;
    int broadcast_perm = 1;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    if ((status = getaddrinfo(TESTHOST, PORT, &hints, &dest)) != 0){
        ESP_LOGI(TAG, "getaddrinfo() error.\n");
    }

    int sock = 0;
    for (p = dest; p != NULL; p = p->ai_next){
        if ((sock = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1){
            ESP_LOGI(TAG, "failed to create a socket. Reason: %s\n", strerror(errno));
            continue;
        }
        break;
    }

    if (p == NULL){
        ESP_LOGI(TAG, "ERROR: cannot create a socket.\n");
    }

    if ((status = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_perm, sizeof(broadcast_perm))) == -1){
        ESP_LOGI(TAG, "ERROR: setsockopt() failed. Reason: %s", strerror(errno));
    }

    char send[msglen];
    strncpy(send, msg, msglen);
    send[msglen - 1] = '\0';

    while (1){
        int status = sendto(sock, send, msglen, 0, p->ai_addr, p->ai_addrlen);
        if (status == -1){
            ESP_LOGI(TAG, "sendto() failed. Reason: %s\n", strerror(errno));
        } else {
            ESP_LOGI(TAG, "successfully send %d chars to %s.\n", msglen, BROADCAST);
        }
        vTaskDelay(10);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    init_AP();
    
    vTaskDelay(10);

    xTaskCreate(udp_broadcast, "udp_broadcast_task", 16384, NULL, 10, NULL);
}
