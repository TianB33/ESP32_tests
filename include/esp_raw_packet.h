/*header for preparing raw packets*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "driver/uart.h"

#define BEACON_SSID_OFFSET                   38
#define SRCADDR_OFFSET                       10
#define BSSID_OFFSET                         16
#define SEQNUM_OFFSET                        22
#define CONTENT_OFFSET                       38

typedef struct _M_msg{
    double scanning_angle;
    int id;
    char cmd[8];
} M_msg;

typedef struct _uart_msg{
    uint8_t addr[6];
    char cmd[8];
    unsigned pad:16;
    double scanning_angle;
    int led_level;
} uart_msg;

typedef struct _W_msg{
    double original_scanning_angle;
    int wid;
    int id;
    int led_level;
    char cmd[8];
} W_msg;

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

const uint8_t ieee80211_raw_template[] = {
	0x80, 0x00,							                                // 0-1: Frame Control
	0x00, 0x00,							                                // 2-3: Duration                                     
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,				                    // 4-9: Destination address (broadcast)                      -To be filled
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,				                    // 10-15: Source address                                     -To be filled
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,				                    // 16-21: BSSID                                              
	0x00, 0x00,							                                // 22-23: Sequence / fragment number
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,		             	// 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
	0x64, 0x00,							                                // 32-33: Beacon interval
	0x31, 0x04,							                                // 34-35: Capability info
	0x00, 0x00, 0x00,			                                        // 36-38: SSID parameter set, 0x00:length:content
	0x01, 0x08, 0x82, 0x84,	0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,	        // 39-48: Supported rates
	0x03, 0x01, 0x01,						                            // 49-51: DS Parameter set, current channel 1 (= 0x01),
	0x05, 0x04, 0x01, 0x02, 0x00, 0x00,				                    // 52-57: Traffic Indication Map
};

/*
@brief Given a pointer to the content of packet, prepare a raw packet based on ieee802.11 protocol.
@param beacon       the pointer to where the data will be written
@param self_addr    address of this device
@param dest_addr    address of the device where the raw packet will be sent to
@param content      the pointer to the content of the packet
@param size         size of content
*/
void prepare_raw_packet(uint8_t *beacon, uint8_t self_addr[6], uint8_t dest_addr[6], void *content, unsigned int size){
    memcpy(beacon, ieee80211_raw_template, BEACON_SSID_OFFSET - 1);
    beacon[BEACON_SSID_OFFSET - 1] = size;
    memcpy(&beacon[BEACON_SSID_OFFSET], content, size);
    memcpy(&beacon[BEACON_SSID_OFFSET + size], &ieee80211_raw_template[BEACON_SSID_OFFSET], sizeof(ieee80211_raw_template) - BEACON_SSID_OFFSET);

    memcpy(beacon + 4, dest_addr, 6 * sizeof(uint8_t));
    memcpy(beacon + 10, self_addr, 6 * sizeof(uint8_t));
}

