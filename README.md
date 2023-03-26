# ESP32_tests

some tester codes and PING_PONG implementation.

## how to use

```bash
idf.py build
idf.py -p [PORT] flash monitor
```

## promisc

ESP32 promiscuous mode sniffer.

## GY-511 and MPU6050

GY-511: Need to connect GND to GND, SDA to GPIO21, SCL to GPIO22, VIN to 3V3. <br />
MPU6050: Need to connect GND to GND, SDA to GPIO21, SCL to GPIO22, VCC to 3V3. <br />
Can change GPIO pins in the source code.

tester codes for GY-511 (magnetometer and accelerometer) and MPU6050 (gyroscope and accelerometer). <br />
I used complementary filter for MPU6050. For GY-511, I had a problem getting the heading angle out of the x,y,z raw data. atan(y/x) seems to be not working.

## udp_node

previous try of broadcasting UDP packets.

## PING_PONG

Between M_recv and M_send: Need to connect GPIO4 to GPIO5, GPIO5 to GPIO4, GND to GND. <br />
Between W_recv and W_send: Need to connect GPIO4 to GPIO5, GPIO5 to GPIO4, GND to GND. (same)

Works as follows:

M_send_node is broadcasting raw packets. W_recv_node is in promiscuous mode and can sniff the packets with rssi values. If the RSSI value is high enough, a UART
message is sent to W_send_note to send a raw packet back to M_recv_node. RSSI value is in the UART message, and sent back to M_recv_node. M_recv_node respond to
the RSSI value in the message and change its frequency of LED blinking. (LED part may not work. Not tested)

M_recv and W_recv are in promiscuous mode (WIFI_MODE_NONE), and M_send and W_sned are in AP mode (WIFI_MODE_AP).

Possible improvement: Maybe can reduce the system to 3 nodes by sending UDP packets back to M, so no need to have M_recv_node. The purpose of sending raw packets
is to get RSSI values. But to receive UDP packets, M and W need to be under the same network, which maybe impossible.

## Freq_based

