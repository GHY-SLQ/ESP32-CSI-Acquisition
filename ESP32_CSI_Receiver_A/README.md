# CSI_RECEIVER_A

Channel: CONFIG_LESS_INTERFERENCE_CHANNEL

The channel selections for 2.4GHz are channels 1 through 13.

For 5GHz, channels 36, 40, 44, 48, 52, 56, 60, and 64 are selected; in addition, channels 149, 153, 157, 161, and 165 are not tested for availability.

Bandwidth: CONFIG_WIFI_BANDWIDTH

Mac: CONFIG_CSI_SEND_MAC

Second channel: effective when the bandwidth is set to 40MHz. You can modify the esp_wifi_set_channel() function in the wifi_init() function. The default is to take the channel below the main channel, for example, the second channel for channel 12 is channel 8.

**The channel, bandwidth, mac and second channel should be the same as the ESP32 transmitter.**

SPI pins vary depending on the ESP32 model. It is recommended to use the default configuration in the code when using ESP32S3 and ESP32C5. Please refer to the datasheet for other models. The datasheet can be downloaded at the following address: https://www.espressif.com/en/products/socs
