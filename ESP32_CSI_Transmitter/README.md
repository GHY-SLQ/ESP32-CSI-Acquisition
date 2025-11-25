# CSI_TRANSMITTER

Commonly modified parameters are listed below:

Channel: CONFIG_LESS_INTERFERENCE_CHANNEL

Channel selection for 2.4GHz is channels 1 through 13

For 5GHz, the channel selection is 36, 40, 44, 48, 52, 56, 60, 64. In addition, the five channels 149, 153, 157, 161, 165 are not tested for availability.

Bandwidth: CONFIG_WIFI_BANDWIDTH

Transmit frequency: CONFIG_SEND_FREQUENCY

Mac: CONFIG_CSI_SEND_MAC

Second channel: effective when the bandwidth is set to 40MHz. You can modify the esp_wifi_set_channel() function in the wifi_init() function. By default, it selects the channel under the primary channel. For example, channel 12's second channel is channel 8.
