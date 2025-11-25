# ESP32接收端A [English](./README.md)

## 常见修改参数

信道：CONFIG_LESS_INTERFERENCE_CHANNEL

2.4GHz的信道选择为信道1到13

5GHz的信道选择为36，40，44，48，52，56，60，64。此外，149，153，157，161，165这五个信道未测试是否可用。

带宽：CONFIG_WIFI_BANDWIDTH

Mac：CONFIG_CSI_SEND_MAC

辅助信道：带宽设置为40MHz时生效。可以修改wifi_init()函数中的esp_wifi_set_channel()函数。默认为取主信道之下的信道，比如信道12的辅助信道是信道8。

**信道，带宽，mac和辅助信道应该与ESP32发送端保持一致。**

SPI引脚根据ESP32型号不同有所区别。代码中定义了PIN_MOSI，PIN_MISO，PIN_SCLK和PIN_CS。其中MISO未使用，实际无需接线。

在使用ESP32S3和ESP32C5时推荐使用代码中的默认配置，其他型号请自行查阅数据手册。数据手册下载地址如下：https://www.espressif.com/en/products/socs
