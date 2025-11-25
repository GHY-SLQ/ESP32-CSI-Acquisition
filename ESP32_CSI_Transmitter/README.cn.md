常见修改参数如下：

信道：CONFIG_LESS_INTERFERENCE_CHANNEL

2.4GHz的信道选择为信道1到13

5GHz的信道选择为36，40，44，48，52，56，60，64。此外，149，153，157，161，165这五个信道未测试是否可用。

带宽：CONFIG_WIFI_BANDWIDTH

发送频率：CONFIG_SEND_FREQUENCY

Mac：CONFIG_CSI_SEND_MAC

辅助信道：带宽设置为40MHz时生效。可以修改wifi_init()函数中的esp_wifi_set_channel()函数。默认为取主信道之下的信道，比如信道12的辅助信道是信道8。
