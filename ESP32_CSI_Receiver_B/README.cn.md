# ESP32接收端B [English](README.md)

**需要手动设置网络连接，包括：WIFI_SSID，WIFI_PASS，PC_IP和PC_PORT。**

其中WIFI_SSID和WIFI_PASS一般使用手机热点或路由器配置。需要保证ESP32接收端B和上位机连接在同一网络内。查询上位机的IP地址，作为ESP32接收端B的PC_IP。PC端口默认设置为5000，在Windows操作系统中需要在防火墙中打开对应端口的UDP接收设置。在macOS中无需额外设置端口，但是在第一次运行pc_parse.py时会出现端口打开的提示。

**打开Windows系统对应端口可以参考下面的链接：https://learn.microsoft.com/en-us/sql/reporting-services/report-server/configure-a-firewall-for-report-server-access?view=sql-server-ver17**

SPI引脚根据ESP32型号不同有所区别。代码中定义了PIN_MOSI，PIN_MISO，PIN_SCLK和PIN_CS。其中MISO未使用，实际无需接线。

在使用ESP32S3和ESP32C5时推荐使用代码中的默认配置，其他型号请自行查阅数据手册。数据手册下载地址如下：https://www.espressif.com/en/products/socs
