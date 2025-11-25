这是一个基于ESP32的CSI采集平台。其中，ESP32发射端和ESP32接收端A都基于Espressif的Github仓库，链接如下：https://github.com/espressif/esp-csi/tree/master

整体采集过程如下图所示：

<img width="3346" height="583" alt="New Design Receiving CSI" src="https://github.com/user-attachments/assets/31938c37-e6a9-418b-a23e-142d233ba882" />

ESP32发射端，ESP32接收端A和ESP32接收端B的程序基于ESP-IDF v5.5.1，其他版本可能出现意外问题。

程序兼容ESP32,ESP32S3和ESP32C5三个系列。

如果采集2.4GHz的CSI数据，ESP32发射端可以用上述3款ESP32的任意一种；接收端A推荐使用ESP32S3；接收端B推荐使用ESP32S3或ESP32C5。如果采集5GHz的CSI数据。此时，ESP32发送端和接收端A都必须使用ESP32C5，接收端B可以使用ESP32S3或ESP32C5。

PC解析端可以采集CSI数据，并导出成.csv文件。同时，文件中还会包含大部分能够读取的数据，如RSSI，mac等。

需要注意的是，如果使用ESP32C5作为发射端和接收端A，那么部分数据受限于API配置无法读取，在ESP32接收端A的程序中对于不可读取的数据赋值0处理。

关于CSI数据的排列方式，请参考Espressif的官方文档，链接如下：https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32c5/api-guides/wifi.html#wi-fi-multiple-antennas
