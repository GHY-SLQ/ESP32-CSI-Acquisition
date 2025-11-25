# 基于ESP32的CSI采集平台 [View instructions in English](./README.md)

## 概述

这是一个基于ESP32的CSI采集平台。其中，ESP32发射端和ESP32接收端A都基于Espressif的Github仓库，链接如下：https://github.com/espressif/esp-csi/tree/master

**整体采集过程如下图所示：**
<img width="3346" height="583" alt="New Design Receiving CSI" src="https://github.com/user-attachments/assets/fc2f51b5-d1dd-4028-a71a-a22c7ba4d684" />


ESP32发射端，ESP32接收端A和ESP32接收端B的程序基于ESP-IDF v5.5.1，其他版本可能出现意外问题。

程序兼容ESP32,ESP32S3和ESP32C5三个系列。

如果采集2.4GHz的CSI数据，ESP32发射端可以用上述3款ESP32的任意一种；接收端A推荐使用ESP32S3；接收端B推荐使用ESP32S3或ESP32C5。

如果采集5GHz的CSI数据。此时，ESP32发送端和接收端A都必须使用ESP32C5，接收端B可以使用ESP32S3或ESP32C5。

PC解析端可以采集CSI数据，并导出成.csv文件。同时，文件中还会包含大部分能够读取的数据，如RSSI，mac等。

**需要注意的是，如果使用ESP32C5作为发射端和接收端A，那么部分数据受限于API配置无法读取，在ESP32接收端A的程序中对于不可读取的数据赋值0处理。**

关于CSI数据的排列方式，请参考Espressif的官方文档，链接如下：https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32c5/api-guides/wifi.html#wi-fi-multiple-antennas

## 运行效果

以下效果图均为串口和终端打印截图。

### ESP32发送端

<img width="747" height="119" alt="Result01" src="https://github.com/user-attachments/assets/fbe9e2b9-3166-416c-8da9-8dde3bb88b98" />

### ESP32接收端A

<img width="1210" height="226" alt="Result02" src="https://github.com/user-attachments/assets/600da260-d89b-4e9a-b302-6f75a6c79fbb" />

### ESP32接收端B

<img width="1048" height="401" alt="Result03" src="https://github.com/user-attachments/assets/3ff62b38-b293-4204-aa7e-19d9b0d6537f" />

### PC解析

程序会在代码所处文件夹下创建received_CSI文件夹，csv文件以程序开始运行时刻命名。表格效果请参考Example/example.csv

<img width="1375" height="654" alt="Result04" src="https://github.com/user-attachments/assets/249d122d-0819-4cb8-b7cf-51eecb608731" />

