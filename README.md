# ESP32-CSI-Acquisition

This is a CSI acquisition platform based on ESP32. The ESP32 transmitter and ESP32 receiver A are based on Espressif's Github repository, which is linked as follows: https://github.com/espressif/esp-csi/tree/master

All components are shown below: 
<img width="3346" height="583" alt="New Design Receiving CSI" src="https://github.com/user-attachments/assets/02c17b6f-1e22-4922-a300-20c8baec0fcd" />

The programs for ESP32 Transmitter, ESP32 Receiver A and ESP32 Receiver B are based on ESP-IDF v5.5.1, other versions may have unexpected problems.

The program is compatible with ESP32, ESP32S3 and ESP32C5 series.

If collecting 2.4GHz CSI data, ESP32 transmitter can use any one of the above three ESP32; receiver A is recommended to use ESP32S3; receiver B is recommended to use ESP32S3 or ESP32C5. If the CSI data of 5GHz is collected. At this time, both ESP32 transmitter and receiver A must use ESP32C5; receiver B can use ESP32S3 or ESP32C5.

The PC parsing end can capture the CSI data and export it to a .csv file. At the same time, the file will contain most of the data that can be read, such as RSSI, mac and so on.

It should be noted that, if you use ESP32C5 as the transmitter and receiver A, then some of the data can not be read due to the API configuration. In the ESP32 receiver A program, for the unreadable data is assigned to 0 processing.

For more information on how CSI data are arranged, please refer to the official Espressif documentation at the following link: https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32c5/api-guides/wifi.html#wi-fi-multiple-antennas
