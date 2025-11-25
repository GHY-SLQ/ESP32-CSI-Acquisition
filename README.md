# ESP32-CSI-Acquisition [中文](./README.cn.md)

## Introduction

This is a CSI acquisition platform based on ESP32. The ESP32 transmitter and ESP32 receiver A are based on Espressif's Github repository, which is linked as follows: https://github.com/espressif/esp-csi/tree/master

**All components are shown below:**

<img width="3346" height="583" alt="New Design Receiving CSI" src="https://github.com/user-attachments/assets/28a94bf8-d1bb-4933-858b-d843c5ce9d97" />

The programs for ESP32 Transmitter, ESP32 Receiver A and ESP32 Receiver B are based on ESP-IDF v5.5.1, other versions may have unexpected problems.

The program is compatible with the ESP32, ESP32S3 and ESP32C5 series.

If collecting 2.4GHz CSI data, the ESP32 transmitter can use any one of the above three ESP32s; receiver A is recommended to use ESP32S3; receiver B is recommended to use ESP32S3 or ESP32C5. If collecting 5GHz CSI data, both the ESP32 transmitter and receiver A must use ESP32C5; receiver B can use ESP32S3 or ESP32C5.

The PC parsing end can capture the CSI data and export it to a .csv file. At the same time, the file will contain most of the data that can be read, such as RSSI, mac and so on.

**It should be noted that, if you use ESP32C5 as the transmitter and receiver A, then some of the data can not be read due to the API configuration. In the ESP32 receiver A program, the unreadable data is assigned to 0 processing.**

For more information on how CSI data are arranged, please refer to the official Espressif documentation at the following link: https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32c5/api-guides/wifi.html#wi-fi-multiple-antennas

## Operational Effect

The following screenshots are serial port and terminal printouts.

### ESP32 transmitter

<img width="747" height="119" alt="Result01" src="https://github.com/user-attachments/assets/854fd159-a482-4550-9649-b6fa4497ec60" />

### ESP32 receiver A

<img width="1210" height="226" alt="Result02" src="https://github.com/user-attachments/assets/fc611e0e-b612-4809-a485-39894d8a79c9" />

### ESP32 receiver B

<img width="1048" height="401" alt="Result03" src="https://github.com/user-attachments/assets/487961cd-f033-49fd-baf9-f1f361ea242a" />

### PC parsing

The program creates the received_CSI folder under the folder where the code is located, and the csv file is named after the moment when the program starts running. Please refer to Example/example.csv for table effects.

<img width="1375" height="654" alt="Result04" src="https://github.com/user-attachments/assets/09ff50aa-d2f1-4baf-b043-ff00245f90ec" />

