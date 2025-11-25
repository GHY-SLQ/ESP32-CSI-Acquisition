# CSI_RECEIVER_B [中文](README.cn.md)

**You need to set up the network connection manually, including: WIFI_SSID, WIFI_PASS, PC_IP and PC_PORT.**

Among them, WIFI_SSID and WIFI_PASS are usually configured using a cell phone hotspot or router. You need to ensure that the ESP32 receiver B and the host computer are connected to the same network. Query the IP address of the host computer as the PC_IP of the ESP32 receiver B program. The default setting of the PC port is 5000, and you need to open the UDP receive setting of the corresponding port in the firewall in Windows. In macOS, there is no need to set the port, but there will be a prompt to open the port when you run pc_parse.py for the first time.

**To open the corresponding port in the Windows system, you can refer to the following link: https://learn.microsoft.com/en-us/sql/reporting-services/report-server/configure-a-firewall-for-report-server-access?view=sql-server-ver17**

SPI pins vary depending on the ESP32 model. The code defines PIN_MOSI, PIN_MISO, PIN_SCLK, and PIN_CS. MISO is not used, and no wiring is actually required.

It is recommended to use the default configuration in the code when using ESP32S3 and ESP32C5. Please refer to the datasheet for other models. The datasheet can be downloaded at the following address: https://www.espressif.com/en/products/socs
