/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
    This program is based on the repository at 
    https://github.com/espressif/esp-csi/tree/master/examples/get-started/csi_send
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"

#include "esp_timer.h"

#define CONFIG_LESS_INTERFERENCE_CHANNEL 12

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
#define CONFIG_WIFI_BAND_MODE WIFI_BAND_MODE_5G_ONLY
#define CONFIG_WIFI_2G_BANDWIDTHS WIFI_BW_HT40
#define CONFIG_WIFI_5G_BANDWIDTHS WIFI_BW_HT40
#define CONFIG_WIFI_2G_PROTOCOL WIFI_PROTOCOL_11N
#define CONFIG_WIFI_5G_PROTOCOL WIFI_PROTOCOL_11N
#define CONFIG_ESP_NOW_PHYMODE WIFI_PHY_MODE_HT40
#else
#define CONFIG_WIFI_BANDWIDTH WIFI_BW_HT40
#endif

#define CONFIG_ESP_NOW_RATE WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_SEND_FREQUENCY 2000

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_send";

/* --------- 发送定时器相关 / TX timer related --------- */

/* 高精度发送定时器句柄，用于周期性触发发送回调
 * High-resolution esp_timer handle used to trigger the periodic send callback.
 */
static esp_timer_handle_t s_send_timer = NULL;

/* ESP-NOW 发送的目标 MAC 地址
 * 这里设置为广播地址 FF:FF:FF:FF:FF:FF，表示对所有节点广播发送
 * ESP-NOW peer MAC address.
 * Currently set to the broadcast address FF:FF:FF:FF:FF:FF, meaning send to all peers.
 */
static uint8_t s_peer_addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* 已发送数据包计数（使用 volatile，保证在中断/任务间读取是可见的）
 * Global TX counter, incremented each time a frame is sent.
 * Marked volatile so updates are visible across tasks/ISR contexts.
 */

static volatile uint32_t s_tx_cnt = 0;

/* 周期性发送回调函数
 * Periodic send callback function.
 *
 * 说明 / Notes:
 * - 此回调运行在 ESP_TIMER_TASK 上下文中，可以安全地调用 esp_now_send()。
 *   The callback runs in the ESP_TIMER_TASK context, where it is safe to call esp_now_send().
 * - 建议只做“轻量级”工作，避免大量计算或频繁日志打印。
 *   Keep the work lightweight; avoid heavy computation or frequent logging.
 */

static void periodic_send_cb(void *arg)
{
    /* 取当前计数值并自增，用作要发送的数据内容
     * Get the current counter value and post-increment it;
     * this value is used as the payload to be sent.
     */
    uint32_t cnt = s_tx_cnt++;

    /* 通过 ESP-NOW 发送计数值到指定的 MAC 地址
     * Send the counter value via ESP-NOW to the configured peer MAC address.
     *
     * 参数说明 / Parameters:
     * - s_peer_addr: 目标设备的 MAC 地址（这里是广播地址）
     *                Destination MAC address (broadcast in this example).
     * - (const uint8_t *)&cnt: 要发送的数据指针
     *                          Pointer to the payload (the counter).
     * - sizeof(cnt): 数据长度（这里是 4 字节）
     *                Length of the payload in bytes (4 bytes for uint32_t).
     */
    esp_err_t e = esp_now_send(s_peer_addr, (const uint8_t *)&cnt, sizeof(cnt));

    /* 此处忽略返回值，只是避免未使用变量告警。
     * 如果需要统计发送失败次数或调试，可在此根据 e 做计数或打印日志，
     * 但要注意不要在定时器回调中频繁 ESP_LOG*。
     *
     * We ignore the return value here to avoid an unused-variable warning.
     * If you need to collect statistics (e.g., count failures) or debug,
     * you can inspect 'e' here, but avoid heavy logging from this callback.
     */
    (void)e;
}
/* --------------------------------------- */

static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_country_t c = {.cc = "AU", .schan = 1, .nchan = 14, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    ESP_ERROR_CHECK(esp_wifi_set_country(&c));

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE);
    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
        .ghz_5g = CONFIG_WIFI_5G_PROTOCOL};
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));
    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
        .ghz_5g = CONFIG_WIFI_5G_BANDWIDTHS};
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));
#else
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, CONFIG_WIFI_BANDWIDTH));
    ESP_ERROR_CHECK(esp_wifi_start());

#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, CONFIG_ESP_NOW_RATE));
#endif

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    if ((CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_2G_ONLY && CONFIG_WIFI_2G_BANDWIDTHS == WIFI_BW_HT20) || (CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_5G_ONLY && CONFIG_WIFI_5G_BANDWIDTHS == WIFI_BW_HT20))
    {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    }
    else
    {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#else
    if (CONFIG_WIFI_BANDWIDTH == WIFI_BW_HT20)
    {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    }
    else
    {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#endif
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
}

static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate = CONFIG_ESP_NOW_RATE,
        .ersu = false,
        .dcm = false};
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));
#endif
}

void app_main(void)
{
    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Wi-Fi 初始化 */
    wifi_init();

    /* ESP-NOW 初始化 */
    esp_now_peer_info_t peer = {
        .channel = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };
    wifi_esp_now_init(peer);
    memcpy(s_peer_addr, peer.peer_addr, 6); // 记住广播地址，供定时器回调使用

    ESP_LOGI(TAG, "================ CSI SEND ================");
    ESP_LOGI(TAG, "wifi_channel: %d, send_frequency: %d, mac: " MACSTR,
             CONFIG_LESS_INTERFERENCE_CHANNEL, CONFIG_SEND_FREQUENCY, MAC2STR(CONFIG_CSI_SEND_MAC));

    /* 用高精度周期定时器以 CONFIG_SEND_FREQUENCY Hz 发送
     * Use a high-resolution periodic timer to send at CONFIG_SEND_FREQUENCY Hz.
     */
    const esp_timer_create_args_t targs = {
        /* 定时器回调函数指针
         * Pointer to the callback function that will be called on every timer tick.
         */
        .callback = &periodic_send_cb,

        /* 传递给回调函数的参数，这里为 NULL
         * Argument passed to the callback function; set to NULL here.
         */
        .arg = NULL,

        /* 指定回调在 ESP_TIMER_TASK 上下文中执行（非中断上下文）
         * Callback is dispatched in the ESP_TIMER_TASK context (not in ISR),
         * which allows calling functions that不能在中断里调用，例如 esp_now_send()。
         */
        .dispatch_method = ESP_TIMER_TASK, // 在 timer task 上回调（非中断）

        /* 定时器名称，用于调试和日志
         * Name of the timer, useful for debugging/logging.
         */
        .name = "espnow_tx"};

    /* 创建高精度定时器实例，句柄保存在 s_send_timer 中
     * Create the high-resolution timer instance; handle is stored in s_send_timer.
     */
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_send_timer));

    /* 启动周期定时器
     * Start the periodic timer.
     *
     * 第二个参数单位是微秒(us)，这里使用：
     *   1000000ULL / CONFIG_SEND_FREQUENCY
     * 例如：CONFIG_SEND_FREQUENCY = 2000 Hz 时：
     *   周期 = 1,000,000 us / 2000 = 500 us
     *
     * The second argument is the period in microseconds (us):
     *   1000000ULL / CONFIG_SEND_FREQUENCY
     * For example, if CONFIG_SEND_FREQUENCY = 2000 Hz,
     *   period = 1,000,000 us / 2000 = 500 us.
     */
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_send_timer,
                                             1000000ULL / CONFIG_SEND_FREQUENCY)); // 500 us @ 2 kHz

    /* 不再需要 while+usleep 循环；当前任务可以结束
     * No need for a while+usleep loop anymore; this task can terminate now.
     *
     * 定时器和其他系统任务会在后台继续运行，周期性回调 periodic_send_cb 完成发送。
     * The timer and other system tasks will keep running in the background,
     * and periodic_send_cb will continue to be called periodically for sending.
     */
    vTaskDelete(NULL);
}
