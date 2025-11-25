/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
    This program is based on the repository at 
    https://github.com/espressif/esp-csi/tree/master/examples/get-started/csi_recv
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h> 

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_attr.h" 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "csi_frame.h" /* 统一帧头 / unified CSI frame header */

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
#define CONFIG_FORCE_GAIN 1
#if CONFIG_IDF_TARGET_ESP32C5
#define CSI_FORCE_LLTF 0
#endif
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
#define CONFIG_GAIN_CONTROL 1
#endif

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_recv";

/* ========= SPI Master 硬件参数（按需要修改引脚/时钟）/ SPI Master Hardware Parameters (modify pins/clocks as needed) ========= */
#define SPI_HOST_USE SPI2_HOST /* VSPI on ESP32 */
#if CONFIG_IDF_TARGET_ESP32S3
#define PIN_MOSI GPIO_NUM_13
#define PIN_MISO -1
#define PIN_SCLK GPIO_NUM_12
#define PIN_CS GPIO_NUM_10
#elif CONFIG_IDF_TARGET_ESP32C5
#define PIN_MOSI GPIO_NUM_2
#define PIN_MISO -1
#define PIN_SCLK GPIO_NUM_6
#define PIN_CS GPIO_NUM_10
#else
#define PIN_MOSI GPIO_NUM_23
#define PIN_MISO -1
#define PIN_SCLK GPIO_NUM_18
#define PIN_CS GPIO_NUM_5
#endif
#define SPI_CLK_HZ (40 * 1000 * 1000) // SCLK 
#define SPI_Q_SIZE 8
#define CSI_QUEUE_LEN 64
#define SPI_CS_PRE_CYCLES 16  // CS 置低到首个 SCLK 之间预留的时钟数 / Number of clocks reserved between CS low and first SCLK
#define SPI_CS_POST_CYCLES 16 // 最后一个 SCLK 到 CS 置高的时钟数 / Number of clocks from last SCLK to CS high

static spi_device_handle_t s_spi = NULL; // SPI 设备句柄(主机上的从设备) / SPI device handle
static QueueHandle_t s_csi_q = NULL;     // CSI 数据队列句柄 / Queue handle for CSI data
static volatile uint32_t s_seq = 0;      // CSI 帧发送序号 / Sequence number for CSI frames

/* =============================================================== */

// 统计量：Wi-Fi 回调收到/入队/丢弃的包数
// Counters: total received in callback / enqueued / dropped (queue full)
static volatile uint32_t g_csi_cb_total = 0;   // 回调收到 / received in callback
static volatile uint32_t g_csi_enq_total = 0;  // 入队成功 / enqueued successfully
static volatile uint32_t g_csi_drop_total = 0; // 入队失败(队列满) / enqueue failed (queue full)

// SPI 发送统计：总字节数与总帧数
// SPI TX statistics: total bytes and packets sent
static volatile uint64_t g_spi_bytes_total = 0; // 累计 SPI 发送字节 / total SPI bytes sent
static volatile uint32_t g_spi_pkts_total = 0;  // 累计 SPI 发送帧数 / total SPI frames sent

// 自旋锁，用于保护 CSI 相关共享数据
// Spinlock to protect shared CSI data
static portMUX_TYPE s_csi_mux = portMUX_INITIALIZER_UNLOCKED;

// 原始 CSI 数据结构（打包 Wi-Fi 元信息 + CSI 缓冲）
// Raw CSI structure: Wi-Fi meta info + CSI buffer
typedef struct
{
    // —— 元信息（来自 wifi_csi_info_t / rx_ctrl） / meta info from wifi_csi_info_t / rx_ctrl ——
    uint8_t mac[6];   // 发送端 MAC / sender MAC
    int8_t rssi;      // 信号强度 / RSSI
    uint8_t rate;     // 速率 / data rate
    uint8_t sig_mode; // 信号模式 / signal mode
    uint8_t mcs;      // MCS 编号 / MCS index
    uint8_t cwb;      // 信道带宽 / channel width

    uint8_t smoothing;         // 平滑标志 / smoothing flag
    uint8_t not_sounding;      // sounding 标志 / not sounding
    uint8_t aggregation;       // 聚合标志 / aggregation flag
    uint8_t stbc;              // STBC 标志 / STBC flag
    uint8_t fec_coding;        // FEC 编码类型 / FEC coding
    uint8_t sgi;               // 是否 SGI / short GI flag
    int8_t noise_floor;        // 噪声底 / noise floor
    uint8_t ampdu_cnt;         // AMPDU 计数 / AMPDU count
    uint8_t channel;           // 主信道 / primary channel
    uint8_t secondary_channel; // 副信道 / secondary channel

    uint32_t timestamp; // 时间戳 / timestamp
    uint8_t ant;        // 天线号 / antenna index

    uint8_t agc_gain; // ★ AGC 增益 / AGC gain
    uint8_t fft_gain; // ★ FFT 增益 / FFT gain

    uint16_t sig_len;           // 有效数据长度 / signal length
    uint16_t rx_state;          // 接收状态 / RX state
    uint8_t first_word_invalid; // 首字无效标志 / first word invalid flag

    // 业务相关字段 / application-level field
    uint32_t rx_id; // 接收序号 / RX sequence ID

    // —— 原始 CSI 缓冲 / raw CSI buffer ——
    uint16_t len;      // CSI 数据长度 / CSI data length (info->len)
    uint8_t buf[2048]; // CSI 数据缓存 / CSI data buffer (enough size)
} csi_raw_t;

/* ================== 判断CPU核心 / Decide CPU core ==================== */
#if (portNUM_PROCESSORS == 1)
#define CORE_RXUDP 0 // 单核：RX 任务绑在 core 0 / single core: RX task on core 0
#define CORE_STATS 0 // 单核：统计任务也绑 core 0（或用 tskNO_AFFINITY）
                     // single core: stats task on core 0 (or tskNO_AFFINITY)
#else
#define CORE_RXUDP 1              // 双核：RX 任务绑在 core 1 / dual core: RX task on core 1
#define CORE_STATS tskNO_AFFINITY // 双核：统计任务不固定核心 / stats task no fixed core
#endif

typedef struct
{
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    unsigned : 32; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    unsigned : 16; /**< reserved */
    unsigned fft_gain : 8;
    unsigned agc_gain : 8;
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    signed : 8;    /**< reserved */
    unsigned : 24; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
} wifi_pkt_rx_ctrl_phy_t;

#if CONFIG_FORCE_GAIN
/**
 * @brief Enable/disable automatic fft gain control and set its value
 * @param[in] force_en true to disable automatic fft gain control
 * @param[in] force_value forced fft gain value
 */
extern void phy_fft_scale_force(bool force_en, uint8_t force_value);

/**
 * @brief Enable/disable automatic gain control and set its value
 * @param[in] force_en true to disable automatic gain control
 * @param[in] force_value forced gain value
 */
extern void phy_force_rx_gain(int force_en, int force_value);
#endif

/* ================= SPI Master initialization ================= */
/* 预先把 SPI CS 拉高，避免上电瞬间误触发从机
 * Pre-claim SPI CS as GPIO and set it HIGH to avoid spurious selects at power-up.
 */
static void spi_master_preclaim_cs_high(void)
{
    gpio_config_t spi_cs = {
        // CS 引脚配置 / CS pin configuration
        .pin_bit_mask = 1ULL << PIN_CS, // 选择 CS 引脚 / select CS pin
        .mode = GPIO_MODE_OUTPUT,       // 设为输出模式 / output mode
        .pull_up_en = 0,                // 不使能上拉 / no pull-up
        .pull_down_en = 0,              // 不使能下拉 / no pull-down
        .intr_type = GPIO_INTR_DISABLE  // 不用中断 / no interrupt
    };
    gpio_config(&spi_cs);      // 应用配置 / apply configuration
    gpio_set_level(PIN_CS, 1); // 先让 CS 处于空闲高电平 / keep CS idle HIGH
}

/* 初始化 SPI 主机（总线 + 从设备）
 * Initialize SPI master (bus + attached slave device).
 */
static void spi_init_master(void)
{
    // 1) 先把 CS 抢占为 GPIO 输出并拉高，避免上电窗口的误触发
    // 1) Pre-claim CS as GPIO and set HIGH to avoid glitches at power-up
    spi_master_preclaim_cs_high();

    // 2) 初始化 SPI 总线配置
    // 2) Configure and initialize the SPI bus
    spi_bus_config_t spi_buscfg = {
        .mosi_io_num = PIN_MOSI, // MOSI 引脚号 / MOSI pin
        .miso_io_num = PIN_MISO, // MISO 引脚号(此处未用) / MISO pin (unused = -1)
        .sclk_io_num = PIN_SCLK, // SCLK 引脚号 / SCLK pin
        .quadwp_io_num = -1,     // 不用四线模式 / no quad WP
        .quadhd_io_num = -1,     // 不用四线模式 / no quad HD
        .max_transfer_sz = CSI_HDR_SIZE + CSI_MAX_PAYLOAD + 4,
        // 单次最大传输字节数 / max bytes per transfer
    };
    // SPI_HOST_USE: 选择使用哪个 SPI 控制器
    // SPI_HOST_USE: which SPI host to use
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_USE, &spi_buscfg, SPI_DMA_CH_AUTO));

    // 3) 降低 SCLK/MOSI/CS 的驱动强度，减小过冲和串扰
    //    （2 或 3 都可；数值越大驱动越强，这里用 2 比较“温和”）
    // 3) Reduce drive strength to mitigate overshoot / crosstalk
    gpio_set_drive_capability(PIN_SCLK, GPIO_DRIVE_CAP_2);
    gpio_set_drive_capability(PIN_MOSI, GPIO_DRIVE_CAP_2);
    gpio_set_drive_capability(PIN_CS, GPIO_DRIVE_CAP_2);

    // 4) 配置并挂载 SPI 从设备
    // 4) Configure and attach the SPI slave device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLK_HZ,           // SPI 时钟频率 / SPI clock frequency
        .mode = 0,                              // SPI 模式 0 / SPI mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = PIN_CS,                 // 硬件 CS 引脚 / HW CS pin
        .queue_size = SPI_Q_SIZE,               // 事务队列深度 / transaction queue size
        .cs_ena_pretrans = SPI_CS_PRE_CYCLES,   // CS 置低到第一个 SCLK 的预留周期 / CS setup cycles
        .cs_ena_posttrans = SPI_CS_POST_CYCLES, // 最后一个 SCLK 到 CS 置高的周期 / CS hold cycles
        //.flags          = 0, // 保持默认 / keep default
        //.flags = SPI_DEVICE_HALFDUPLEX, // 半双工可选 / optional half-duplex
    };
    // s_spi: 返回的设备句柄，用于后续 transmit
    // s_spi: device handle used later for transmit
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USE, &devcfg, &s_spi));
}

/* ================= Wi-Fi initialization ================= */
static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // 添加国家码
    // Add Country Code
    wifi_country_t country_code = {.cc = "AU", .schan = 1, .nchan = 14, .policy = WIFI_COUNTRY_POLICY_MANUAL};
    ESP_ERROR_CHECK(esp_wifi_set_country(&country_code));

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
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    else
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
#else
    if (CONFIG_WIFI_BANDWIDTH == WIFI_BW_HT20)
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    else
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
}

#if CONFIG_IDF_TARGET_ESP32C5
static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate = CONFIG_ESP_NOW_RATE,
        .ersu = false,
        .dcm = false};
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));
}
#endif

/*====================== Wi-Fi CSI receive callback =========================*/
/* Wi-Fi CSI 接收回调
 * Wi-Fi CSI receive callback.
 * 由 esp_wifi_set_csi_rx_cb 注册，在每次收到一帧 CSI 时被调用。
 * Registered via esp_wifi_set_csi_rx_cb, called for every received CSI frame.
 */
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    // 基本合法性检查：info 为空或 buf 为空则直接返回
    // Basic sanity check: return if info or info->buf is NULL.
    if (!info || !info->buf)
        return;

    // 只保留来自指定 MAC (CONFIG_CSI_SEND_MAC) 的 CSI，其它发射端直接丢弃
    // Only keep CSI from CONFIG_CSI_SEND_MAC; drop others.
    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6))
        return;

    // 回调收到包计数 +1（用自旋锁保护）
    // Increase callback counter with spinlock protection.
    portENTER_CRITICAL(&s_csi_mux);
    g_csi_cb_total++;
    portEXIT_CRITICAL(&s_csi_mux);

    // 将整个 info 强制转换为 PHY 控制结构，用于读取 agc_gain/fft_gain
    // Cast info to PHY control struct to access agc_gain / fft_gain.
    wifi_pkt_rx_ctrl_phy_t *phy_info = (wifi_pkt_rx_ctrl_phy_t *)info;
    // rx_ctrl 指针，包含 RSSI、rate、channel 等常规字段
    // Pointer to rx_ctrl with RSSI, rate, channel, etc.
    const wifi_pkt_rx_ctrl_t *rx = &info->rx_ctrl;

#if CONFIG_GAIN_CONTROL
    // 统计前 100 帧的增益并求平均，用于后面锁定增益
    // Accumulate AGC/FFT gain for first 100 frames to compute average.
    static int s_count = 0;
    static uint16_t agc_sum = 0, fft_sum = 0;
    static uint8_t agc_force = 0, fft_force = 0;
    if (s_count < 100)
    {
        agc_sum += phy_info->agc_gain;
        fft_sum += phy_info->fft_gain;
    }
    else if (s_count == 100)
    {
        // 取平均值，作为“锁定增益”的目标
        // Take average as target fixed gain.
        agc_force = agc_sum / 100;
        fft_force = fft_sum / 100;
#if CONFIG_FORCE_GAIN
        // 调用底层 API，关闭自动增益并强制使用该值
        // Call PHY APIs to disable auto gain and force these values.
        phy_fft_scale_force(1, fft_force);
        phy_force_rx_gain(1, agc_force);
#endif
        ESP_LOGI(TAG, "fft_force %d, agc_force %d", fft_force, agc_force);
    }
#endif

    // 从 payload 中解析 rx_id（与发送端对齐的业务序号）
    // Extract rx_id from payload (application-level sequence number).
    uint32_t rx_id = 0;
    if (info->payload && info->payload_len >= 19)
    {
        memcpy(&rx_id, info->payload + 15, sizeof(rx_id));
    }

    // 构造一条原始 CSI 记录，初始化为 0
    // Build a raw CSI record, zero-initialized.
    csi_raw_t item = {0};
    memcpy(item.mac, info->mac, 6); // 发射端 MAC / sender MAC

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    // C5/C6 上 rx_ctrl 暴露字段较少，有什么用什么，剩下的用 0 或由配置推导
    // On C5/C6 rx_ctrl exposes fewer fields: fill what we can, others from config or zero.
    item.rssi = rx->rssi;
    item.rate = rx->rate;
    item.noise_floor = rx->noise_floor;
    item.channel = rx->channel;
    item.timestamp = rx->timestamp;
    item.sig_len = rx->sig_len;
    item.rx_state = rx->rx_state;

    // 以下字段未公开，填 0 或根据带宽配置推断
    // Fields not exposed: fill 0 or derive from bandwidth config.
    item.sig_mode = 0;
    item.mcs = 0; // 无公开 MCS 字段 / no public MCS field
    item.cwb = (CONFIG_WIFI_5G_BANDWIDTHS == WIFI_BW_HT40) ? 1 : 0;
    item.smoothing = 0;
    item.not_sounding = 0;
    item.aggregation = 0;
    item.stbc = 0;
    item.fec_coding = 0;
    item.sgi = 0;
    item.ampdu_cnt = 0;
    // secondary_channel 未公开，根据配置推断 20MHz/40MHz
    // secondary_channel not exposed; infer from bandwidth config.
    item.secondary_channel = (CONFIG_WIFI_2G_BANDWIDTHS == WIFI_BW_HT20 || CONFIG_WIFI_5G_BANDWIDTHS == WIFI_BW_HT20) ? 0 : 1;
    item.ant = 0;

#else
    // 在 S3 等芯片上，rx_ctrl 字段比较完整，直接拷贝
    // On S3-like targets rx_ctrl is complete, copy directly.
    item.rssi = rx->rssi;
    item.rate = rx->rate;
    item.sig_mode = rx->sig_mode;
    item.mcs = rx->mcs;
    item.cwb = rx->cwb;
    item.smoothing = rx->smoothing;
    item.not_sounding = rx->not_sounding;
    item.aggregation = rx->aggregation;
    item.stbc = rx->stbc;
    item.fec_coding = rx->fec_coding;
    item.sgi = rx->sgi;
    item.noise_floor = rx->noise_floor;
    item.ampdu_cnt = rx->ampdu_cnt;
    item.channel = rx->channel;
    item.secondary_channel = rx->secondary_channel;
    item.timestamp = rx->timestamp;
    item.ant = rx->ant;
    item.sig_len = rx->sig_len;
    item.rx_state = rx->rx_state;
#endif

    // 记录 PHY 层的 AGC / FFT 增益
    // Store PHY-layer AGC / FFT gain.
    item.agc_gain = phy_info->agc_gain;
    item.fft_gain = phy_info->fft_gain;

    // 其它 CSI 相关标志与业务字段
    // Other CSI-related flags and application fields.
    item.first_word_invalid = info->first_word_invalid;
    item.rx_id = rx_id;

    // 复制 CSI 数据本体，注意长度上限
    // Copy CSI payload, with upper bound check.
    item.len = info->len;
    if (item.len > sizeof(item.buf))
        item.len = sizeof(item.buf);
    memcpy(item.buf, info->buf, item.len);

    // 将打包好的 CSI 结构入队，交给 SPI 发送任务处理
    // Enqueue the packed CSI struct for the SPI TX task.
    if (s_csi_q)
    {
        if (xQueueSend(s_csi_q, &item, 0) == pdTRUE)
        {
            // 入队成功计数 +1
            // Enqueue success counter.
            portENTER_CRITICAL(&s_csi_mux);
            g_csi_enq_total++;
            portEXIT_CRITICAL(&s_csi_mux);
        }
        else
        {
            // 队列满：丢包计数 +1
            // Queue full: drop counter.
            portENTER_CRITICAL(&s_csi_mux);
            g_csi_drop_total++;
            portEXIT_CRITICAL(&s_csi_mux);
        }
    }

#if CONFIG_GAIN_CONTROL
    // 更新计数，用于前面的前 100 帧统计
    // Increase sample count for the first-100-frames averaging.
    s_count++;
#endif
}

/* 统计任务：每秒打印一次 CSI / SPI 相关统计信息
 * Stats task: print CSI / SPI statistics every second.
 */
static void stats_task(void *arg)
{
    // 上一秒“回调/入队/丢弃”计数快照
    // Snapshot of last-second counters for:
    //  - cb   : total CSI packets seen in callback
    //  - enq  : total CSI packets successfully enqueued
    //  - drop : total CSI packets dropped (queue full)
    uint32_t last_cb = 0;   // 上一秒时 g_csi_cb_total 的值 / previous g_csi_cb_total
    uint32_t last_enq = 0;  // 上一秒时 g_csi_enq_total 的值 / previous g_csi_enq_total
    uint32_t last_drop = 0; // 上一秒时 g_csi_drop_total 的值 / previous g_csi_drop_total

    // ★ 上一秒 SPI 统计快照
    // SPI statistics snapshot from the previous second:
    //  - last_spi_bytes: total SPI bytes sent up to last second
    //  - last_spi_pkts : total SPI frames sent up to last second
    uint64_t last_spi_bytes = 0; // 上一秒时 g_spi_bytes_total / previous g_spi_bytes_total
    uint32_t last_spi_pkts = 0;  // 上一秒时 g_spi_pkts_total  / previous g_spi_pkts_total

    // 本任务生命周期内的“最小栈高水位”，用于观察是否接近栈溢出
    // Minimum stack high-water mark during this task's lifetime
    // (in words), useful to check if the stack is close to overflow.
    UBaseType_t wm_min = 0xFFFFFFFF;

    for (;;)
    {
        // 每 1000 ms 执行一次统计周期
        // Run statistics once every 1000 ms.
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 当前时刻的全局计数值（总量）
        // Current values of global counters (cumulative totals).
        uint32_t cb, enq, drop;
        uint64_t spi_bytes; // 当前累计 SPI 发送字节数 / current total SPI bytes
        uint32_t spi_pkts;  // 当前累计 SPI 发送帧数 / current total SPI packets

        // 加锁读取，避免在读的过程中被其他任务修改
        // Read under spinlock to avoid races while other tasks update them.
        portENTER_CRITICAL(&s_csi_mux);
        cb = g_csi_cb_total;           // 回调收包总数 / total callback CSI count
        enq = g_csi_enq_total;         // 入队成功总数 / total enqueued count
        drop = g_csi_drop_total;       // 队列满丢弃总数 / total dropped count
        spi_bytes = g_spi_bytes_total; // SPI 发送字节总数 / total SPI bytes sent
        spi_pkts = g_spi_pkts_total;   // SPI 发送帧总数 / total SPI packets sent
        portEXIT_CRITICAL(&s_csi_mux);

        // —— 将“总量”转换成“每秒增量” —— //
        // Convert cumulative totals to per-second increments.

        // cb_ps: 当前这一秒内，回调收到多少 CSI 包
        // cb_ps: CSI packets received in callback per second.
        uint32_t cb_ps = cb - last_cb;
        // enq_ps: 当前这一秒内，有多少 CSI 包成功入队
        // enq_ps: CSI packets enqueued per second.
        uint32_t enq_ps = enq - last_enq;
        // drop_ps: 当前这一秒内，有多少 CSI 包因队列满被丢弃
        // drop_ps: CSI packets dropped (queue full) per second.
        uint32_t drop_ps = drop - last_drop;

        // 更新“上一秒快照”，供下一轮计算差值
        // Update snapshots for the next iteration.
        last_cb = cb;
        last_enq = enq;
        last_drop = drop;

        // SPI 侧同理：计算本秒 SPI 的吞吐量
        // Same idea for SPI: compute per-second throughput.

        // spi_Bps: 本秒实际推上 SPI 总线的字节数（Byte/s）
        // spi_Bps: SPI throughput in bytes per second.
        uint64_t spi_Bps = spi_bytes - last_spi_bytes;
        // spi_Pps: 本秒通过 SPI 发送的帧数（packet/s）
        // spi_Pps: SPI frames per second.
        uint32_t spi_Pps = spi_pkts - last_spi_pkts;

        // 更新 SPI 的上一秒快照
        // Update SPI snapshots.
        last_spi_bytes = spi_bytes;
        last_spi_pkts = spi_pkts;

        // 当前 CSI 队列里的消息数，反映队列深度/积压程度
        // Current number of CSI items in the queue, showing backlog / depth.
        UBaseType_t q_now = (s_csi_q ? uxQueueMessagesWaiting(s_csi_q) : 0);

        // 当前任务的栈剩余高水位（单位是 word，不是字节）
        // Current stack high-water mark for this task (in words, not bytes).
        UBaseType_t wm = uxTaskGetStackHighWaterMark(NULL);
        if (wm < wm_min)
            wm_min = wm; // 记录运行到目前为止的最小值 / track the minimum value so far

        // 打印整体统计：
        //   - CSI 收/入队/丢包速率（pkt/s）
        //   - SPI 吞吐（B/s）与帧率（pkt/s）
        //   - 当前队列长度 q_now，当前发送序号 s_seq
        //   - 当前栈水位 wm 以及历史最小值 wm_min
        //
        // Log overall statistics:
        //   - CSI receive/enqueue/drop rates (pkt/s)
        //   - SPI throughput (B/s) and frame rate (pkt/s)
        //   - Queue depth q_now, current seq s_seq
        //   - Current stack watermark wm and minimum wm_min.
        ESP_LOGI("STAT",
                 "CSI=%u pkt/s, enq=%u pkt/s, drop=%u pkt/s | "
                 "SPI=%" PRIu64 " B/s, %u pkt/s | q=%u, seq=%u | wm=%u words (min=%u)",
                 cb_ps, enq_ps, drop_ps,
                 spi_Bps, spi_Pps,
                 (unsigned)q_now, (unsigned)s_seq,
                 (unsigned)wm, (unsigned)wm_min);
    }
}

/* ================= CSI 功能配置（保留你原分支） ================= */
static void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

#if CONFIG_IDF_TARGET_ESP32C5
    wifi_csi_config_t csi_config = {
        .enable = true,
        .acquire_csi_legacy = false,
        .acquire_csi_force_lltf = CSI_FORCE_LLTF,
        .acquire_csi_ht20 = true,
        .acquire_csi_ht40 = true,
        .acquire_csi_vht = false,
        .acquire_csi_su = false,
        .acquire_csi_mu = false,
        .acquire_csi_dcm = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg = 0,
        .dump_ack_en = false,
        .reserved = false};
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable = true,
        .acquire_csi_legacy = false,
        .acquire_csi_ht20 = true,
        .acquire_csi_ht40 = true,
        .acquire_csi_su = false,
        .acquire_csi_mu = false,
        .acquire_csi_dcm = false,
        .acquire_csi_beamformed = false,
        .acquire_csi_he_stbc = 2,
        .val_scale_cfg = false,
        .dump_ack_en = false,
        .reserved = false};
#else
    wifi_csi_config_t csi_config = {
        .lltf_en = false,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = true,
        .manu_scale = false,
        .shift = false,
    };
#endif
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

/* ================= SPI 发送任务（单帧装下整条 CSI）/ SPI Transmit Task (single frame loads the entire CSI) ================= */
// SPI 发送帧缓冲，放在 DMA 可访问区域：大小 = 帧头 + 最大 CSI 负载
// SPI TX frame buffer in DMA-capable memory: size = header + CSI_MAX_PAYLOAD
static DMA_ATTR csi_frame_t spi_tx_frame; // 全局 DMA 缓冲 / global DMA buffer

/* SPI 发送任务：从队列取出原始 CSI，打包成 csi_frame_t，经 SPI 发给 B 端
 * SPI TX task: fetch raw CSI from queue, pack into csi_frame_t, and send via SPI.
 */
static void spi_tx_task(void *arg)
{
    // 队列中取出的原始 CSI 数据（包含元信息 + buf）
    // Raw CSI item from queue (meta info + buffer).
    csi_raw_t csi_raw_in;

    // 持续从队列阻塞式读取，直到任务结束（通常不会返回）
    // Block on the queue and process items indefinitely.
    while (xQueueReceive(s_csi_q, &csi_raw_in, portMAX_DELAY))
    {
        // 防溢出保护：CSI 长度超过协议上限就截断到 CSI_MAX_PAYLOAD
        // Overflow protection: clamp CSI length to CSI_MAX_PAYLOAD.
        if (csi_raw_in.len > CSI_MAX_PAYLOAD)
        {
            ESP_LOGW(TAG, "CSI len=%u > CSI_MAX_PAYLOAD=%u, truncating",
                     (unsigned)csi_raw_in.len, (unsigned)CSI_MAX_PAYLOAD);
            csi_raw_in.len = CSI_MAX_PAYLOAD;
        }

        // spi_tx_p 指向全局 DMA 帧缓冲 spi_tx_frame
        // spi_tx_p points to the global DMA frame buffer spi_tx_frame.
        csi_frame_t *spi_tx_p = &spi_tx_frame;

        // ===== 填写帧头通用字段 =====
        // ===== Fill common frame header fields =====
        spi_tx_p->magic = CSI_MAGIC;            // 帧标识 / magic number
        spi_tx_p->seq = s_seq;                  // 发送序号 / sequence number
        spi_tx_p->total_len = csi_raw_in.len;   // 整条 CSI 实际长度 / total CSI length
        spi_tx_p->offset = 0;                   // 不做分片，偏移恒为 0 / no fragmentation, offset = 0
        spi_tx_p->payload_len = csi_raw_in.len; // 本帧净荷长度 / payload length in this frame

        // ===== 拷贝元信息（MAC、RSSI、调制参数等） =====
        // ===== Copy meta info (MAC, RSSI, modulation params, etc.) =====
        memcpy(spi_tx_p->mac, csi_raw_in.mac, 6);
        spi_tx_p->rssi = csi_raw_in.rssi;
        spi_tx_p->rate = csi_raw_in.rate;
        spi_tx_p->sig_mode = csi_raw_in.sig_mode;
        spi_tx_p->mcs = csi_raw_in.mcs;
        spi_tx_p->cwb = csi_raw_in.cwb;
        spi_tx_p->smoothing = csi_raw_in.smoothing;
        spi_tx_p->not_sounding = csi_raw_in.not_sounding;
        spi_tx_p->aggregation = csi_raw_in.aggregation;
        spi_tx_p->stbc = csi_raw_in.stbc;
        spi_tx_p->fec_coding = csi_raw_in.fec_coding;
        spi_tx_p->sgi = csi_raw_in.sgi;
        spi_tx_p->noise_floor = csi_raw_in.noise_floor;
        spi_tx_p->ampdu_cnt = csi_raw_in.ampdu_cnt;
        spi_tx_p->channel = csi_raw_in.channel;
        spi_tx_p->secondary_channel = csi_raw_in.secondary_channel;
        spi_tx_p->timestamp = csi_raw_in.timestamp;
        spi_tx_p->ant = csi_raw_in.ant;

        // 记录 PHY 层 AGC / FFT 增益
        // Store PHY-layer AGC / FFT gain.
        spi_tx_p->agc_gain = csi_raw_in.agc_gain; // ★ 新增 / added field
        spi_tx_p->fft_gain = csi_raw_in.fft_gain; // ★ 新增 / added field

        // 其它 CSI 相关字段
        // Other CSI-related fields.
        spi_tx_p->sig_len = csi_raw_in.sig_len;
        spi_tx_p->rx_state = csi_raw_in.rx_state;
        spi_tx_p->csi_len = csi_raw_in.len; // 与 payload_len 一致 / equal to payload_len
        spi_tx_p->first_word_invalid = csi_raw_in.first_word_invalid;
        spi_tx_p->rx_id = csi_raw_in.rx_id; // 业务序号 / application-level ID

        // ===== 拷贝 CSI 复数数据到 payload 区域 =====
        // ===== Copy CSI data into payload area =====
        memcpy(spi_tx_p->payload, csi_raw_in.buf, csi_raw_in.len);

        // 只发送“帧头 + 实际 CSI 数据”，不发送未使用的尾部
        // Only send "header + actual CSI" and ignore unused tail.
        size_t len_bytes = CSI_HDR_SIZE + csi_raw_in.len;

        // 为了满足 SPI 4 字节对齐，计算需要补齐的 0 字节数 pad ∈ [0..3]
        // To align total length to 4 bytes, compute pad ∈ [0..3].
        size_t pad = (4 - (len_bytes & 3)) & 3;
        if (pad)
            memset(spi_tx_p->payload + csi_raw_in.len, 0, pad); // 在 payload 末尾补 0
                                                                // zero-pad at end of payload

        // 准备 SPI 事务描述符
        // Prepare SPI transaction descriptor.
        spi_transaction_t t = {0};
        t.length = (len_bytes + pad) * 8; // 单位是 bit，因此 ×8；已按 4 字节对齐
                                          // length in bits; includes 4-byte alignment pad
        t.tx_buffer = spi_tx_p;           // 发送缓冲指针 / pointer to TX buffer

        // 通过 SPI 同步发送一帧 CSI 数据
        // Transmit the CSI frame synchronously via SPI.
        esp_err_t e = spi_device_transmit(s_spi, &t);
        if (e == ESP_OK)
        {
            // 累加这次真正推上 SPI 总线的“有效字节数”（不含 CS 预/后时钟）
            // Accumulate the actual number of bytes put on the SPI bus this time.
            size_t tx_bytes = len_bytes + pad;
            portENTER_CRITICAL(&s_csi_mux);
            g_spi_bytes_total += tx_bytes; // 总字节计数 / total bytes counter
            g_spi_pkts_total += 1;         // 帧计数 / total packets counter
            portEXIT_CRITICAL(&s_csi_mux);
        }
        else
        {
            ESP_LOGW(TAG, "SPI tx err=%d", e); // 打印 SPI 发送错误 / log SPI TX error
        }

        // 一条 CSI 帧发送完成，序号自增，用于调试/丢包分析
        // One CSI frame sent, increment sequence for debugging / loss analysis.
        s_seq++;
    }
}

/* ================= app_main：初始化顺序 / initialization flow ================= */
void app_main()
{
    /* ---------- 1. 初始化 NVS（非易失性存储） ----------
     * 先尝试 nvs_flash_init()，如果发现页满或版本不匹配，则擦除后重建。
     *
     * 1. Init NVS (non-volatile storage).
     * Try nvs_flash_init(); if pages are full or version changed,
     * erase NVS and re-init.
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ---------- 2. 先创建 CSI 队列 ----------
     * 保证从开启 CSI 到回调触发时，队列已经存在，可以安全入队。
     *
     * 2. Create CSI queue first.
     * Ensure the queue is ready before CSI RX callback starts pushing items.
     */
    s_csi_q = xQueueCreate(CSI_QUEUE_LEN, sizeof(csi_raw_t));

    /* ---------- 3. 初始化 Wi-Fi ----------
     * 包括：模式、国家代码、信道、带宽、速率、MAC 等配置。
     *
     * 3. Initialize Wi-Fi.
     * Includes mode, country, channel, bandwidth, data rate, MAC, etc.
     */
    wifi_init();

#if CONFIG_IDF_TARGET_ESP32C5
    /* ---------- 3.1（可选）ESP-NOW 初始化 ----------
     * 这里保留你原来的 ESP-NOW 初始化结构，虽然当前方案不直接用，
     * 但保留不影响正常 CSI 功能。
     *
     * 3.1 (optional) ESP-NOW init.
     * Kept for compatibility with your original structure; harmless even if unused.
     */
    esp_now_peer_info_t peer = {
        .channel = CONFIG_LESS_INTERFERENCE_CHANNEL,       // 使用的信道 / operating channel
        .ifidx = WIFI_IF_STA,                              // STA 接口 / STA interface
        .encrypt = false,                                  // 不加密 / no encryption
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, // 广播地址 / broadcast addr
    };
    wifi_esp_now_init(peer);
#endif

    /* ---------- 4. 开启 CSI 功能 ----------
     * 设置 CSI 配置、注册回调、使能 CSI 采集。
     *
     * 4. Enable CSI.
     * Configure CSI, register RX callback, and enable CSI collection.
     */
    wifi_csi_init();

    /* ---------- 5. 初始化 SPI 主机并启动发送任务 ----------
     * spi_init_master(): 配置 SPI 总线和从设备（连接到 B 端）。
     * spi_tx_task     : 从 CSI 队列取数据，打包成帧，经 SPI 发送给 B 端。
     *
     * 5. Init SPI master and start TX task.
     * spi_init_master(): setup SPI bus and slave device to ESP32-B.
     * spi_tx_task     : read CSI from queue, pack into frame, send via SPI.
     */
    spi_init_master();
    xTaskCreatePinnedToCore(
        spi_tx_task,   // 任务函数 / task function
        "spi_tx_task", // 任务名 / task name
        8192,          // 栈大小 / stack size
        NULL,          // 参数 / task parameter
        8,             // 优先级（略高）/ priority (relatively high)
        NULL,          // 任务句柄（此处不需要）/ task handle (unused)
        CORE_RXUDP     // 绑定的 CPU 核心 / target core
    );

    /* ---------- 6. 启动统计打印任务 ----------
     * 周期性打印 CSI / SPI 速率、队列深度、栈水位等，用于调试与性能观测。
     *
     * 6. Start statistics task.
     * Periodically logs CSI/SPI rates, queue depth, stack watermark, etc.
     */
    xTaskCreatePinnedToCore(
        stats_task,   // 任务函数 / task function
        "stats_task", // 任务名 / task name
        4096,         // 栈大小 / stack size
        NULL,         // 参数 / task parameter
        4,            // 优先级（比 TX 低）/ lower priority than TX
        NULL,         // 任务句柄 / task handle
        CORE_STATS    // 绑定的 CPU 核心 / target core
    );
}