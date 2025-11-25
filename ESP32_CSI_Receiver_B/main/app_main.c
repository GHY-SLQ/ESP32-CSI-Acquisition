/* ESP32-B: SPI Slave -> Wi-Fi STA -> UDP to PC (single-CSI-per-SPI-frame) */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <stddef.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_attr.h" // 提供 DMA_ATTR / WORD_ALIGNED_ATTR

#include "csi_frame.h" // CSI_MAGIC / CSI_MAX_PAYLOAD / csi_frame_t / csi_v2_hdr_t

/* ==================== 用户网络参数 user-network parameter ==================== */
#define WIFI_SSID "iPhone"
#define WIFI_PASS "slq135790"
#define PC_IP "172.20.10.10"

#define PC_PORT 5000

/* ================== 判断CPU核心 / Decide CPU core ==================== */
#if (portNUM_PROCESSORS == 1)
#define CORE_RXUDP 0 // 单核：RX 任务绑在 core 0 / single core: RX task on core 0
#define CORE_STATS 0 // 单核：统计任务也绑 core 0（或用 tskNO_AFFINITY）
                     // single core: stats task on core 0 (or tskNO_AFFINITY)
#else
#define CORE_RXUDP 1              // 双核：RX 任务绑在 core 1 / dual core: RX task on core 1
#define CORE_STATS tskNO_AFFINITY // 双核：统计任务不固定核心 / stats task no fixed core
#endif

/* ============= SPI 引脚 / SPI pinout ============== */
#define SPI_HOST_USE SPI2_HOST
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

#define PIN_READY 4

#define SPI_QUEUE_SIZE 8

/* ==================== 日志 TAG & 事件位 / Log TAG & event bits ==================== */
static const char *TAG = "B_SPI_UDP";         // 日志 TAG 名称 / log tag name
static EventGroupHandle_t s_wifi_event_group; // Wi-Fi 事件组句柄 / event group for Wi-Fi state
#define WIFI_CONNECTED_BIT BIT0               // Wi-Fi 已连接位 / bit: Wi-Fi connected
#define IP_GOT_BIT BIT1                       // 已获取 IP 位 / bit: IP address obtained

/* ==================== UDP 句柄 / UDP handles ==================== */
static int s_udp_sock = -1;            // UDP socket（-1 表示未创建）/ UDP socket (-1 = not created)
static struct sockaddr_in s_pc_addr;   // PC 端地址信息 / destination address (PC)
static volatile bool g_has_ip = false; // 是否已经拿到 IP / whether STA has got IP

/* ==================== UDP 合包参数 / UDP coalescing parameters ==================== */
#define UDP_MTU 1472 // UDP 负载 MTU（避开以太网 MTU 1500）/ UDP payload MTU

// 向上按 4 字节对齐 / round up to 4-byte alignment
#define ROUND4(x) (((x) + 3) & ~3)

// SPI 接收最大长度：csi_frame_t + 若干填充，DMA 要求按 4 字节对齐
// Max SPI RX length: sizeof(csi_frame_t) plus padding, 4-byte aligned for DMA.
#define SPI_MAX_RX ROUND4(sizeof(csi_frame_t) + 4) // DMA 长度按 4 字节对齐 / DMA length aligned to 4 bytes

// 拥塞回退窗口（微秒）：检测到 UDP 拥塞后暂时停止发送
// Congestion backoff window (us): pause sending for this duration on congestion.
#define CONGEST_US 12000
// UDP 拼包的时间窗口（微秒）：两次 flush 之间至少间隔，避免太多小包
// UDP coalescing time window (us): minimum interval between flushes to avoid tiny packets.
#define FLUSH_US 20000

// 统计：UDP 合包阶段主动丢包数量（例如缓冲放不下时）
// Statistics: number of packets dropped during UDP coalescing (buffer full, etc.).
static volatile uint32_t g_udp_drop_total = 0;

// 拥塞退避的截止时间（单调递增时间戳, us）
// Deadline until which we stay in congestion backoff (monotonic time in us).
static volatile uint64_t s_congest_until_us = 0;

// UDP 发送缓冲区：用于拼接多个 csi_v2_hdr + payload 再一次性发送
// UDP send buffer: used to pack multiple csi_v2_hdr + payload blocks into one UDP packet.
static uint8_t s_udp_buf[UDP_MTU];
static size_t s_udp_fill = 0;         // 当前缓冲中已使用字节数 / current filled bytes in s_udp_buf
static uint64_t s_udp_last_flush = 0; // 上次 flush 的时间戳(us) / timestamp of last flush in us

// 临时大缓冲：用于单次超大帧（>UDP_MTU）直接 send，一般很少用到
// Temporary large buffer: for single large frame (>UDP_MTU) direct send; rarely used.
static uint8_t s_big_tmp[sizeof(csi_v2_hdr_t) + CSI_MAX_PAYLOAD];

/* ==================== 统计 / Statistics ==================== */
static portMUX_TYPE s_stat_mux = portMUX_INITIALIZER_UNLOCKED;
// 自旋锁，用来保护统计变量的并发访问
// Spinlock to protect concurrent access to stats variables.

static volatile uint32_t g_spi_pkts_total = 0;          // SPI 收到的 CSI 包总数
                                                        // Total number of CSI packets received over SPI.
static volatile uint32_t g_spi_bytes_total = 0;         // CSI 总字节数（只算 payload 部分）
                                                        // Total CSI bytes (payload only).
static volatile uint32_t g_udp_pkts_total = 0;          // UDP 发送的包总数
                                                        // Total number of UDP packets sent.
static volatile uint32_t g_udp_bytes_total = 0;         // UDP 发送的字节总数
                                                        // Total bytes sent via UDP.
static volatile uint32_t g_udp_err_total = 0;           // UDP 发送失败的总次数
                                                        // Total number of UDP send failures.
static volatile uint32_t g_udp_errno_eagain_total = 0;  // errno=EAGAIN/EWOULDBLOCK 计数
                                                        // Count of errno = EAGAIN / EWOULDBLOCK.
static volatile uint32_t g_udp_errno_enobufs_total = 0; // errno=ENOBUFS 计数
                                                        // Count of errno = ENOBUFS.
static volatile uint32_t g_udp_errno_other_total = 0;   // 其它 errno 的计数
                                                        // Count of all other errno values.

// 线程安全的统计加法（放在最前面，先于任何调用）
// Thread-safe add helper for stats (defined before any use).
static inline void stat_add(volatile uint32_t *p, uint32_t v)
{
    portENTER_CRITICAL(&s_stat_mux); // 进入临界区 / enter critical section
    *p += v;                         // 累加统计值 / accumulate stat
    portEXIT_CRITICAL(&s_stat_mux);  // 退出临界区 / exit critical section
}

// —— errno 直方图（0..255 足够）
// errno histogram (0..255 is sufficient).
static volatile uint32_t g_udp_errno_hist[256] = {0};

static inline void stat_errno(int err)
{
    if (err < 0)
        err = -err;                          // 确保为正数 / normalize to positive
    if ((unsigned)err < 256)                 // 只统计 0..255 范围内的 errno
        stat_add(&g_udp_errno_hist[err], 1); // 对应 errno 桶 +1 / increment histogram bucket
}

// —— errno 名称（常见值），仅用于打印
// errno name mapping for common values, used only for logging.
static const char *errno_name(int e)
{
    switch (e)
    {
#if EAGAIN != EWOULDBLOCK
    case EWOULDBLOCK:
#endif
    case EAGAIN:
        return "EAGAIN";
    case ENOBUFS:
        return "ENOBUFS";
    case ENETDOWN:
        return "ENETDOWN";
    case ENETUNREACH:
        return "ENETUNREACH";
    case EHOSTUNREACH:
        return "EHOSTUNREACH";
    case EMSGSIZE:
        return "EMSGSIZE";
    case ECONNRESET:
        return "ECONNRESET";
    case EPIPE:
        return "EPIPE";
    case EINVAL:
        return "EINVAL";
    case ENOMEM:
        return "ENOMEM";
    default:
        return NULL; // 其它就打印数字 / for others, just print the numeric code
    }
}

// —— 背压告警阈值（按需调小/调大）
// Backpressure warning thresholds (tune as needed).
#define BACKPRESSURE_ERR_TH 50 // 本秒 UDP 发送错误数 >= 50 触发告警
                               // Warn if UDP send errors per second >= 50.
#define BACKPRESSURE_DROP_TH 5 // 本秒 UDP 主动丢弃数  >= 5 触发告警
                               // Warn if actively dropped packets per second >= 5.

/* ==================== Wi-Fi 事件回调 / Wi-Fi event callback ==================== */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    // STA 启动事件：开始尝试连接 AP
    // STA start event: begin connecting to the configured AP.
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "WiFi STA started, connecting...");
        esp_wifi_connect();
    }
    // STA 断开事件：清除连接标志，打印原因，并自动重连
    // STA disconnected event: clear connection flags, log reason, and reconnect.
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        const wifi_event_sta_disconnected_t *disc = (const wifi_event_sta_disconnected_t *)data;
        g_has_ip = false;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | IP_GOT_BIT);
        ESP_LOGW(TAG, "WiFi disconnected, reason=%d. Reconnecting...", disc ? disc->reason : -1);
        esp_wifi_connect();
    }
    // 获取 IP 事件：设置 has_ip 标志，置位事件组，打印 IP/GW/Mask
    // Got-IP event: set has_ip flag, set event bits, and log IP/GW/Mask.
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        const ip_event_got_ip_t *ev = (const ip_event_got_ip_t *)data;
        g_has_ip = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT | IP_GOT_BIT);
        ESP_LOGI(TAG, "Got IP: " IPSTR "  GW: " IPSTR "  MASK: " IPSTR,
                 IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.gw), IP2STR(&ev->ip_info.netmask));
    }
}

/* ==================== Wi-Fi STA 初始化 / Wi-Fi STA initialization ==================== */
static void wifi_sta_init(void)
{
    // 创建一个事件组，用于跟踪 Wi-Fi 连接 / 获取 IP 等状态
    // Create an event group to track Wi-Fi connection / IP acquisition state.
    s_wifi_event_group = xEventGroupCreate();

    // 初始化 TCP/IP 网络栈与默认事件循环
    // Initialize TCP/IP stack and default event loop.
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 创建默认的 Wi-Fi STA 网络接口
    // Create default Wi-Fi STA (station) network interface.
    esp_netif_create_default_wifi_sta();

    // 使用默认参数初始化 Wi-Fi 驱动
    // Initialize Wi-Fi driver with default configuration.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册 Wi-Fi 事件和 IP 事件的回调处理函数
    // Register event handlers for Wi-Fi and IP events.
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // 配置 STA 模式下的 SSID 和密码
    // Configure SSID and password for STA mode.
    wifi_config_t sta = {0};
    strncpy((char *)sta.sta.ssid, WIFI_SSID, sizeof(sta.sta.ssid));
    strncpy((char *)sta.sta.password, WIFI_PASS, sizeof(sta.sta.password));
    // 要求至少 WPA2-PSK 安全级别
    // Require at least WPA2-PSK security.
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    // 设置为 STA 模式并应用配置
    // Set Wi-Fi to STA mode and apply configuration.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));

    // 关闭省电模式，尽量保证链路稳定和吞吐
    // Disable power-save mode for better stability and throughput.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // 启动 Wi-Fi，之后由事件回调负责连接、重连等逻辑
    // Start Wi-Fi; connection / reconnection is handled in the event callback.
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ==================== UDP 初始化 / UDP initialization ==================== */
static void udp_init(void)
{
    // 如果之前已经创建过 socket，则先关掉，重新创建一个干净的
    // If a socket already exists, close it first to recreate a fresh one.
    if (s_udp_sock != -1)
    {
        close(s_udp_sock);
        s_udp_sock = -1;
    }

    // 创建 UDP socket（IPv4 + DGRAM）
    // Create UDP socket (IPv4 + datagram).
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    // 创建失败，打印错误并返回
    // If creation fails, log error and return.
    if (s_udp_sock < 0)
    {
        ESP_LOGE(TAG, "socket() failed");
        return;
    }

    // 设置 IP 头部 TOS 字段：DSCP=46(EF)，用于 QoS 优先级（语音/实时类似）
    // Set IP TOS: DSCP = 46 (EF), ECN = 0, for higher QoS priority (e.g., voice-like traffic).
    int tos = 0xB8; // DSCP=46(EF)<<2，ECN=0
    if (setsockopt(s_udp_sock, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) != 0)
    {
        ESP_LOGW(TAG, "setsockopt(IP_TOS) failed, errno=%d", errno);
    }

    // 允许端口复用（重启时避免 TIME_WAIT 等造成 bind 问题）
    // Allow address reuse (avoid bind issues on restart).
    int yes = 1;
    setsockopt(s_udp_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    // 调大发送缓冲区，降低 ENOBUFS 概率，提升高吞吐下的容忍度
    // Enlarge send buffer to reduce ENOBUFS under high throughput.
    int sndbuf = 128 * 1024;
    (void)setsockopt(s_udp_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    // 配置 PC 端的目标地址和端口
    // Configure destination address (PC) and port.
    memset(&s_pc_addr, 0, sizeof(s_pc_addr));
    s_pc_addr.sin_family = AF_INET;
    s_pc_addr.sin_port = htons(PC_PORT);
    inet_pton(AF_INET, PC_IP, &s_pc_addr.sin_addr.s_addr);

    // 使用 connect() 绑定默认目标，这样之后 send() 就可以不用每次填 sockaddr
    // Use connect() to fix the default peer; later send() can omit sockaddr.
    if (connect(s_udp_sock, (struct sockaddr *)&s_pc_addr, sizeof(s_pc_addr)) < 0)
    {
        ESP_LOGW(TAG, "connect() failed, errno=%d", errno);
    }

    // UDP 就绪，打印目标 IP 和端口
    // UDP ready, log target IP and port.
    ESP_LOGI(TAG, "UDP ready -> %s:%d", PC_IP, PC_PORT);
}

/* ==================== SPI 从机初始化 / SPI slave initialization ==================== */

// 判断某个 GPIO 是否支持内部上拉（这里简单认为 0~33 都可以）
// Check if a GPIO can use internal pull-up (here we simply treat 0..33 as valid).
static inline bool can_use_internal_pu(gpio_num_t io) { return (io >= 0) && (io <= 33); }

// 判断某个 GPIO 是否具有输出能力（这里同样简单认为 0~33 都可以）
// Check if a GPIO is output-capable (also treat 0..33 as valid).
static inline bool is_output_capable(gpio_num_t io) { return (io >= 0) && (io <= 33); }

static void spi_slave_init(void)
{
    // SPI 总线配置：指定 MOSI / MISO / SCLK 引脚和最大传输长度
    // SPI bus configuration: assign MOSI / MISO / SCLK and max transfer size.
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1, // 未使用的引脚设为 -1 / unused pins set to -1
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_RX, // 单次最大接收长度 / max RX size per transfer
    };

    // SPI 从机配置：模式 0、CS 引脚、队列深度等
    // SPI slave device configuration: mode 0, CS pin, queue size, etc.
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,                    // SPI 模式 0 / SPI mode 0
        .spics_io_num = PIN_CS,       // 从机片选引脚 / slave CS pin
        .queue_size = SPI_QUEUE_SIZE, // 队列中允许挂起的事务个数 / number of queued transactions
        .flags = 0,                   // 默认标志 / default flags
    };

    // 初始化 SPI 从机，使用 DMA（自动选择通道）
    // Initialize SPI slave with DMA (SPI_DMA_CH_AUTO picks a DMA channel).
    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST_USE, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));

    // READY 引脚（可选）：拉高表示 B 端已经准备好收 SPI 数据
    // Optional READY pin: pull high to indicate B-side is ready for SPI traffic.
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_READY,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io);
    gpio_set_level(PIN_READY, 1); // 置 1 表示“就绪” / set to 1 = "ready"

    // 对 CS/SCLK/MOSI 做内部上拉配置，减小抖动或悬空风险
    // Configure internal pull-ups for CS/SCLK/MOSI to avoid floating/noise.
    if (can_use_internal_pu(PIN_CS))
        gpio_pullup_en(PIN_CS); // CS 上拉 / pull CS up
    if (can_use_internal_pu(PIN_SCLK))
        gpio_pullup_dis(PIN_SCLK); // SCLK 一般不再上拉，由主机驱动 / usually no pull-up
    if (can_use_internal_pu(PIN_MOSI))
        gpio_pullup_dis(PIN_MOSI); // MOSI 同理 / same for MOSI

#if (PIN_MISO >= 0)
    // 如果 MISO 是可输出引脚，设置稍强一点的驱动能力，保证波形
    // If MISO is output-capable, set a slightly stronger drive strength for better signal.
    if (is_output_capable(PIN_MISO))
        gpio_set_drive_capability(PIN_MISO, GPIO_DRIVE_CAP_3);
#endif

    // 打印最终的 SPI 引脚配置
    // Log final SPI pin configuration.
    ESP_LOGI(TAG, "SPI slave ready (MOSI=%d MISO=%d SCLK=%d CS=%d).",
             PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS);
}

/* ==================== UDP flush（统计版）/ UDP flush with stats ==================== */
static void udp_flush(bool force)
{
    // 如果当前没有待发送的数据，或者 UDP socket 尚未就绪，直接返回
    // If there's no pending data, or UDP socket is not ready, just return.
    if (s_udp_fill == 0 || s_udp_sock < 0)
        return;

    // 当前时间（微秒）
    // Current time in microseconds.
    uint64_t now = esp_timer_get_time();

    // 仍然处于拥塞退避期，且本次不是强制 flush：先不发
    // Still in congestion backoff window and not a forced flush: skip sending.
    if (!force && now < s_congest_until_us)
        return;

    // 距离上次 flush 的时间太短，且非强制 flush：先攒一会儿
    // If too soon since last flush and not forced: keep buffering for a while.
    if (!force && (now - s_udp_last_flush) < FLUSH_US)
        return;

    size_t len = s_udp_fill;
    // 使用非阻塞发送，避免由于内核阻塞导致 SPI 线程被卡住
    // Use non-blocking send to avoid blocking the SPI processing thread.
    ssize_t s = send(s_udp_sock, s_udp_buf, len, MSG_DONTWAIT);

    if (s >= 0)
    {
        // 发送成功：更新 UDP 包数和字节数统计
        // On success: update UDP packet and byte counters.
        stat_add(&g_udp_pkts_total, 1);
        stat_add(&g_udp_bytes_total, (uint32_t)s);

        // 清空发送缓冲区计数，并记录本次 flush 时间
        // Reset buffer fill count and record last flush time.
        s_udp_fill = 0;
        s_udp_last_flush = now;

        // 发送成功，清空拥塞退避窗口（下次可以正常发送）
        // Clear congestion backoff window on success.
        s_congest_until_us = 0;
    }
    else
    {
        // 发送失败：进入拥塞退避窗口（如 ENOBUFS / EAGAIN 等）
        // On failure: enter congestion backoff window (e.g., ENOBUFS / EAGAIN).
        s_congest_until_us = now + CONGEST_US;
        stat_add(&g_udp_err_total, 1); // UDP 发送错误总数 +1 / increment UDP error counter

        int err = errno;
        if (err == EAGAIN || err == EWOULDBLOCK)
        {
            // 发送缓冲暂时满 / socket temporarily cannot send
            stat_add(&g_udp_errno_eagain_total, 1);
        }
        else if (err == ENOBUFS)
        {
            // 内核缓冲区不足 / no buffer space available in kernel
            stat_add(&g_udp_errno_enobufs_total, 1);
        }
        else
        {
            // 其它类型错误 / other error types
            stat_add(&g_udp_errno_other_total, 1);
        }

        // 把本次 errno 记入直方图，用于后续统计分析
        // Record this errno value into the histogram for later analysis.
        stat_errno(err);
        // 如需实时观察失败原因，可打开下行 DEBUG 日志（注意刷屏）
        // For real-time debugging, enable the log below (may be verbose):
        // ESP_LOGD("UDP", "send() failed, errno=%d", err);

        // 注意：这里不会清空 s_udp_fill，保留已有数据，等待下一轮再尝试发送
        // Note: we do NOT clear s_udp_fill; keep buffered data for the next attempt.
    }
}

/* ==================== SPI -> UDP 转发任务 / SPI-to-UDP forwarding task ==================== */
static void spi_rx_and_udp_task(void *arg)
{
    // 8 槽 DMA 接收缓冲：轮转使用的 RX buffer 池
    // 8-slot DMA RX buffers: a pool of rotating RX buffers.
#define RX_SLOTS 8
    static WORD_ALIGNED_ATTR DMA_ATTR uint8_t rxbuf[RX_SLOTS][SPI_MAX_RX];
    static spi_slave_transaction_t tr[RX_SLOTS];

    // 预先把所有事务排入 SPI 从机队列，主机一来就能直接写入
    // Pre-queue all transactions so master can start writing immediately.
    for (int i = 0; i < RX_SLOTS; ++i)
    {
        memset(&tr[i], 0, sizeof(tr[i]));
        tr[i].length = SPI_MAX_RX * 8; // 长度单位是 bit / length is in bits
        tr[i].rx_buffer = rxbuf[i];    // 接收到的数据放在 rxbuf[i] / RX data goes into rxbuf[i]
        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI_HOST_USE, &tr[i], portMAX_DELAY));
    }

    // csi_frame_t 结构中 payload 字段的偏移量，即“帧头”长度
    // Offset of 'payload' in csi_frame_t, i.e. header size.
    const size_t hdr_size = offsetof(csi_frame_t, payload);

    // 上电预热时丢掉前面若干帧，避免脏数据干扰
    // Drop a few warmup frames after power-up to avoid dirty data.
    uint32_t warmup_drops = 0;

    for (;;)
    {
        // 如果当前还没拿到 IP 或 UDP socket 未就绪，就暂时等待
        // If IP is not acquired or UDP socket not ready, wait a bit.
        if (!g_has_ip || s_udp_sock < 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 取回一笔完成的 SPI 从机事务；同时还有其他事务继续挂在队列中
        // Get a completed SPI slave transaction; others remain queued.
        spi_slave_transaction_t *ret;
        ESP_ERROR_CHECK(spi_slave_get_trans_result(SPI_HOST_USE, &ret, portMAX_DELAY));

        uint8_t *rxbuf_ptr = (uint8_t *)ret->rx_buffer;
        size_t rx_bytes = ret->trans_len / 8; // trans_len 是 bit，转成字节
                                              // trans_len is in bits; convert to bytes.

        // 丢弃上电初期的若干脏帧
        // Drop several initial "warm-up" frames right after power-up.
        if (warmup_drops < 4)
        {
            warmup_drops++;
            goto REQUEUE; // 上电脏帧 / power-on dirty frame
        }

        // 帧长度不足以包含头部，直接丢弃
        // If shorter than header size, drop.
        if (rx_bytes < hdr_size)
            goto REQUEUE;

        // 按 csi_frame_t 解释整块 RX 缓冲
        // Interpret RX buffer as csi_frame_t.
        csi_frame_t *f = (csi_frame_t *)rxbuf_ptr;

        // magic 不匹配，说明不是我们定义的 CSI 帧，丢弃
        // If magic mismatches, it's not our CSI frame; drop it.
        if (f->magic != CSI_MAGIC)
            goto REQUEUE;

        // need = 头部 + 实际 payload 长度
        // need = header size + actual payload length.
        size_t need = hdr_size + f->payload_len;
        // pad = 实际接收长度中多出来的“对齐填充”字节数
        // pad = extra alignment padding in the actual RX length.
        size_t pad = (rx_bytes > need) ? (rx_bytes - need) : 0;

        // 基本健壮性检查：payload_len 与 csi_len 一致、长度不越界、pad 不超过 3 字节
        // Basic sanity checks: payload_len == csi_len, size within limit, pad <= 3.
        if (f->payload_len != f->csi_len ||
            f->payload_len > CSI_MAX_PAYLOAD ||
            rx_bytes < need || pad > 3)
            goto REQUEUE;

        // ★ 统计：SPI 成功收到一条完整 CSI 帧
        // Statistics: successfully received one CSI frame via SPI.
        stat_add(&g_spi_pkts_total, 1);
        stat_add(&g_spi_bytes_total, (uint32_t)f->payload_len);

        // === 发送到 PC（根据大小选择“直接发”或“合包发”） ===
        // === Send to PC (direct send if big, or coalesced send if small) ===
        size_t need_udp = sizeof(csi_v2_hdr_t) + f->payload_len;

        // 情况 1：单条 CSI + 头部已经超过 UDP MTU，直接单独发一包
        // Case 1: one CSI + header already exceeds UDP MTU, send as a single big UDP packet.
        if (need_udp > UDP_MTU)
        {
            uint8_t *tmp = s_big_tmp;
            csi_v2_hdr_t *vh = (csi_v2_hdr_t *)tmp;

            // 填写 UDP v2 头部，各字段一一从 SPI 帧复制
            // Fill UDP v2 header, copy fields from SPI frame.
            vh->magic = CSI2_MAGIC;
            vh->hdr_len = CSI2_HDR_LEN;
            vh->ver = 2;
            vh->rx_id = f->rx_id;
            memcpy(vh->mac, f->mac, 6);
            vh->rssi = f->rssi;
            vh->rate = f->rate;
            vh->sig_mode = f->sig_mode;
            vh->mcs = f->mcs;
            vh->cwb = f->cwb;
            vh->smoothing = f->smoothing;
            vh->not_sounding = f->not_sounding;
            vh->aggregation = f->aggregation;
            vh->stbc = f->stbc;
            vh->fec_coding = f->fec_coding;
            vh->sgi = f->sgi;
            vh->noise_floor = f->noise_floor;
            vh->ampdu_cnt = f->ampdu_cnt;
            vh->channel = f->channel;
            vh->secondary_channel = f->secondary_channel;
            vh->timestamp = f->timestamp;
            vh->ant = f->ant;

            vh->agc_gain = f->agc_gain; // 新增 AGC 增益 / newly added AGC gain
            vh->fft_gain = f->fft_gain; // 新增 FFT 增益 / newly added FFT gain

            vh->sig_len = f->sig_len;
            vh->rx_state = f->rx_state;
            vh->csi_len = f->payload_len;
            vh->first_word_invalid = f->first_word_invalid;

            // 把 CSI payload 紧跟在头部后面
            // Place CSI payload right after the header.
            memcpy(tmp + sizeof(*vh), f->payload, f->payload_len);

            // 非阻塞 UDP 发送
            // Non-blocking UDP send.
            ssize_t s = send(s_udp_sock, tmp, sizeof(*vh) + f->payload_len, MSG_DONTWAIT);
            if (s >= 0)
            {
                stat_add(&g_udp_pkts_total, 1);
                stat_add(&g_udp_bytes_total, (uint32_t)s);
                s_congest_until_us = 0; // 清除拥塞退避 / clear backoff
            }
            else
            {
                // 发送失败：进入拥塞退避，并统计 errno
                // On failure: enter congestion backoff and update errno stats.
                s_congest_until_us = esp_timer_get_time() + CONGEST_US;
                stat_add(&g_udp_err_total, 1);

                int err = errno;
                if (err == EAGAIN || err == EWOULDBLOCK)
                {
                    stat_add(&g_udp_errno_eagain_total, 1);
                }
                else if (err == ENOBUFS)
                {
                    stat_add(&g_udp_errno_enobufs_total, 1);
                }
                else
                {
                    stat_add(&g_udp_errno_other_total, 1);
                }

                stat_errno(err); // 记入 errno 直方图 / record in errno histogram
                // 不重试，避免阻塞 SPI 主路径；这类超 MTU 大包本来也很少
                // Do not retry, avoid blocking SPI path; such oversized packets are rare.
            }
        }
        else
        {
            // 情况 2：单帧 <= MTU，走 UDP 拼包路径
            // Case 2: frame fits in MTU, use UDP coalescing path.

            // 1) 先看当前缓冲是否有空间，不够的话先 flush 一次
            // 1) If not enough space in buffer, try flush first.
            if (s_udp_fill + need_udp > UDP_MTU)
            {
                udp_flush(true); // 尽力 flush 当前批次 / force flush current batch
                // 2) 若 flush 后仍然放不下，则清空旧批次并记一次丢弃
                // 2) If still doesn't fit, drop the old batch and start over.
                if (s_udp_fill + need_udp > UDP_MTU)
                {
                    s_udp_fill = 0;
                    stat_add(&g_udp_drop_total, 1);
                }
            }

            // 3) 现在一定放得下，可以安全写入头部和 payload
            // 3) Now it must fit; safely write header and payload.
            csi_v2_hdr_t *vh = (csi_v2_hdr_t *)(s_udp_buf + s_udp_fill);
            vh->magic = CSI2_MAGIC;
            vh->hdr_len = CSI2_HDR_LEN;
            vh->ver = 2;
            vh->rx_id = f->rx_id;
            memcpy(vh->mac, f->mac, 6);
            vh->rssi = f->rssi;
            vh->rate = f->rate;
            vh->sig_mode = f->sig_mode;
            vh->mcs = f->mcs;
            vh->cwb = f->cwb;
            vh->smoothing = f->smoothing;
            vh->not_sounding = f->not_sounding;
            vh->aggregation = f->aggregation;
            vh->stbc = f->stbc;
            vh->fec_coding = f->fec_coding;
            vh->sgi = f->sgi;
            vh->noise_floor = f->noise_floor;
            vh->ampdu_cnt = f->ampdu_cnt;
            vh->channel = f->channel;
            vh->secondary_channel = f->secondary_channel;
            vh->timestamp = f->timestamp;
            vh->ant = f->ant;

            vh->agc_gain = f->agc_gain; // ★ 新增 / added
            vh->fft_gain = f->fft_gain; // ★ 新增 / added

            vh->sig_len = f->sig_len;
            vh->rx_state = f->rx_state;
            vh->csi_len = f->payload_len;
            vh->first_word_invalid = f->first_word_invalid;

            // 先移动填充指针，再复制 payload
            // Advance fill pointer, then copy payload.
            s_udp_fill += sizeof(*vh);
            memcpy(s_udp_buf + s_udp_fill, f->payload, f->payload_len);
            s_udp_fill += f->payload_len;

            // 4) 如果当前处于拥塞退避期：先不 flush，只 requeue SPI buffer
            // 4) If in congestion backoff: don't flush yet, just requeue RX buffer.
            if (esp_timer_get_time() < s_congest_until_us)
            {
                goto REQUEUE;
            }

            // 5) 尝试非阻塞 flush（由 udp_flush 控制时间窗口）
            // 5) Attempt non-blocking flush; udp_flush handles timing window.
            udp_flush(false);
        }

    REQUEUE:
        // 把这块 RX 缓冲重新投入 SPI 队列，保证主机下一帧随时有接收槽位
        // Requeue this RX buffer to ensure next SPI frame can be received immediately.
        ret->length = SPI_MAX_RX * 8;
        ESP_ERROR_CHECK(spi_slave_queue_trans(SPI_HOST_USE, ret, portMAX_DELAY));
    }
}

/* ==================== 每秒统计打印 / Per-second statistics logging task ==================== */
static void stats_task(void *arg)
{
    // 上一次采样时的累计值快照，用来计算“每秒增量”
    // Snapshots of previous totals, used to compute per-second deltas.
    uint32_t last_spi_pkts = 0, last_spi_bytes = 0;              // SPI 包数 / 字节数
    uint32_t last_udp_pkts = 0, last_udp_bytes = 0;              // UDP 包数 / 字节数
    uint32_t last_udp_err = 0, last_udp_drop = 0;                // UDP 错误数 / 丢包数
    uint32_t last_eagain = 0, last_enobufs = 0, last_eother = 0; // 三类 errno 的快照

    for (;;)
    {
        // 每 1 秒统计一次
        // Run once per second.
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t sp, sb, up, ub, ue, udrop;
        uint64_t congest_until;
        uint32_t ea, enb, eoth; // 本次读取的 errno 累计值 / current errno totals

        // 在临界区中读取全局统计量，避免竞争
        // Read global stats inside critical section to avoid race conditions.
        portENTER_CRITICAL(&s_stat_mux);
        sp = g_spi_pkts_total;              // SPI 收到的总包数 / total SPI packets
        sb = g_spi_bytes_total;             // SPI 收到的总字节数 / total SPI bytes
        up = g_udp_pkts_total;              // UDP 已发送总包数 / total UDP packets sent
        ub = g_udp_bytes_total;             // UDP 已发送总字节数 / total UDP bytes sent
        ue = g_udp_err_total;               // UDP 发送错误总数 / total UDP send errors
        udrop = g_udp_drop_total;           // UDP 主动丢弃总数 / total dropped packets in coalescing
        ea = g_udp_errno_eagain_total;      // errno=EAGAIN/EWOULDBLOCK 总数
        enb = g_udp_errno_enobufs_total;    // errno=ENOBUFS 总数
        eoth = g_udp_errno_other_total;     // 其它 errno 总数
        congest_until = s_congest_until_us; // 当前拥塞退避截止时间 / current congestion backoff deadline
        portEXIT_CRITICAL(&s_stat_mux);

        // 与上一秒对比，得到“本秒新增”的统计量
        // Compute this-second increments by subtracting last snapshot.
        uint32_t dsp = sp - last_spi_pkts;    // 本秒 SPI 包数 / SPI pkts per second
        uint32_t dsb = sb - last_spi_bytes;   // 本秒 SPI 字节数 / SPI bytes per second
        uint32_t dup = up - last_udp_pkts;    // 本秒 UDP 包数 / UDP pkts per second
        uint32_t dub = ub - last_udp_bytes;   // 本秒 UDP 字节数 / UDP bytes per second
        uint32_t deu = ue - last_udp_err;     // 本秒新增 UDP 错误数 / UDP errors this second
        uint32_t ddp = udrop - last_udp_drop; // 本秒新增丢包数 / drops this second

        // 计算当前离拥塞退避结束还有多少毫秒
        // Compute remaining backoff time in ms.
        uint64_t now_us = esp_timer_get_time();
        uint32_t cong_left_ms = (now_us < congest_until)
                                    ? (uint32_t)((congest_until - now_us) / 1000)
                                    : 0;

        // 打印带宽和错误统计：SPI/UDP 吞吐、错误增量、丢包增量以及拥塞剩余时间
        // Log bandwidth and error stats: SPI/UDP throughput, error/drops, congestion time left.
        ESP_LOGI("STAT",
                 "SPI: %u pkt/s, CSI=%u B/s | UDP: %u pkt/s, %u B/s | err+=%u drop+=%u (tot=%u) cong=%ums",
                 dsp, dsb, dup, dub, deu, ddp, udrop, cong_left_ms);

        // 计算本秒三类 errno 的增量
        // Compute per-second increments for three errno categories.
        uint32_t dea = ea - last_eagain;    // 本秒新出现的 EAGAIN 次数
        uint32_t denb = enb - last_enobufs; // 本秒新出现的 ENOBUFS 次数
        uint32_t deot = eoth - last_eother; // 本秒新出现的其它 errno 次数

        ESP_LOGI("STAT", "errno+= {EAGAIN:%u ENOBUFS:%u OTHER:%u}", dea, denb, deot);

        // 背压预警逻辑：如果错误数或丢弃数超过阈值，认为链路正在背压
        // Backpressure warning: if errors or drops exceed thresholds, treat as backpressure.
        if (deu >= BACKPRESSURE_ERR_TH || ddp >= BACKPRESSURE_DROP_TH)
        {
            ESP_LOGW("BACKP",
                     "Backpressure detected: err+=%u, drop+=%u, cong_left=%u ms",
                     deu, ddp, cong_left_ms);
        }

        // —— 打印本秒 errno 的 Top-3（避免一次性打印全部 0..255）
        // —— Print Top-3 errno codes this second (avoid flooding logs for all 0..255).
        static uint32_t last_hist[256] = {0}; // 上一次的 errno 直方图快照 / last histogram snapshot
        int code[3] = {0}, inc[3] = {0};      // Top-3 errno 及其增量 / errno codes and increments
        for (int i = 0; i < 256; ++i)
        {
            uint32_t now = g_udp_errno_hist[i]; // 当前某个 errno 的累计次数
            uint32_t d = now - last_hist[i];    // 本秒该 errno 的增量
            if (d > inc[0])
            {
                inc[2] = inc[1];
                code[2] = code[1];
                inc[1] = inc[0];
                code[1] = code[0];
                inc[0] = d;
                code[0] = i;
            }
            else if (d > inc[1])
            {
                inc[2] = inc[1];
                code[2] = code[1];
                inc[1] = d;
                code[1] = i;
            }
            else if (d > inc[2])
            {
                inc[2] = d;
                code[2] = i;
            }
            // 更新快照，便于下一秒计算增量
            // Update snapshot for next-second delta computation.
            last_hist[i] = now;
        }

        // 如果本秒确实出现过某些 errno，就把 Top-3 打一行
        // If any errno occurred this second, log Top-3.
        if (inc[0] || inc[1] || inc[2])
        {
            const char *n0 = errno_name(code[0]);
            const char *n1 = errno_name(code[1]);
            const char *n2 = errno_name(code[2]);
            ESP_LOGI("STAT", "errno top: %s(%d):%u, %s(%d):%u, %s(%d):%u",
                     n0 ? n0 : "E", code[0], inc[0],
                     n1 ? n1 : "E", code[1], inc[1],
                     n2 ? n2 : "E", code[2], inc[2]);
        }

        // 更新“上一秒快照”，供下次循环使用
        // Update "last second" snapshots for next iteration.
        last_spi_pkts = sp;
        last_spi_bytes = sb;
        last_udp_pkts = up;
        last_udp_bytes = ub;
        last_udp_err = ue;
        last_udp_drop = udrop;

        last_eagain = ea;
        last_enobufs = enb;
        last_eother = eoth;
    }
}

/* ==================== app_main ==================== */
void app_main(void)
{
    // 初始化 NVS（非易失性存储），用于 Wi-Fi 配置等
    // Initialize NVS (non-volatile storage), used by Wi-Fi and other components.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // 如果 NVS 分区满了或版本不兼容，则先擦除再重新初始化
        // If NVS partition is full or version changed, erase and re-init.
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 初始化 Wi-Fi STA，并开始连接指定 AP
    // Initialize Wi-Fi STA and start connecting to the configured AP.
    wifi_sta_init();

    // 等待最多 15 秒，直到拿到 IP_GOT_BIT（即已获取 IP）
    // Wait up to 15 seconds for IP_GOT_BIT (IP acquired).
    (void)xEventGroupWaitBits(s_wifi_event_group, IP_GOT_BIT,
                              pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

    // 初始化 UDP：创建 socket，配置目标 PC 地址与端口
    // Initialize UDP: create socket and set remote PC address/port.
    udp_init();

    // 初始化 SPI 从机：准备好接收来自 A 端的 CSI 帧
    // Initialize SPI slave: ready to receive CSI frames from A-side.
    spi_slave_init();

    // 创建 SPI 接收 + UDP 转发任务，绑到指定 CPU 核心
    // Create SPI RX + UDP forward task, pinned to given CPU core.
    xTaskCreatePinnedToCore(spi_rx_and_udp_task, "spi_rx_udp",
                            8192, NULL, 7, NULL, CORE_RXUDP);

    // 每秒打印统计信息（SPI/UDP 吞吐、错误、背压等），优先级稍低
    // per-second statistics task (SPI/UDP throughput, errors, backpressure), lower priority.
    xTaskCreatePinnedToCore(stats_task, "stats",
                            4096, NULL, 4, NULL, CORE_STATS);
}
