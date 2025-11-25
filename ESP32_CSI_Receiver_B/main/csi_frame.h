#pragma once
#include <stdint.h>

/* CSI 帧的魔数，用于识别 SPI/UDP 中的 CSI 数据帧
 * Magic number for CSI frames, used to identify CSI data over SPI/UDP.
 * ASCII: 'C' 'S' 'I' '2'
 */
#define CSI_MAGIC 0x43534932u /* 'CSI2' */

/* 单个分片允许携带的最大 CSI 净荷字节数
 * Maximum CSI payload size (bytes) per fragment/frame.
 */
#define CSI_MAX_PAYLOAD 4096  /* 单分片净荷上限 / per-fragment payload upper limit */

/* SPI A->B 分片帧：包含完整元信息（1 字节对齐以跨端一致）
 * SPI A->B fragment frame: includes full meta info, 1-byte packed for cross-platform consistency.
 */
#pragma pack(push, 1)
typedef struct
{
    /* 分片公共头 / Common fragment header */
    uint32_t magic;       /* 固定 0x43534932，用于帧同步和校验 / fixed 0x43534932 for frame sync/check */
    uint16_t seq;         /* 发送序号（A 端递增）/ TX sequence number (incremented on A side) */
    uint16_t total_len;   /* 原始 CSI 总长（未分片前的总字节数）/ total original CSI length in bytes */
    uint16_t offset;      /* 本分片在整条 CSI 中的偏移 / offset of this fragment in the full CSI stream */
    uint16_t payload_len; /* 本分片净荷长度 (= csi_len) / payload length of this fragment (= csi_len) */

    /* —— 从 wifi_csi_info_t->rx_ctrl 搬来的全部元信息 —— */
    /* —— Full meta info copied from wifi_csi_info_t->rx_ctrl —— */
    uint8_t mac[6];       /* 发送端 MAC 地址 / sender MAC address */
    int8_t  rssi;         /* 接收 RSSI / received RSSI */
    uint8_t rate;         /* 速率 / data rate */
    uint8_t sig_mode;     /* 信号模式（11b/g/n 等）/ signal mode (11b/g/n etc.) */
    uint8_t mcs;          /* MCS 编号 / MCS index */
    uint8_t cwb;          /* 信道带宽指示 / channel width flag */

    uint8_t smoothing;    /* 平滑标志 / smoothing flag */
    uint8_t not_sounding; /* sounding 标志 / not-sounding flag */
    uint8_t aggregation;  /* 聚合标志 / aggregation flag */
    uint8_t stbc;         /* STBC 标志 / STBC flag */
    uint8_t fec_coding;   /* FEC 编码类型 / FEC coding type */
    uint8_t sgi;          /* 短 GI 标志 / short-GI flag */
    int8_t  noise_floor;  /* 噪声底（dBm）/ noise floor (dBm) */
    uint8_t ampdu_cnt;    /* AMPDU 计数 / AMPDU count */
    uint8_t channel;      /* 主信道号 / primary channel index */
    uint8_t secondary_channel; /* 副信道信息 / secondary channel info */

    uint32_t timestamp;   /* 接收时间戳 / RX timestamp from hardware */
    uint8_t  ant;         /* 接收天线编号 / RX antenna index */
    
    uint8_t  agc_gain;    /* AGC 增益 / AGC gain */
    uint8_t  fft_gain;    /* FFT 缩放增益 / FFT scaling gain */
    
    uint16_t sig_len;     /* 有效信号长度 / signal length (bytes) */
    uint16_t rx_state;    /* 接收状态标志位 / RX state flags */
    uint16_t csi_len;     /* CSI 数据长度（与 payload_len 相同）/ CSI data length (= payload_len) */
    uint8_t  first_word_invalid; /* 首字无效标志 / first-word-invalid flag */

    /* 业务字段 / Application-level field */
    uint32_t rx_id;       /* 业务侧的接收序号（发送端写入）/ app-level RX ID set by sender */

    /* 分片数据 / Fragment payload */
    uint8_t payload[CSI_MAX_PAYLOAD]; /* 实际 CSI 字节序列 / raw CSI bytes */
} csi_frame_t;
#pragma pack(pop)

/* ===== UDP v2 表头（发给 PC）/ UDP v2 header (sent to PC) ===== */
#pragma pack(push, 1)
typedef struct
{
    uint32_t magic;   // 魔数：0x32495343 'CSI2' (LE)，用于识别 v2 包
                      // Magic: 0x32495343 'CSI2' (LE), identifies v2 packets.
    uint16_t hdr_len; // 表头长度，固定 47 字节
                      // Header length, fixed to 47 bytes.
    uint16_t ver;     // 协议版本，建议填 2
                      // Protocol version, recommended 2.
    uint32_t rx_id;   // 业务侧 ID（与 SPI 帧中的 rx_id 对应）
                      // Application ID, matches rx_id in SPI frame.

    uint8_t mac[6];   // 发送端 MAC / sender MAC
    int8_t  rssi;     // RSSI / received RSSI
    uint8_t rate, sig_mode, mcs, cwb; // 调制 & 带宽信息 / rate & PHY mode & MCS & channel width
    uint8_t smoothing, not_sounding, aggregation, stbc;
    uint8_t fec_coding, sgi;
    int8_t  noise_floor;
    uint8_t ampdu_cnt, channel, secondary_channel;
    uint32_t timestamp;
    uint8_t ant;
 
    uint8_t  agc_gain;   // AGC 增益 / AGC gain
    uint8_t  fft_gain;   // FFT 增益 / FFT scaling gain

    uint16_t sig_len, rx_state;
    uint16_t csi_len;    // 后面紧跟 csi_len 字节的 CSI 数据
                         // Number of CSI bytes following this header.
    uint8_t  first_word_invalid;
} csi_v2_hdr_t;
#pragma pack(pop)

/* UDP v2 协议常量 / UDP v2 protocol constants */
#define CSI2_MAGIC   0x32495343u  // 'CSI2' 魔数，与 csi_v2_hdr_t.magic 一致
                                  // Magic value matching csi_v2_hdr_t.magic.
#define CSI2_HDR_LEN 47u          // csi_v2_hdr_t 的固定长度（字节数）
                                  // Fixed size of csi_v2_hdr_t in bytes.

/* 编译期断言：确保 csi_v2_hdr_t 实际大小就是 47 字节，否则编译报错
 * Compile-time assert: ensure csi_v2_hdr_t is exactly 47 bytes, or fail build.
 */
_Static_assert(sizeof(csi_v2_hdr_t) == CSI2_HDR_LEN, "csi_v2_hdr_t must be 47 bytes");
