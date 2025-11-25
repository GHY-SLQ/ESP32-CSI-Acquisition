#pragma once
#include <stdint.h>
#include <stddef.h>

/* CSI 帧头大小 = csi_frame_t 结构中 payload 字段相对于结构起始的偏移
 * CSI header size = offset of 'payload' field inside csi_frame_t.
 */
#define CSI_HDR_SIZE ((size_t)offsetof(csi_frame_t, payload))

/* 固定帧标识，用于识别 CSI 帧（ASCII: 'C' 'S' 'I' '2'）
 * Fixed frame magic, used to identify CSI frames (ASCII 'CSI2').
 */
#define CSI_MAGIC 0x43534932u /* 'CSI2' */

/* 单帧/单分片的最大净荷字节数
 * Maximum payload size (bytes) for a single frame/fragment.
 */
#define CSI_MAX_PAYLOAD 4096  /* 单分片上限 / per-fragment upper limit */

#pragma pack(push, 1)
/* CSI 传输帧结构：分片头 + 元信息 + 业务字段 + CSI 数据
 * CSI transfer frame: header + meta info + app fields + CSI payload.
 */
typedef struct 
{
    /* 分片头（与以前保持前缀兼容）
     * Fragment header (kept compatible with legacy prefix layout).
     */
    uint32_t magic;       /* 固定 0x43534932 / constant 0x43534932 ('CSI2') */
    uint16_t seq;         /* 发送序号 / sequence number */
    uint16_t total_len;   /* 原始 CSI 总长 / total CSI length (bytes) */
    uint16_t offset;      /* 本分片在总数据中的偏移 / offset of this fragment in whole CSI */
    uint16_t payload_len; /* 本分片净荷长度 / payload length in this fragment (= csi_len) */

    /* 元信息（从 wifi_csi_info_t->rx_ctrl 搬过来）
     * Meta info copied from wifi_csi_info_t->rx_ctrl.
     */
    uint8_t mac[6];       /* 发送端 MAC 地址 / sender MAC address */
    int8_t  rssi;         /* 接收 RSSI / received RSSI */
    uint8_t rate;         /* 速率 / data rate */
    uint8_t sig_mode;     /* 信号模式 / signal mode */
    uint8_t mcs;          /* MCS 编号 / MCS index */
    uint8_t cwb;          /* 信道带宽指示 / channel width flag */

    uint8_t smoothing;    /* 平滑标志 / smoothing flag */
    uint8_t not_sounding; /* sounding 标志 / not-sounding flag */
    uint8_t aggregation;  /* 聚合标志 / aggregation flag */
    uint8_t stbc;         /* STBC 标志 / STBC flag */
    uint8_t fec_coding;   /* FEC 编码类型 / FEC coding type */
    uint8_t sgi;          /* 短 GI 标志 / short GI flag */
    int8_t  noise_floor;  /* 噪声底 / noise floor (dBm) */
    uint8_t ampdu_cnt;    /* AMPDU 计数 / AMPDU counter */
    uint8_t channel;      /* 主信道号 / primary channel */
    uint8_t secondary_channel; /* 副信道信息 / secondary channel info */

    uint32_t timestamp;   /* 接收时间戳 / RX timestamp */
    uint8_t  ant;         /* 天线号 / antenna index */

    uint8_t  agc_gain;    /* AGC 增益 / AGC gain */
    uint8_t  fft_gain;    /* FFT 增益 / FFT scaling gain */

    uint16_t sig_len;     /* 有效数据长度 / signal length (bytes) */
    uint16_t rx_state;    /* 接收状态 / RX state flags */
    uint16_t csi_len;     /* CSI 数据长度（与 payload_len 相同） / CSI data length (= payload_len) */
    uint8_t  first_word_invalid; /* 首字无效标志 / first-word-invalid flag */

    /* 业务字段
     * Application-level field.
     */
    uint32_t rx_id;       /* 业务序号（如发送端标记） / application RX ID or sequence */

    /* 分片数据：实际 CSI 复数序列按字节打包存放
     * Fragment payload: raw CSI complex samples packed in bytes.
     */
    uint8_t payload[CSI_MAX_PAYLOAD];
} csi_frame_t;
#pragma pack(pop)
