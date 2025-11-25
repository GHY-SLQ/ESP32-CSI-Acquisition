#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket, os, csv, struct, time, queue, threading, argparse, sys
from datetime import datetime

# ====== 与固件 csi_bin_v2_t 完全一致（小端） ======
# ====== Exactly the same layout as firmware struct csi_bin_v2_t (little-endian) ======
HDR_FMT  = "<I H H I 6s b B B B B B B B B B B b B B B I B B B H H H B"
# struct.unpack 使用的格式串，描述 UDP 头部的二进制布局（小端）
# Format string for struct.unpack, describing the binary header layout (little-endian).

HDR_SIZE  = struct.calcsize(HDR_FMT)   # 47
# 头部总字节数，应当等于固件端的 sizeof(csi_v2_hdr_t) = 47
# Total header size in bytes; must match sizeof(csi_v2_hdr_t) = 47 on firmware.

MAGIC_CSI = 0x32495343                 # 'CSI2' 小端
# 帧识别用的 magic 值（ASCII "CSI2"，little-endian 存储为 0x32495343）
# Magic value to identify CSI2 frames (ASCII "CSI2" stored as 0x32495343 in little-endian).

DEFAULT_CSI_MAX = 612
# 默认允许的最大 csi_len，防止异常长度导致解析越界或内存爆炸
# Default maximum accepted csi_len to avoid overflow or huge allocations.

CSV_COLUMNS = [
    "pc_perf_us",        # CPU 单调时钟的微秒级时间戳 / CPU monotonic clock timestamp in microseconds
    "magic","hdr_len","ver",
    "rx_id","mac",
    "rssi","rate","sig_mode","mcs","cwb",
    "smoothing","not_sounding","aggregation","stbc",
    "fec_coding","sgi",
    "noise_floor","ampdu_cnt","channel","secondary_channel",
    "timestamp","ant",
    "agc_gain","fft_gain",     # added columns for AGC/FFT gain
    "sig_len","rx_state",
    "csi_len","first_word_invalid",
    "csi"                      # CSI 数据数组，序列化为字符串 / CSI data array serialized as string
]

def mac_to_str(b6: bytes) -> str:
    """将 6 字节 MAC 地址转换为常见形式的字符串（xx:xx:xx:xx:xx:xx）
    Convert a 6-byte MAC address into the common string form (xx:xx:xx:xx:xx:xx).
    """
    return ":".join(f"{x:02x}" for x in b6)


def parse_frames_into_rows(dat_mv: memoryview, csi_max: int, keep_csi: bool, rows_out: list):
    """
    从一块 UDP 原始数据中按帧解析 CSI2 记录，并转换为 CSV 行。
    Parse multiple CSI2 frames from a UDP data block and append them as CSV rows.
    """
    off = 0
    n = len(dat_mv)
    # 循环扫描 dat_mv，只要还剩下至少一个完整头部的长度就继续
    # Loop as long as there is space for at least one full header.
    while off + HDR_SIZE <= n:
        try:
            # 按固定格式解出一个头部
            # Unpack one header using the fixed format.
            head = struct.unpack(HDR_FMT, dat_mv[off:off+HDR_SIZE])
        except struct.error:
            # 数据剩余不足一个头，或者解析失败，直接结束本批解析
            # Not enough data or corrupt header: stop parsing this buffer.
            break

        (magic, hdr_len, ver,
         rx_id, mac6,
         rssi, rate, sig_mode, mcs, cwb,
         smoothing, not_sounding, aggregation, stbc,
         fec_coding, sgi,
         noise_floor, ampdu_cnt, channel, secondary_channel,
         timestamp, ant, 
         agc_gain, fft_gain,                 # ★ AGC / FFT 增益
         sig_len, rx_state,
         csi_len, first_word_invalid) = head

        # 检查 magic 和 hdr_len 是否匹配预期，不对就认为是“偏移错位”，off+1 继续扫
        # Validate magic and header length; if mismatch, slide by 1 byte and retry.
        if magic != MAGIC_CSI or hdr_len != HDR_SIZE:
            off += 1
            continue

        # 检查 csi_len 合法性，防止异常长度
        # Validate csi_len to avoid bogus or huge values.
        if csi_len < 0 or csi_len > csi_max:
            off += 1
            continue

        # 这一帧总共需要的字节数 = 头部 + CSI 数据
        # Total bytes needed for this frame = header + CSI payload.
        need = hdr_len + csi_len
        if off + need > n:
            # 剩余数据不够一整帧，认为被截断，等下一个 UDP 包再解析
            # Not enough bytes for a full frame; treat as truncated and stop.
            break  # 被截断 / truncated

        # 使用 CPU 单调时钟作为本地时间戳，单位：微秒
        # Use CPU monotonic clock as local timestamp in microseconds.
        pc_us = int(time.perf_counter_ns() // 1_000)

        if keep_csi:
            # 取出此帧的 CSI byte 序列
            # Slice out CSI byte sequence for this frame.
            csi_mv = dat_mv[off+HDR_SIZE: off+HDR_SIZE+csi_len]
            try:
                # 解析为有符号 int8 数组
                # Interpret as signed int8 array.
                csi_vals = struct.unpack_from(f"<{csi_len}b", csi_mv, 0)
            except struct.error:
                # CSI 段损坏，跳过这个偏移，从下一个字节继续
                # Corrupted CSI segment; slide by 1 byte and retry.
                off += 1
                continue
            # 序列化为字符串形式写入 CSV
            # Serialize CSI array as a string for CSV.
            csi_str = "[" + ",".join(str(v) for v in csi_vals) + "]"
        else:
            # 不需要保存 CSI 数组时，只写空数组占位，速度更快
            # When CSI array is not needed, store an empty placeholder (faster).
            csi_str = "[]"

        # 组装一行 CSV 数据，字段顺序与 CSV_COLUMNS 完全一致
        # Build one CSV row; field order matches CSV_COLUMNS exactly.
        rows_out.append([
            pc_us,                       # 写入微秒时间戳 / timestamp (us) on CPU of PC
            magic, hdr_len, ver,
            rx_id, mac_to_str(mac6),
            rssi, rate, sig_mode, mcs, cwb,
            smoothing, not_sounding, aggregation, stbc,
            fec_coding, sgi,
            noise_floor, ampdu_cnt, channel, secondary_channel,
            timestamp, ant, 
            agc_gain, fft_gain,
            sig_len, rx_state,
            csi_len, first_word_invalid,
            csi_str
        ])

        # 移动偏移到下一帧开头（紧跟在当前 payload 后面）
        # Advance offset to the next frame (right after current payload).
        off += need


def writer_thread(csv_path: str,
                  q_rows: "queue.Queue[list]",
                  flush_sec: float,
                  stop_evt: threading.Event,
                  stats: dict,
                  stat_lock: threading.Lock):
    """
    从 q_rows 中取出行并写入 CSV 文件的后台线程。
    Background thread that consumes rows from q_rows and writes them to a CSV file.
    """
    # 确保输出目录存在
    # Ensure the output directory exists.
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    # 以写模式打开 CSV 文件，并写入表头
    # Open CSV file for writing and emit the header row.
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLUMNS)
        last_flush = time.time()

        while not stop_evt.is_set():
            try:
                # 从队列中取一行，如果 0.2 秒内没数据就抛 queue.Empty
                # Get one row from the queue; timeout after 0.2 s if empty.
                item = q_rows.get(timeout=0.2)
            except queue.Empty:
                # 即使暂时没数据，也定期 flush 一下，避免缓存太久
                # Even if no data, periodically flush to avoid long buffering.
                if time.time() - last_flush >= flush_sec:
                    try:
                        f.flush()
                    except:
                        pass
                    last_flush = time.time()
                continue

            # 使用 None 作为“结束标记”，退出写线程
            # Use None as a sentinel to terminate the writer thread.
            if item is None:
                break

            # 正常写入一行 CSV
            # Write one CSV row.
            w.writerow(item)

            # 更新已写入行数统计
            # Update number of written rows in stats.
            with stat_lock:
                stats["rows_written"] += 1

            # 到达 flush 周期就手动 flush 一次
            # Flush to disk when flush interval has elapsed.
            if time.time() - last_flush >= flush_sec:
                try:
                    f.flush()
                except:
                    pass
                last_flush = time.time()

        # 线程退出前再最后 flush 一次，保证数据落盘
        # Final flush before thread exits to ensure data is on disk.
        try:
            f.flush()
        except:
            pass

def run(bind_ip: str, bind_port: int, out_dir: str, csi_max: int,
        rcvbuf_bytes: int, flush_sec: float, print_sec: float, keep_csi: bool):
    """
    主控制函数：负责收 UDP、解析 CSI 帧、将数据写入 CSV，并周期性打印统计信息。
    Main control function: receive UDP, parse CSI frames, write them to CSV,
    and periodically print runtime statistics.
    """

    # 生成带时间戳的输出文件名，例如 csi_data_20251124_010203.csv
    # Generate timestamped CSV file path, e.g. csi_data_20251124_010203.csv
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(out_dir, f"csi_data_{ts}.csv")

    # 行队列：从解析线程到写文件线程（每个元素是一行 CSV）
    # Row queue: from parser thread to writer thread (each item is one CSV row).
    q_rows: "queue.Queue[list]" = queue.Queue(maxsize=200000)
    # UDP 队列：从收包线程到解析线程（每个元素是一个完整 UDP 包的 bytes）
    # UDP queue: from receiver thread to parser thread (each item is a full UDP packet).
    q_udp: "queue.Queue[bytes]" = queue.Queue(maxsize=20000)
    # 停止事件：用于优雅退出所有线程
    # Stop event to signal all threads to exit gracefully.
    stop_evt = threading.Event()

    # 统计字典：用于记录各种计数（收包数、帧数、写入行数等）
    # Stats dictionary: records various counters (UDP packets, frames, rows, etc.).
    stats = {
        "udp_pkts": 0, "udp_bytes": 0, "udp_drops": 0,
        "frames": 0, "csi_bytes": 0,
        "rows_enq": 0, "rows_drop": 0,
        "rows_written": 0,
        "parse_err": 0,
    }
    # 互斥锁：保护 stats 的并发访问
    # Mutex to protect concurrent access to stats.
    stat_lock = threading.Lock()

    # 简化的统计累加函数
    # Helper function to safely increment a statistic.
    def add_stat(k, v=1):
        with stat_lock:
            stats[k] += v

    # 启动 CSV 写线程：持续从 q_rows 中取出行写到 csv_path
    # Start CSV writer thread: continuously consumes rows from q_rows into csv_path.
    wt = threading.Thread(target=writer_thread,
                          args=(csv_path, q_rows, flush_sec, stop_evt, stats, stat_lock),
                          daemon=True)
    wt.start()

    # 解析线程：从 q_udp 拿 UDP 包，解析成多行 CSV，放入 q_rows
    # Parser thread: takes UDP packets from q_udp, parses them into rows, pushes to q_rows.
    def parse_loop():
        rows_buf = []  # 临时缓存本次从一个 UDP 包解析出的所有行
        # Temporary buffer for rows parsed from a single UDP datagram.
        while not stop_evt.is_set():
            try:
                # 取一个 UDP 包，0.2 秒无数据则重试
                # Get one UDP packet; timeout after 0.2 s if empty.
                dat = q_udp.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                rows_buf.clear()
                mv = memoryview(dat)
                before = len(rows_buf)
                # 从这个 UDP 包中解析出若干帧，并追加到 rows_buf
                # Parse multiple frames from this UDP packet into rows_buf.
                parse_frames_into_rows(mv, csi_max, keep_csi, rows_buf)
                parsed = len(rows_buf) - before
                if parsed > 0:
                    # 成功解析出若干帧
                    # Successfully parsed some frames.
                    add_stat("frames", parsed)
                else:
                    # 没解析出任何帧，视作解析错误（可能是残包或无效数据）
                    # No frames parsed, treat as parse error.
                    add_stat("parse_err", 1)
            except Exception:
                # 任意异常都计入解析错误，避免线程直接崩溃
                # Any unexpected error increments parse_err to keep the thread alive.
                add_stat("parse_err", 1)
                continue

            # 将解析得到的每一行丢进写线程的队列
            # Enqueue every parsed row to the writer thread's queue.
            for r in rows_buf:
                try:
                    q_rows.put_nowait(r)
                    add_stat("rows_enq", 1)
                except queue.Full:
                    # 若行队列满，则丢弃该行并计数
                    # If row queue is full, drop this row and count it.
                    add_stat("rows_drop", 1)

    # 启动解析线程
    # Start the parser thread.
    pt = threading.Thread(target=parse_loop, daemon=True)
    pt.start()

    # 创建 UDP socket 并设置基本选项
    # Create UDP socket and set basic options.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        # 尝试设置较大的接收缓冲区，减少 UDP 丢包
        # Try to set a large receive buffer to reduce UDP drops.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf_bytes)
    except OSError:
        # 某些系统可能不允许修改，失败就忽略
        # Some systems may reject this; ignore on failure.
        pass
    # 绑定到指定 IP 和端口
    # Bind to given IP and port.
    sock.bind((bind_ip, bind_port))
    # 设置 recvfrom 超时，方便定期检查 stop_evt
    # Set recv timeout so we can periodically check stop_evt.
    sock.settimeout(0.5)

    # 查询真实的接收缓冲区大小（操作系统可能会调整）
    # Query the actual receive buffer size (OS may adjust the requested value).
    try:
        real_rcvbuf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    except OSError:
        real_rcvbuf = -1

    print(f"[INFO] Listening on UDP {bind_ip}:{bind_port}, SO_RCVBUF(real)={real_rcvbuf}")
    print(f"[INFO] Writing CSV to {csv_path}")
    print(f"[INFO] keep_csi={'ON' if keep_csi else 'OFF'}")

    # 收包线程：只负责把 raw UDP 数据丢进 q_udp
    # Receiver thread: only receives raw UDP data and pushes it into q_udp.
    def recv_loop():
        buf = bytearray(65535)  # 单帧最大 UDP 包缓存 / buffer for a single UDP packet
        while not stop_evt.is_set():
            try:
                # 直接读入预分配的 buf，减少临时对象分配
                # Read directly into preallocated buffer to reduce allocations.
                n, _ = sock.recvfrom_into(buf)
            except socket.timeout:
                continue
            except OSError:
                # socket 被关闭或出错时退出循环
                # Exit loop when socket is closed or errored.
                break
            if n <= 0:
                continue
            # 将有效字节切片成 bytes 对象，避免整块复制
            # Slice valid bytes as a separate bytes object.
            pkt = bytes(memoryview(buf)[:n])
            try:
                # 非阻塞入队，队满就统计为 UDP 丢包
                # Non-blocking enqueue; if full, count as UDP drop.
                q_udp.put_nowait(pkt)
                add_stat("udp_pkts", 1)
                add_stat("udp_bytes", n)
            except queue.Full:
                add_stat("udp_drops", 1)

    # 启动收包线程
    # Start the UDP receiver thread.
    rt = threading.Thread(target=recv_loop, daemon=True)
    rt.start()

    # 统计打印相关变量
    # Variables for periodic statistics printing.
    last = time.time()
    last_stats = {k: 0 for k in stats}
    last_total5 = time.time()  # 5 秒总量打印计时 / timer for 5-second total print

    try:
        # 主线程循环：定期打印统计信息
        # Main loop: periodically print runtime statistics.
        while True:
            time.sleep(print_sec)
            now = time.time()
            elapsed = max(1e-6, now - last)  # 防止除零 / prevent division by zero
            with stat_lock:
                cur = stats.copy()
            # 计算本周期内各统计量的增量
            # Compute increments of each statistic for this period.
            inc = {k: cur[k]-last_stats[k] for k in cur}
            last_stats = cur
            last = now

            # 各类速率（每秒）
            # Per-second rates.
            udp_pps  = inc["udp_pkts"]/elapsed     # UDP 包速率 / UDP packets per second
            udp_bps  = inc["udp_bytes"]/elapsed    # UDP 字节速率 / UDP bytes per second
            frm_ps   = inc["frames"]/elapsed       # 解析出的帧速率 / parsed frames per second
            rows_ps  = inc["rows_enq"]/elapsed     # 入队 CSV 行速率 / enqueued rows per second

            print(f"[STAT] UDP: {udp_pps:.0f} pkt/s, {udp_bps:.0f} B/s, "
                  f"drop={inc['udp_drops']} | "
                  f"Frames: {frm_ps:.0f} /s | "
                  f"Rows->CSV: {rows_ps:.0f} /s, drop={inc['rows_drop']} | "
                  f"parse_err+={inc['parse_err']} | "
                  f"q_udp={q_udp.qsize()} q_rows={q_rows.qsize()}",
                  flush=True)

            # 每 5 秒打印一次总写入行数
            # Every 5 seconds print total written rows.
            if now - last_total5 >= 5.0:
                print(f"[TOTAL] written_rows={cur['rows_written']}", flush=True)
                last_total5 = now

    except KeyboardInterrupt:
        # Ctrl+C 时优雅退出
        # Gracefully shut down on Ctrl+C.
        print("\n[INFO] stopping...")
    finally:
        # 通知各线程停止
        # Signal all threads to stop.
        stop_evt.set()
        # 给 writer 线程发一个 None 作为结束标志
        # Send a None sentinel to the writer thread to unblock and exit.
        try:
            q_rows.put_nowait(None)
        except:
            pass
        # 关闭 socket，结束 recv_loop
        # Close socket to terminate recv_loop.
        try:
            sock.close()
        except:
            pass
        # 等待各线程结束（带超时，防止卡死）
        # Join threads with timeouts to avoid hanging indefinitely.
        rt.join(timeout=1.0)
        pt.join(timeout=1.0)
        wt.join(timeout=2.0)
        print(f"[INFO] CSV saved to {csv_path}")

def main():
    """
    命令行入口：解析参数后调用 run()。
    Command-line entry point: parse arguments and call run().
    """
    ap = argparse.ArgumentParser(
        description="ESP32 CSI v2 UDP Receiver -> CSV (high-throughput)"
    )
    # 监听 IP，默认 0.0.0.0（监听所有本地网卡）
    # Bind IP, default 0.0.0.0 (listen on all local interfaces).
    ap.add_argument("--ip", default="0.0.0.0",
                    help="Bind IP (default: 0.0.0.0)")
    # 监听 UDP 端口，默认 5000
    # UDP listen port, default 5000.
    ap.add_argument("--port", type=int, default=5000,
                    help="Bind UDP port (default: 5000)")
    # 输出目录，默认 received_CSI
    # Output directory for CSV files, default: received_CSI.
    ap.add_argument("--out", default="received_CSI",
                    help="Output directory (default: received_CSI)")
    # 允许的最大 csi_len，用来过滤异常数据
    # Maximum csi_len to accept, used to filter invalid data.
    ap.add_argument("--csi-max", type=int, default=DEFAULT_CSI_MAX,
                    help="Max csi_len to accept")
    # socket 接收缓冲区大小（字节），默认 8 MB
    # Socket receive buffer size in bytes, default: 8 MB.
    ap.add_argument("--rcvbuf", type=int, default=8*1024*1024,
                    help="SO_RCVBUF bytes (default: 8MB)")
    # CSV flush 周期（秒）
    # Interval in seconds to flush CSV file.
    ap.add_argument("--flush", type=float, default=1.0,
                    help="Flush CSV every N seconds")
    # 统计打印周期（秒）
    # Interval in seconds to print runtime statistics.
    ap.add_argument("--print", type=float, default=1.0,
                    help="Print stats every N seconds")
    # 不写入 CSI 数组本体，只写元信息（大幅减小 CSV 体积、提高速度）
    # Do not write CSI array itself, only metadata (much faster, smaller CSV).
    ap.add_argument("--no-csi", action="store_true",
                    help="Do not write CSI array to CSV (much faster)")

    args = ap.parse_args()

    # 根据命令行参数启动主逻辑
    # Start main logic with parsed arguments.
    run(args.ip, args.port, args.out, args.csi_max,
        args.rcvbuf, args.flush, args.print,
        keep_csi=not args.no_csi)


if __name__ == "__main__":
    main()

