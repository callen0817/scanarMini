#pragma once

#include <atomic>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <mutex>
#include <cmath>
#include <algorithm>

namespace robosense
{
namespace lidar
{

struct DriverDiagnostics
{
  // 1. Packet Conservation Counters (Measured at exact transition points)
  std::atomic<uint64_t> pkts_recv{0};            // Incremented immediately after recvfrom() > 0
  std::atomic<uint64_t> pkts_queued{0};          // Incremented inside packetPut() upon push
  std::atomic<uint64_t> pkts_dropped_socket{0};  // Incremented on socket error
  std::atomic<uint64_t> pkts_dequeued{0};        // Incremented inside processPacket() upon pop
  std::atomic<uint64_t> pkts_dropped_queue{0};   // Incremented on queue overflow
  std::atomic<uint64_t> pkts_decoded{0};        // Incremented inside processMsopPkt()
  std::atomic<uint64_t> pkts_decoder_rejected{0};// Incremented on bad header/CRC

  // 2. Queue Depth Statistics
  std::atomic<uint32_t> cur_queue_depth{0};
  std::atomic<uint32_t> max_queue_depth{0};
  std::atomic<uint64_t> total_queue_samples{0};
  std::atomic<uint64_t> sum_queue_depth{0};

  // 3. Frame & Revolution Conservation
  std::atomic<uint64_t> revolutions_detected{0};
  std::atomic<uint64_t> frame_builds_started{0};
  std::atomic<uint64_t> frame_builds_completed{0};
  std::atomic<uint64_t> frame_builds_aborted{0};
  std::atomic<uint64_t> frame_builds_timed_out{0};
  std::atomic<uint64_t> pointclouds_published{0}; // Incremented inside sendPointCloud()

  // 4. Frame Completion Reasons
  std::atomic<uint64_t> completed_by_angle{0};
  std::atomic<uint64_t> completed_by_timeout{0};
  std::atomic<uint64_t> completed_by_pkt_cnt{0};
  std::atomic<uint64_t> completed_by_flush{0};

  // 5. Discard Reasons
  std::atomic<uint64_t> discard_timeout{0};
  std::atomic<uint64_t> discard_overflow{0};
  std::atomic<uint64_t> discard_decoder_error{0};
  std::atomic<uint64_t> discard_missing_block{0};
  std::atomic<uint64_t> discard_short_frame{0};

  // 6. Frame Lifetime Histogram (Assembly duration from first block to final split)
  std::atomic<uint64_t> lifetime_90_110ms{0};  // 90ms - 110ms (Nominal single revolution)
  std::atomic<uint64_t> lifetime_110_150ms{0}; // 110ms - 150ms
  std::atomic<uint64_t> lifetime_150_250ms{0}; // 150ms - 250ms (2 revolutions merged)
  std::atomic<uint64_t> lifetime_250_400ms{0}; // 250ms - 400ms (3 revolutions merged)
  std::atomic<uint64_t> lifetime_gt400ms{0};   // >400ms

  // 7. Publish Interval Histogram
  std::atomic<uint64_t> hist_100ms{0}; // 80ms - 150ms (Nominal 10Hz)
  std::atomic<uint64_t> hist_200ms{0}; // 150ms - 250ms (1 scan dropped)
  std::atomic<uint64_t> hist_300ms{0}; // 250ms - 350ms (2 scans dropped)
  std::atomic<uint64_t> hist_400ms{0}; // 350ms - 450ms (3 scans dropped)
  std::atomic<uint64_t> hist_gt400ms{0}; // >450ms

  // Packets Per Frame Stats
  std::atomic<uint32_t> min_pkts_per_frame{999999};
  std::atomic<uint32_t> max_pkts_per_frame{0};
  std::atomic<uint64_t> sum_pkts_per_frame{0};

  // Latency & Timing
  double last_pub_sys_ts{0.0};
  std::mutex diag_mutex;

  static DriverDiagnostics& instance()
  {
    static DriverDiagnostics diag;
    return diag;
  }

  void update_queue_depth(uint32_t depth)
  {
    cur_queue_depth.store(depth);
    sum_queue_depth.fetch_add(depth);
    total_queue_samples.fetch_add(1);
    
    uint32_t current_max = max_queue_depth.load();
    while (depth > current_max && !max_queue_depth.compare_exchange_weak(current_max, depth))
    {
    }
  }

  void record_frame_lifetime(double lifetime_ms)
  {
    if (lifetime_ms >= 90.0 && lifetime_ms < 110.0) lifetime_90_110ms.fetch_add(1);
    else if (lifetime_ms >= 110.0 && lifetime_ms < 150.0) lifetime_110_150ms.fetch_add(1);
    else if (lifetime_ms >= 150.0 && lifetime_ms < 250.0) lifetime_150_250ms.fetch_add(1);
    else if (lifetime_ms >= 250.0 && lifetime_ms < 400.0) lifetime_250_400ms.fetch_add(1);
    else if (lifetime_ms >= 400.0) lifetime_gt400ms.fetch_add(1);
  }

  void record_pkts_per_frame(uint32_t pkts)
  {
    sum_pkts_per_frame.fetch_add(pkts);
    
    uint32_t cur_min = min_pkts_per_frame.load();
    while (pkts < cur_min && !min_pkts_per_frame.compare_exchange_weak(cur_min, pkts)) {}

    uint32_t cur_max = max_pkts_per_frame.load();
    while (pkts > cur_max && !max_pkts_per_frame.compare_exchange_weak(cur_max, pkts)) {}
  }

  void record_publish_interval(double current_sys_ts)
  {
    std::lock_guard<std::mutex> lock(diag_mutex);
    if (last_pub_sys_ts > 0.0)
    {
      double delta_ms = (current_sys_ts - last_pub_sys_ts) * 1000.0;
      if (delta_ms >= 80.0 && delta_ms < 150.0) hist_100ms.fetch_add(1);
      else if (delta_ms >= 150.0 && delta_ms < 250.0) hist_200ms.fetch_add(1);
      else if (delta_ms >= 250.0 && delta_ms < 350.0) hist_300ms.fetch_add(1);
      else if (delta_ms >= 350.0 && delta_ms < 450.0) hist_400ms.fetch_add(1);
      else if (delta_ms >= 450.0) hist_gt400ms.fetch_add(1);
    }
    last_pub_sys_ts = current_sys_ts;
    pointclouds_published.fetch_add(1);
  }

  void log_frame_published(uint64_t frame_id, double sys_ts, double hdr_ts, size_t num_pts, size_t num_pkts, double lifetime_ms)
  {
    std::cout << "[rslidar_sdk C++ FRAME PUB #" << frame_id << "]"
              << " SysTS: " << std::fixed << std::setprecision(3) << sys_ts
              << " | HdrTS: " << std::fixed << std::setprecision(3) << hdr_ts
              << " | Pts: " << num_pts
              << " | Pkts: " << num_pkts
              << " | Duration: " << std::fixed << std::setprecision(1) << lifetime_ms << " ms"
              << std::endl;
  }

  void print_summary(double elapsed_sec)
  {
    double sys_now = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    
    uint64_t samples = total_queue_samples.load();
    double avg_q_depth = (samples > 0) ? (double)sum_queue_depth.load() / samples : 0.0;

    uint64_t started = frame_builds_started.load();
    uint64_t completed = frame_builds_completed.load();
    uint64_t published = pointclouds_published.load();

    // Internal State Machine Completion Ratio (Refinement 1)
    double completion_ratio = (started > 0) ? ((double)completed / started) * 100.0 : 0.0;

    uint64_t avg_pkts = (completed > 0) ? sum_pkts_per_frame.load() / completed : 0;
    uint32_t min_pkts = (completed > 0) ? min_pkts_per_frame.load() : 0;
    uint32_t max_pkts = max_pkts_per_frame.load();

    std::cout << "\n========================================================================\n"
              << "[rslidar_sdk C++ PRODUCTION DIAGNOSTIC SUMMARY]\n"
              << "  * System Timestamp (UTC): " << std::fixed << std::setprecision(3) << sys_now << " s\n"
              << "  * Elapsed Capture Time:   " << std::fixed << std::setprecision(1) << elapsed_sec << " s\n"
              << "  * Frame Builds Started:   " << started << "\n"
              << "  * Frame Builds Completed: " << completed << "\n"
              << "  * Actual Published Scans: " << published << "\n"
              << "  * COMPLETION RATIO (State Machine): " << std::fixed << std::setprecision(1) << completion_ratio << " %\n"
              << "------------------------------------------------------------------------\n"
              << "1. PACKET CONSERVATION EQUATIONS:\n"
              << "   - UDP Recv (recvfrom > 0): " << pkts_recv.load() << "\n"
              << "   - Queued into Buffer:     " << pkts_queued.load() << " (Socket Drops: " << pkts_dropped_socket.load() << ")\n"
              << "   - Dequeued for Decoder:   " << pkts_dequeued.load() << " (Queue Drops: " << pkts_dropped_queue.load() << ")\n"
              << "   - Decoded MSOP/DIFOP:     " << pkts_decoded.load() << " (Decoder Rejections: " << pkts_decoder_rejected.load() << ")\n"
              << "------------------------------------------------------------------------\n"
              << "2. QUEUE DEPTH STATISTICS:\n"
              << "   - Current Depth: " << cur_queue_depth.load() 
              << " | Max Depth: " << max_queue_depth.load() 
              << " | Avg Depth: " << std::fixed << std::setprecision(1) << avg_q_depth << "\n"
              << "------------------------------------------------------------------------\n"
              << "3. FRAME CONSERVATION EQUATION:\n"
              << "   - Frame Builds Started:   " << started << "\n"
              << "   - Frame Builds Completed: " << completed << " (Angle: " << completed_by_angle.load() 
              << " | Timeout: " << completed_by_timeout.load() << " | PktCount: " << completed_by_pkt_cnt.load() << ")\n"
              << "   - Frame Builds Aborted:   " << frame_builds_aborted.load() << "\n"
              << "   - Frame Builds Timed Out: " << frame_builds_timed_out.load() << "\n"
              << "   - Conservation Check:     " << started << " == " << (completed + frame_builds_aborted.load() + frame_builds_timed_out.load()) << "\n"
              << "------------------------------------------------------------------------\n"
              << "4. PACKETS PER COMPLETED FRAME:\n"
              << "   - Min: " << min_pkts << " | Avg: " << avg_pkts << " | Max: " << max_pkts << "\n"
              << "------------------------------------------------------------------------\n"
              << "5. FRAME LIFETIME HISTOGRAM (Assembly Duration):\n"
              << "   - 90ms - 110ms (Nominal 1 Rev):  " << lifetime_90_110ms.load() << "\n"
              << "   - 110ms - 150ms:                 " << lifetime_110_150ms.load() << "\n"
              << "   - 150ms - 250ms (2 Revs Merged): " << lifetime_150_250ms.load() << "\n"
              << "   - 250ms - 400ms (3 Revs Merged): " << lifetime_250_400ms.load() << "\n"
              << "   - >400ms (Extended Blackout):    " << lifetime_gt400ms.load() << "\n"
              << "------------------------------------------------------------------------\n"
              << "6. PUBLISH INTERVAL HISTOGRAM:\n"
              << "   - 100ms (10 Hz Nominal): " << hist_100ms.load() << "\n"
              << "   - 200ms (1 Scan Missed): " << hist_200ms.load() << "\n"
              << "   - 300ms (2 Scans Missed):" << hist_300ms.load() << "\n"
              << "   - 400ms (3 Scans Missed):" << hist_400ms.load() << "\n"
              << "   - >450ms (Blackout):     " << hist_gt400ms.load() << "\n"
              << "========================================================================\n" << std::endl;
  }
};

} // namespace lidar
} // namespace robosense
