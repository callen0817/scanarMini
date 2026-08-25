#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

struct StreamStats {
  std::string name;
  std::vector<double> header_stamps;
  std::vector<double> host_recv_times;
  std::vector<double> latencies_ms;
  std::mutex mtx;

  void add(double header_sec, double host_sec) {
    std::lock_guard<std::mutex> lock(mtx);
    header_stamps.push_back(header_sec);
    host_recv_times.push_back(host_sec);
    latencies_ms.push_back((host_sec - header_sec) * 1000.0);
  }

  void printReport(double duration_sec) {
    std::lock_guard<std::mutex> lock(mtx);
    if (header_stamps.size() < 5) {
      std::cout << "\n📊 " << name << ": INSUFFICIENT DATA (" << header_stamps.size() << " samples)\n";
      return;
    }

    double rate = static_cast<double>(header_stamps.size()) / duration_sec;
    std::vector<double> sorted_lat = latencies_ms;
    std::sort(sorted_lat.begin(), sorted_lat.end());
    double median_lat = sorted_lat[sorted_lat.size() / 2];
    double mean_lat = std::accumulate(sorted_lat.begin(), sorted_lat.end(), 0.0) / sorted_lat.size();
    double min_lat = sorted_lat.front();
    double max_lat = sorted_lat.back();

    // Intervals & Jitter
    std::vector<double> intervals_ms;
    for (size_t i = 1; i < header_stamps.size(); ++i) {
      intervals_ms.push_back((header_stamps[i] - header_stamps[i-1]) * 1000.0);
    }
    std::sort(intervals_ms.begin(), intervals_ms.end());
    double p50_dt = intervals_ms[intervals_ms.size() * 0.50];
    double p95_dt = intervals_ms[intervals_ms.size() * 0.95];
    double p99_dt = intervals_ms[intervals_ms.size() * 0.99];
    double max_dt = intervals_ms.back();
    double min_dt = intervals_ms.front();

    // Clock drift slope via linear regression
    double t_span = host_recv_times.back() - host_recv_times.front();
    double drift_ppm = 0.0;
    if (t_span > 1.0) {
      double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
      size_t n = host_recv_times.size();
      double t0 = host_recv_times.front();
      for (size_t i = 0; i < n; ++i) {
        double x = host_recv_times[i] - t0;
        double y = latencies_ms[i];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
      }
      double denom = (n * sum_xx - sum_x * sum_x);
      if (std::abs(denom) > 1e-9) {
        double slope = (n * sum_xy - sum_x * sum_y) / denom; // ms/s
        drift_ppm = slope * 1000.0; // ppm (us/s)
      }
    }

    std::cout << "\n📊 " << name << ":\n";
    std::cout << "   • Delivered Samples:      " << header_stamps.size() << " (" << std::fixed << std::setprecision(2) << rate << " Hz)\n";
    std::cout << "   • Ingestion Latency:      Mean: " << std::setprecision(2) << mean_lat << " ms | Median: " << median_lat << " ms (Min: " << min_lat << " ms, Max: " << max_lat << " ms)\n";
    std::cout << "   • Cadence / Jitter:       P50 dt: " << p50_dt << " ms | P95 dt: " << p95_dt << " ms | P99 dt: " << p99_dt << " ms | Range: [" << min_dt << " .. " << max_dt << "] ms\n";
    std::cout << "   • Clock Drift vs Host:    " << std::setprecision(2) << drift_ppm << " ppm (us/s) | Net Window Drift: " << (latencies_ms.back() - latencies_ms.front()) << " ms\n";
  }
};

class M9BenchmarkNode : public rclcpp::Node {
public:
  StreamStats lidar_stats{"Airy LiDAR (/rslidar_points)"};
  StreamStats imu_stats{"Airy IMU (/rslidar_imu_data)"};
  StreamStats left_ir_stats{"Gemini Left IR (/camera/left_ir/image_raw)"};
  StreamStats rgb_stats{"Gemini Color RGB (/camera/color/image_raw)"};

  M9BenchmarkNode() : Node("m9_sync_benchmark") {
    auto qos = rclcpp::SensorDataQoS();

    cb_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cb_group_cam_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions opt_lidar;
    opt_lidar.callback_group = cb_group_lidar_;

    rclcpp::SubscriptionOptions opt_cam;
    opt_cam.callback_group = cb_group_cam_;

    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          double host_now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
          double stamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
          lidar_stats.add(stamp_sec, host_now);
        }, opt_lidar);

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/rslidar_imu_data", qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          double host_now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
          double stamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
          imu_stats.add(stamp_sec, host_now);
        }, opt_lidar);

    sub_left_ir_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/left_ir/image_raw", qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          double host_now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
          double stamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
          left_ir_stats.add(stamp_sec, host_now);
        }, opt_cam);

    sub_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          double host_now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
          double stamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
          rgb_stats.add(stamp_sec, host_now);
        }, opt_cam);
  }

  void computeInterSensorPhase() {
    std::lock_guard<std::mutex> lock1(lidar_stats.mtx);
    std::lock_guard<std::mutex> lock2(left_ir_stats.mtx);

    if (lidar_stats.header_stamps.empty() || left_ir_stats.header_stamps.empty()) return;

    std::vector<double> phase_deltas_ms;
    for (double l_stamp : lidar_stats.header_stamps) {
      // Find nearest IR frame
      double min_d = 1e9;
      for (double c_stamp : left_ir_stats.header_stamps) {
        double d = (c_stamp - l_stamp) * 1000.0;
        if (std::abs(d) < std::abs(min_d)) {
          min_d = d;
        }
      }
      if (std::abs(min_d) < 1000.0) {
        phase_deltas_ms.push_back(min_d);
      }
    }

    if (phase_deltas_ms.size() >= 5) {
      std::sort(phase_deltas_ms.begin(), phase_deltas_ms.end());
      double p50 = phase_deltas_ms[phase_deltas_ms.size() * 0.50];
      double p95 = phase_deltas_ms[phase_deltas_ms.size() * 0.95];
      double p99 = phase_deltas_ms[phase_deltas_ms.size() * 0.99];
      double mean = std::accumulate(phase_deltas_ms.begin(), phase_deltas_ms.end(), 0.0) / phase_deltas_ms.size();

      std::cout << "\n🔗 Inter-Sensor Hardware Sync Phase Analysis (Airy LiDAR Sweep Epoch vs Gemini IR Frame Epoch):\n";
      std::cout << "   • Matched Sweep-to-Frame Pairs: " << phase_deltas_ms.size() << "\n";
      std::cout << "   • Mean Phase Offset (IR - LiDAR): " << std::fixed << std::setprecision(2) << mean << " ms\n";
      std::cout << "   • Median Offset (P50):            " << p50 << " ms\n";
      std::cout << "   • P95 Offset:                     " << p95 << " ms\n";
      std::cout << "   • P99 Offset:                     " << p99 << " ms\n";
      std::cout << "   • Phase Offset Range:             [" << phase_deltas_ms.front() << " .. " << phase_deltas_ms.back() << "] ms\n";
    }
  }

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_lidar_;
  rclcpp::CallbackGroup::SharedPtr cb_group_cam_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_left_ir_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<M9BenchmarkNode>();

  double duration = 10.0;
  if (argc > 1) {
    duration = std::atof(argv[1]);
  }

  std::cout << ">>> [M9BenchmarkNode] Initialized. Collecting " << duration << "s of synchronized data..." << std::endl;

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
  executor->add_node(node);

  auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    executor->spin_some(std::chrono::milliseconds(20));
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    if (elapsed >= duration) break;
  }

  std::cout << "\n================================================================================" << std::endl;
  std::cout << " MILESTONE M9: PHYSICAL HARDWARE SYNC & MULTI-SENSOR STABILITY REPORT (" << duration << " s)" << std::endl;
  std::cout << "================================================================================" << std::endl;

  node->lidar_stats.printReport(duration);
  node->imu_stats.printReport(duration);
  node->left_ir_stats.printReport(duration);
  node->rgb_stats.printReport(duration);
  node->computeInterSensorPhase();

  std::cout << "\n================================================================================\n" << std::endl;
  rclcpp::shutdown();
  return 0;
}
