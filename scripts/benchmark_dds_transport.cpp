#include <iostream>
#include <vector>
#include <chrono>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

struct StreamMetrics {
  uint64_t count = 0;
  uint64_t gaps = 0;
  bool has_last_time = false;
  std::chrono::steady_clock::time_point last_time;
  std::vector<double> intervals;
  std::mutex mtx;

  void record(double gap_threshold_sec) {
    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mtx);
    ++count;
    if (has_last_time) {
      double dt = std::chrono::duration<double>(now - last_time).count();
      intervals.push_back(dt);
      if (dt > gap_threshold_sec) {
        ++gaps;
      }
    } else {
      has_last_time = true;
    }
    last_time = now;
  }

  void print(const std::string& name, double duration_sec, double expected_rate_hz) {
    std::lock_guard<std::mutex> lock(mtx);
    double rate = static_cast<double>(count) / duration_sec;
    double median_dt = 0.0, p95_dt = 0.0, p99_dt = 0.0, mean_dt = 0.0, min_dt = 0.0, max_dt = 0.0;
    if (intervals.size() > 1) {
      std::vector<double> sorted = intervals;
      std::sort(sorted.begin(), sorted.end());
      median_dt = sorted[sorted.size() / 2] * 1000.0;
      p95_dt = sorted[static_cast<size_t>(sorted.size() * 0.95)] * 1000.0;
      p99_dt = sorted[static_cast<size_t>(sorted.size() * 0.99)] * 1000.0;
      mean_dt = (std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size()) * 1000.0;
      min_dt = sorted.front() * 1000.0;
      max_dt = sorted.back() * 1000.0;
    }
    double delivery_pct = (rate / expected_rate_hz) * 100.0;
    std::cout << "  - " << name << ": " << count << " msgs | " 
              << rate << " Hz (" << delivery_pct << "%) | Median: " << median_dt 
              << " ms | P95: " << p95_dt << " ms | P99: " << p99_dt 
              << " ms | Max: " << max_dt << " ms | Gaps: " << gaps << std::endl;
  }
};

class BenchmarkNode : public rclcpp::Node {
public:
  BenchmarkNode(const std::string& name, bool use_sensor_qos, int queue_depth, bool use_callback_groups)
      : Node(name) {
    rclcpp::QoS qos = use_sensor_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(queue_depth);
    if (use_sensor_qos && queue_depth != 10) {
      qos = rclcpp::SensorDataQoS().keep_last(queue_depth);
    }

    rclcpp::SubscriptionOptions sub_opts_lidar;
    rclcpp::SubscriptionOptions sub_opts_imu;
    rclcpp::SubscriptionOptions sub_opts_cam;

    if (use_callback_groups) {
      cb_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      cb_group_imu_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      cb_group_cam_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

      sub_opts_lidar.callback_group = cb_group_lidar_;
      sub_opts_imu.callback_group = cb_group_imu_;
      sub_opts_cam.callback_group = cb_group_cam_;
    }

    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          lidar_metrics_.record(0.150);
        }, sub_opts_lidar);

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/rslidar_imu_data", qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          imu_metrics_.record(0.010);
        }, sub_opts_imu);

    sub_cam0_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/left_ir/image_raw", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          cam0_metrics_.record(0.025);
        }, sub_opts_cam);

    sub_cam1_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/right_ir/image_raw", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          cam1_metrics_.record(0.025);
        }, sub_opts_cam);

    sub_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          rgb_metrics_.record(0.050);
        }, sub_opts_cam);
  }

  void printReport(double duration_sec, const std::string& config_name) {
    std::cout << "\n================================================================================" << std::endl;
    std::cout << " CONFIGURATION: " << config_name << " (Duration: " << duration_sec << " s)" << std::endl;
    std::cout << "================================================================================" << std::endl;
    lidar_metrics_.print("Airy LiDAR (/rslidar_points)", duration_sec, 10.0);
    imu_metrics_.print("Airy IMU (/rslidar_imu_data)", duration_sec, 200.0);
    cam0_metrics_.print("Left IR (/camera/left_ir/image_raw)", duration_sec, 60.0);
    cam1_metrics_.print("Right IR (/camera/right_ir/image_raw)", duration_sec, 60.0);
    rgb_metrics_.print("RGB (/camera/color/image_raw)", duration_sec, 30.0);
    std::cout << "================================================================================\n" << std::endl;
  }

  StreamMetrics lidar_metrics_;
  StreamMetrics imu_metrics_;
  StreamMetrics cam0_metrics_;
  StreamMetrics cam1_metrics_;
  StreamMetrics rgb_metrics_;

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_lidar_;
  rclcpp::CallbackGroup::SharedPtr cb_group_imu_;
  rclcpp::CallbackGroup::SharedPtr cb_group_cam_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam0_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  int mode = 1;
  double duration = 15.0;
  if (argc > 1) mode = std::atoi(argv[1]);
  if (argc > 2) duration = std::atof(argv[2]);

  std::string config_name = "";
  bool use_sensor_qos = true;
  int queue_depth = 10;
  bool use_cb_groups = false;
  int num_threads = 1;

  switch (mode) {
    case 1:
      config_name = "Mode 1: SingleThreadedExecutor + Reliable(10)";
      use_sensor_qos = false;
      queue_depth = 10;
      use_cb_groups = false;
      num_threads = 1;
      break;
    case 2:
      config_name = "Mode 2: SingleThreadedExecutor + SensorDataQoS(10)";
      use_sensor_qos = true;
      queue_depth = 10;
      use_cb_groups = false;
      num_threads = 1;
      break;
    case 3:
      config_name = "Mode 3: MultiThreadedExecutor(4) + SensorDataQoS(10) [Flat Callbacks]";
      use_sensor_qos = true;
      queue_depth = 10;
      use_cb_groups = false;
      num_threads = 4;
      break;
    case 4:
      config_name = "Mode 4: MultiThreadedExecutor(4) + SensorDataQoS(5) + Isolated Callback Groups";
      use_sensor_qos = true;
      queue_depth = 5;
      use_cb_groups = true;
      num_threads = 4;
      break;
    case 5:
      config_name = "Mode 5: MultiThreadedExecutor(6) + SensorDataQoS(2) + Isolated Callback Groups";
      use_sensor_qos = true;
      queue_depth = 2;
      use_cb_groups = true;
      num_threads = 6;
      break;
    default:
      config_name = "Custom Mode";
      break;
  }

  auto node = std::make_shared<BenchmarkNode>("dds_benchmark_node", use_sensor_qos, queue_depth, use_cb_groups);

  if (num_threads == 1) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto start_time = std::chrono::steady_clock::now();
    std::thread spin_thread([&executor]() {
      executor.spin();
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(duration * 1000)));
    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
  } else {
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);
    executor.add_node(node);
    auto start_time = std::chrono::steady_clock::now();
    std::thread spin_thread([&executor]() {
      executor.spin();
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(duration * 1000)));
    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
  }

  node->printReport(duration, config_name);
  rclcpp::shutdown();
  return 0;
}
