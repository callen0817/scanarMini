#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <mutex>
#include <thread>

struct StreamStats {
  std::string name;
  std::atomic<uint64_t> count{0};
  std::vector<double> intervals;
  std::chrono::steady_clock::time_point last_time;
  std::mutex mtx;

  void record() {
    auto now = std::chrono::steady_clock::now();
    uint64_t c = ++count;
    std::lock_guard<std::mutex> lock(mtx);
    if (c > 1) {
      double dt = std::chrono::duration<double, std::milli>(now - last_time).count();
      intervals.push_back(dt);
    }
    last_time = now;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int duration_sec = 15;
  if (argc > 1) duration_sec = std::atoi(argv[1]);

  auto node = std::make_shared<rclcpp::Node>("benchmark_livo_consumer");
  StreamStats lidar_stat{"Airy LiDAR (/rslidar_points)"};
  StreamStats imu_stat{"Airy IMU (/rslidar_imu_data)"};
  StreamStats left_ir_stat{"Left IR (/camera/left_ir/image_raw)"};

  auto cb_group_lidar = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cb_group_imu   = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cb_group_img   = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions opt_lidar;
  opt_lidar.callback_group = cb_group_lidar;
  rclcpp::SubscriptionOptions opt_imu;
  opt_imu.callback_group = cb_group_imu;
  rclcpp::SubscriptionOptions opt_img;
  opt_img.callback_group = cb_group_img;

  auto sub_lidar = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/rslidar_points", rclcpp::SensorDataQoS(),
    [&lidar_stat](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      (void)msg;
      lidar_stat.record();
    }, opt_lidar);

  auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(
    "/rslidar_imu_data", rclcpp::SensorDataQoS(),
    [&imu_stat](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      (void)msg;
      imu_stat.record();
    }, opt_imu);

  auto sub_left_ir = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/left_ir/image_raw", rclcpp::SensorDataQoS(),
    [&left_ir_stat](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      (void)msg;
      left_ir_stat.record();
    }, opt_img);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  std::cout << "\n================================================================================" << std::endl;
  std::cout << " BENCHMARK: FAST-LIVO2 (LiDAR + IMU + Left IR) [ISOLATED CALLBACK GROUPS] (" 
            << duration_sec << " s)" << std::endl;
  std::cout << "================================================================================" << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(duration_sec));

  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();

  auto print_stat = [duration_sec](StreamStats& s, double expected_rate) {
    std::lock_guard<std::mutex> lock(s.mtx);
    double rate = s.count.load() / (double)duration_sec;
    double pct = (rate / expected_rate) * 100.0;
    std::cout << "  - " << s.name << ": " << s.count.load() << " msgs | " 
              << rate << " Hz (" << pct << "%)";
    if (!s.intervals.empty()) {
      std::vector<double> sorted = s.intervals;
      std::sort(sorted.begin(), sorted.end());
      double median = sorted[sorted.size() / 2];
      double p95 = sorted[static_cast<size_t>(sorted.size() * 0.95)];
      double p99 = sorted[static_cast<size_t>(sorted.size() * 0.99)];
      double max = sorted.back();
      std::cout << " | Median: " << median << " ms | P95: " << p95 << " ms | P99: " << p99 
                << " ms | Max: " << max << " ms";
    }
    std::cout << std::endl;
  };

  print_stat(lidar_stat, 10.0);
  print_stat(imu_stat, 200.0);
  print_stat(left_ir_stat, 60.0);
  std::cout << "================================================================================\n" << std::endl;

  rclcpp::shutdown();
  return 0;
}
