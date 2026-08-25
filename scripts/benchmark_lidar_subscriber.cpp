#include <iostream>
#include <vector>
#include <chrono>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

class LidarSubscriberBenchmark : public rclcpp::Node {
public:
  LidarSubscriberBenchmark() : Node("lidar_subscriber_benchmark") {
    // Subscribe using SensorDataQoS (Best effort, volatile, depth 10)
    auto qos = rclcpp::SensorDataQoS();
    
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", qos,
        std::bind(&LidarSubscriberBenchmark::cloudCallback, this, std::placeholders::_1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/rslidar_imu_data", qos,
        std::bind(&LidarSubscriberBenchmark::imuCallback, this, std::placeholders::_1));

    std::cout << ">>> [LidarSubscriberBenchmark] Initialized with SensorDataQoS. Listening to /rslidar_points and /rslidar_imu_data..." << std::endl;
  }

  void printStats(double duration_sec) {
    std::cout << "\n================================================================================" << std::endl;
    std::cout << " STAGE D SUBSCRIBER BENCHMARK RESULTS (Test Duration: " << duration_sec << " s)" << std::endl;
    std::cout << "================================================================================" << std::endl;

    // Cloud Stats
    if (cloud_intervals_.size() > 1) {
      double cloud_rate = static_cast<double>(cloud_count_) / duration_sec;
      std::vector<double> sorted_intervals = cloud_intervals_;
      std::sort(sorted_intervals.begin(), sorted_intervals.end());
      double median_dt = sorted_intervals[sorted_intervals.size() / 2];
      double mean_dt = std::accumulate(sorted_intervals.begin(), sorted_intervals.end(), 0.0) / sorted_intervals.size();
      double min_dt = sorted_intervals.front();
      double max_dt = sorted_intervals.back();

      std::cout << "PointCloud (/rslidar_points):" << std::endl;
      std::cout << "  - Total Messages: " << cloud_count_ << std::endl;
      std::cout << "  - Delivered Rate: " << cloud_rate << " Hz" << std::endl;
      std::cout << "  - Median Interval: " << (median_dt * 1000.0) << " ms" << std::endl;
      std::cout << "  - Mean Interval:   " << (mean_dt * 1000.0) << " ms" << std::endl;
      std::cout << "  - Min/Max Interval: " << (min_dt * 1000.0) << " ms / " << (max_dt * 1000.0) << " ms" << std::endl;
      std::cout << "  - Gaps (>150ms):   " << cloud_gaps_ << std::endl;
    } else {
      std::cout << "PointCloud (/rslidar_points): NO DATA RECEIVED!" << std::endl;
    }

    // IMU Stats
    if (imu_intervals_.size() > 1) {
      double imu_rate = static_cast<double>(imu_count_) / duration_sec;
      std::vector<double> sorted_intervals = imu_intervals_;
      std::sort(sorted_intervals.begin(), sorted_intervals.end());
      double median_dt = sorted_intervals[sorted_intervals.size() / 2];
      std::cout << "\nIMU (/rslidar_imu_data):" << std::endl;
      std::cout << "  - Total Messages: " << imu_count_ << std::endl;
      std::cout << "  - Delivered Rate: " << imu_rate << " Hz" << std::endl;
      std::cout << "  - Median Interval: " << (median_dt * 1000.0) << " ms" << std::endl;
    }

    std::cout << "================================================================================\n" << std::endl;
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto now = std::chrono::steady_clock::now();
    ++cloud_count_;
    if (has_last_cloud_time_) {
      double dt = std::chrono::duration<double>(now - last_cloud_time_).count();
      cloud_intervals_.push_back(dt);
      if (dt > 0.150) {
        ++cloud_gaps_;
      }
    } else {
      has_last_cloud_time_ = true;
    }
    last_cloud_time_ = now;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    auto now = std::chrono::steady_clock::now();
    ++imu_count_;
    if (has_last_imu_time_) {
      double dt = std::chrono::duration<double>(now - last_imu_time_).count();
      imu_intervals_.push_back(dt);
    } else {
      has_last_imu_time_ = true;
    }
    last_imu_time_ = now;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  uint64_t cloud_count_ = 0;
  uint64_t cloud_gaps_ = 0;
  bool has_last_cloud_time_ = false;
  std::chrono::steady_clock::time_point last_cloud_time_;
  std::vector<double> cloud_intervals_;

  uint64_t imu_count_ = 0;
  bool has_last_imu_time_ = false;
  std::chrono::steady_clock::time_point last_imu_time_;
  std::vector<double> imu_intervals_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarSubscriberBenchmark>();

  double test_duration = 30.0;
  if (argc > 1) {
    test_duration = std::atof(argv[1]);
  }

  // Spin in background thread with MultiThreadedExecutor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
  executor->add_node(node);
  std::thread spin_thread([executor]() {
    executor->spin();
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(test_duration * 1000)));

  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();

  node->printStats(test_duration);
  return 0;
}
