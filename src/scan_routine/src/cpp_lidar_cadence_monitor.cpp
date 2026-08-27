#include <chrono>
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>

class ComprehensiveCadenceMonitor : public rclcpp::Node {
public:
    ComprehensiveCadenceMonitor() : Node("cpp_comprehensive_monitor") {
        auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group;

        auto sensor_qos = rclcpp::SensorDataQoS();
        auto default_qos = rclcpp::QoS(10);
        
        // 1. Raw LiDAR Points
        sub_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", sensor_qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                raw_lidar_cnt_++;
                double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                raw_hdr_stamps_.push_back(t);
            }, sub_opts);

        // 2. IMU
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/rslidar_imu_data", sensor_qos,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                imu_cnt_++;
            }, sub_opts);

        // 3. FAST-LIVO2 Registered Cloud
        sub_reg_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", default_qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                reg_lidar_cnt_++;
                double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                reg_hdr_stamps_.push_back(t);
            }, sub_opts);

        // 4. FAST-LIVO2 Odometry
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", default_qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_cnt_++;
            }, sub_opts);

        // 5. Gemini Color RGB
        sub_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", sensor_qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                rgb_cnt_++;
                double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                rgb_hdr_stamps_.push_back(t);
            }, sub_opts);

        // 6. Gemini Depth
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", sensor_qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                depth_cnt_++;
                double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                depth_hdr_stamps_.push_back(t);
            }, sub_opts);
    }

    size_t raw_lidar_cnt_ = 0;
    size_t imu_cnt_ = 0;
    size_t reg_lidar_cnt_ = 0;
    size_t odom_cnt_ = 0;
    size_t rgb_cnt_ = 0;
    size_t depth_cnt_ = 0;

    std::vector<double> raw_hdr_stamps_;
    std::vector<double> reg_hdr_stamps_;
    std::vector<double> rgb_hdr_stamps_;
    std::vector<double> depth_hdr_stamps_;

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_raw_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_reg_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
};

void analyze_stamps(const std::string& name, const std::vector<double>& stamps, double dur) {
    std::cout << "--- " << name << " ---" << std::endl;
    std::cout << "  Count: " << stamps.size() << " | Rate: " << (stamps.size() / dur) << " Hz" << std::endl;
    if (stamps.size() < 2) return;
    std::vector<double> diffs;
    for (size_t i = 1; i < stamps.size(); ++i) {
        diffs.push_back((stamps[i] - stamps[i-1]) * 1000.0);
    }
    std::sort(diffs.begin(), diffs.end());
    std::cout << "  P1: " << diffs[diffs.size() * 0.01] << " ms | P25: " << diffs[diffs.size() * 0.25]
              << " ms | P50 (Median): " << diffs[diffs.size() * 0.50] << " ms | P95: " << diffs[diffs.size() * 0.95]
              << " ms | Max: " << diffs.back() << " ms" << std::endl;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    double duration_sec = 60.0;
    if (argc > 1) {
        duration_sec = std::atof(argv[1]);
    }

    auto node = std::make_shared<ComprehensiveCadenceMonitor>();
    std::cout << "[CPP COMPREHENSIVE MONITOR] Listening to all 6 streams for " 
              << duration_sec << " seconds..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    executor.add_node(node);

    auto spin_thread = std::thread([&]() {
        executor.spin();
    });

    std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();

    auto end_time = std::chrono::steady_clock::now();
    double total_elapsed = std::chrono::duration<double>(end_time - start_time).count();

    std::cout << "\n================================================================================" << std::endl;
    std::cout << "              60-SECOND C++ MULTI-STREAM CADENCE REPORT                         " << std::endl;
    std::cout << "================================================================================" << std::endl;
    std::cout << "Actual Elapsed Time: " << total_elapsed << " s" << std::endl;
    std::cout << "IMU Messages:        " << node->imu_cnt_ << " (" << node->imu_cnt_ / total_elapsed << " Hz)" << std::endl;
    std::cout << "FAST-LIVO2 Odometry: " << node->odom_cnt_ << " (" << node->odom_cnt_ / total_elapsed << " Hz)" << std::endl;
    
    analyze_stamps("Airy Raw LiDAR (/rslidar_points)", node->raw_hdr_stamps_, total_elapsed);
    analyze_stamps("FAST-LIVO2 Registered Cloud (/cloud_registered)", node->reg_hdr_stamps_, total_elapsed);
    analyze_stamps("Gemini Color RGB (/camera/color/image_raw)", node->rgb_hdr_stamps_, total_elapsed);
    analyze_stamps("Gemini Depth (/camera/depth/image_raw)", node->depth_hdr_stamps_, total_elapsed);
    std::cout << "================================================================================" << std::endl;

    rclcpp::shutdown();
    return 0;
}
