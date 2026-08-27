#include <chrono>
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

class CadenceMonitorNode : public rclcpp::Node {
public:
    CadenceMonitorNode() : Node("cpp_cadence_monitor") {
        auto qos = rclcpp::SensorDataQoS();
        
        sub_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                raw_count_++;
                double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                raw_header_stamps_.push_back(t);
                raw_arrival_stamps_.push_back(std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            });

        sub_reg_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                reg_count_++;
                double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                reg_header_stamps_.push_back(t);
            });

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_count_++;
            });
    }

    size_t raw_count_ = 0;
    size_t reg_count_ = 0;
    size_t odom_count_ = 0;
    std::vector<double> raw_header_stamps_;
    std::vector<double> raw_arrival_stamps_;
    std::vector<double> reg_header_stamps_;

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_raw_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_reg_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    double duration_sec = 60.0;
    if (argc > 1) {
        duration_sec = std::atof(argv[1]);
    }

    auto node = std::make_shared<CadenceMonitorNode>();
    std::cout << "[CPP MONITOR] Listening to /rslidar_points, /cloud_registered, /aft_mapped_to_init for " 
              << duration_sec << " seconds..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) {
        executor.spin_some();
        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= duration_sec) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    double total_elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    std::cout << "\n============================================================" << std::endl;
    std::cout << "          C++ LIGHTWEIGHT SUBSCRIBER 60-SEC REPORT          " << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "Elapsed Duration:              " << total_elapsed << " s" << std::endl;
    std::cout << "Raw /rslidar_points received:  " << node->raw_count_ << " (" << node->raw_count_ / total_elapsed << " Hz)" << std::endl;
    std::cout << "FAST-LIVO2 odom received:      " << node->odom_count_ << " (" << node->odom_count_ / total_elapsed << " Hz)" << std::endl;
    std::cout << "/cloud_registered received:    " << node->reg_count_ << " (" << node->reg_count_ / total_elapsed << " Hz)" << std::endl;
    
    if (node->raw_header_stamps_.size() > 1) {
        std::vector<double> diffs;
        for (size_t i = 1; i < node->raw_header_stamps_.size(); ++i) {
            diffs.push_back((node->raw_header_stamps_[i] - node->raw_header_stamps_[i-1]) * 1000.0);
        }
        std::sort(diffs.begin(), diffs.end());
        double p50 = diffs[diffs.size() * 0.50];
        double p95 = diffs[diffs.size() * 0.95];
        double min_dt = diffs.front();
        double max_dt = diffs.back();
        std::cout << "Raw Header dt P50 (Median):    " << p50 << " ms" << std::endl;
        std::cout << "Raw Header dt P95:             " << p95 << " ms" << std::endl;
        std::cout << "Raw Header dt Min / Max:       " << min_dt << " ms / " << max_dt << " ms" << std::endl;
    }
    std::cout << "============================================================" << std::endl;

    rclcpp::shutdown();
    return 0;
}
