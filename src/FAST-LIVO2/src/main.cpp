#include "LIVMapper.h"
#include <thread>

int main(int argc, char **argv)
{
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto nh = std::make_shared<rclcpp::Node>("laserMapping", options);
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping", options);
  mapper.initializeSubscribersAndPublishers(nh, it_);

  // Multi-threaded executor on 2 dedicated threads for decoupled ROS 2 DDS transport
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
  executor->add_node(nh);
  std::thread spin_thread([executor]() {
    executor->spin();
  });

  mapper.run(nh);

  executor->cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
