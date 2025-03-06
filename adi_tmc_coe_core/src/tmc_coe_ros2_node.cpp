/**
 * Copyright (c) 2024-2025 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "adi_tmc_coe_core/tmc_coe_ros2.hpp"

TmcCoeROS2 * p_tmc_coe_ros2 = nullptr;
bool g_shutdown = false;

void graceful_shutdown();
void signal_callback_handler(int signum);

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tmc_coe_ros2_node");
  RCLCPP_INFO_STREAM(node->get_logger(), "Starting " << node->get_name() << " ...");

  RCLCPP_INFO_STREAM(node->get_logger(), "Installing new signal handlers...");
  rclcpp::uninstall_signal_handlers();
  std::signal(SIGINT, signal_callback_handler);
  std::signal(SIGTERM, signal_callback_handler);

  rclcpp::executors::MultiThreadedExecutor exec;

  p_tmc_coe_ros2 = new TmcCoeROS2(node);
  if (p_tmc_coe_ros2->initialize())
  {
    exec.add_node(node);
    exec.spin();
  }

  // Else if exits from main loop
  graceful_shutdown();

  return 0;
}

void graceful_shutdown()
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("tmc_coe_ros2_node shutdown"),
    "Initiating graceful shutdown...");
  p_tmc_coe_ros2->deInitialize();
  delete p_tmc_coe_ros2;
  p_tmc_coe_ros2 = nullptr;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("tmc_coe_ros2_node shutdown"), "Successfully shutdown...");
}

void signal_callback_handler(int signum)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("tmc_coe_ros2_node shutdown"), "Caught signal: " << signum <<
    ". Terminating...");
  rclcpp::shutdown();
}
