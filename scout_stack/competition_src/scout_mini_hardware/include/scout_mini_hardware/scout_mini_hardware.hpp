// Copyright (c) 2023, ROAS Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SCOUT_MINI_HARDWARE__SCOUT_MINI_HAREWARE_HPP_
#define SCOUT_MINI_HARDWARE__SCOUT_MINI_HAREWARE_HPP_

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"

#include "scout_mini_msgs/msg/driver_state.hpp"
#include "scout_mini_msgs/msg/fault_state.hpp"
#include "scout_mini_msgs/msg/light_command.hpp"
#include "scout_mini_msgs/msg/light_state.hpp"
#include "scout_mini_msgs/msg/motor_state.hpp"
#include "scout_mini_msgs/msg/robot_state.hpp"

#include "scout_mini_hardware/visibility_control.h"

namespace scout_mini_hardware
{
class ScoutMiniHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ScoutMiniHardware)

  SCOUT_MINI_HARDWARE_PUBLIC
  ~ScoutMiniHardware() override;

  SCOUT_MINI_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  SCOUT_MINI_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SCOUT_MINI_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SCOUT_MINI_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  SCOUT_MINI_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  SCOUT_MINI_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  SCOUT_MINI_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * \brief Callback for reading from hardware interface
   */
  void receive();

  /**
   * \brief Parse the robot state message
   * \param data Robot state message converted to binary number
   */
  void robotState(uint8_t* data);

  /**
   * \brief Parse the motor state message
   * \param index Index of motor
   * \param data Motor state converted to binary number
   */
  void motorState(size_t index, uint8_t* data);

  /**
   * \brief Parse the driver state message
   * \param index Index of driver
   * \param data Driver state converted to binary number
   */
  void driverState(size_t index, uint8_t* data);

  /**
   * \brief Parse the light state message
   * \param data Light state message converted to binary number
   */
  void lightState(uint8_t* data);

  /**
   * \brief Parse the velocity message
   * \param data Velocity message converted to binary number
   */
  void velocity(uint8_t* data);

  /**
   * \brief Parse the position message
   * \param data Position message converted to binary number
   */
  void position(uint8_t* data);

  /**
   * \brief Publish state data
   */
  void publish();

  /**
   * \brief Light command callback
   * \param msg Light command message
   */
  void lightCmdCallback(const scout_mini_msgs::msg::LightCommand::SharedPtr& msg);

  /// Robot hardware interface node
  std::shared_ptr<rclcpp::Node> node_;

private:
  /// Start and stop the bounded-time CAN receive loop.
  void startReceiver();
  void stopReceiver() noexcept;

  /// Send a velocity command after conversion to the Scout Mini CAN format.
  bool sendMotionCommand(double linear, double angular);

  /// Hardware interface
  std::vector<double> hw_commands_, hw_positions_, hw_velocities_, hw_efforts_;

  /// rostopic publisher
  rclcpp::Publisher<scout_mini_msgs::msg::DriverState>::SharedPtr pub_driver_state_;
  rclcpp::Publisher<scout_mini_msgs::msg::LightState>::SharedPtr pub_light_state_;
  rclcpp::Publisher<scout_mini_msgs::msg::MotorState>::SharedPtr pub_motor_state_;
  rclcpp::Publisher<scout_mini_msgs::msg::RobotState>::SharedPtr pub_robot_state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hardware_ready_;
  std::shared_ptr<realtime_tools::RealtimePublisher<scout_mini_msgs::msg::DriverState>> rp_driver_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<scout_mini_msgs::msg::LightState>> rp_light_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<scout_mini_msgs::msg::MotorState>> rp_motor_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<scout_mini_msgs::msg::RobotState>> rp_robot_state_;

  /// rostopic subscriber
  rclcpp::Subscription<scout_mini_msgs::msg::LightCommand>::SharedPtr sub_led_cmd_;

  /// Socket CAN
  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::unique_ptr<std::thread> receiver_thread_;
  std::string interface_;
  std::chrono::nanoseconds timeout_ns_, interval_ns_;
  std::atomic<bool> receiver_running_{ false };
  std::atomic<bool> hardware_ready_{ false };
  std::mutex sender_mutex_;
  std::mutex state_mutex_;
  std::chrono::steady_clock::time_point last_velocity_rx_;
  std::chrono::steady_clock::time_point last_robot_state_rx_;
  std::array<std::chrono::steady_clock::time_point, 4> last_driver_state_rx_;
  bool velocity_received_{ false };
  bool robot_state_received_{ false };
  std::array<bool, 4> driver_state_received_{{ false, false, false, false }};
  bool velocity_stale_reported_{ false };

  /// Robot parameters
  double wheel_radius_, wheel_separation_;
  double velocity_timeout_sec_{ 0.25 };
  double robot_state_timeout_sec_{ 0.50 };
  double driver_state_timeout_sec_{ 1.00 };
  double max_linear_velocity_{ 1.5 };
  double max_angular_velocity_{ 1.0 };
  bool enable_drive_{ true };
};
}  // namespace scout_mini_hardware

#endif  // SCOUT_MINI_HARDWARE__SCOUT_MINI_HAREWARE_HPP_
