#ifndef SO101_HARDWARE__SO101SYSTEM_HPP_
#define SO101_HARDWARE__SO101SYSTEM_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <chrono>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "so101_hardware/sts_bus.hpp"
#include "so101_hardware/sts_registers.hpp"


namespace so101_hardware
{
class SO101System : public hardware_interface::SystemInterface{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kNumJoints = 6;
  static constexpr std::size_t kGripperIndex = kNumJoints - 1;

  // Hardware level config
  std::string serial_port_;
  int baud_rate_{0};

  // Per-joint config, indexed in URDF joint order
  std::array<std::uint8_t, kNumJoints> servo_ids_{};
  std::array<int, kNumJoints> signs_{};
  std::array<std::int32_t, kNumJoints> offsets_{};

  std::array<double, kNumJoints> min_positions_{};
  std::array<double, kNumJoints> max_positions_{};
  std::array<double, kNumJoints> max_velocities_{};

  //Interface names, built once here so read()/write() never builds strings
  std::array<std::string, kNumJoints> pos_state_names_;
  std::array<std::string, kNumJoints> vel_state_names_;
  std::array<std::string, kNumJoints> pos_cmd_names_;
  std::string gripper_effort_name_;


  bool ping_all();
  bool read_and_validate_eeprom();
  void log_eeprom_table();
  std::int32_t rad_to_counts(std::size_t joint, double rad) const;

  StsBus bus_;
  std::array<std::int32_t, kNumJoints> expected_homing_offsets_{};
  std::array<sts::ServoConfig, kNumJoints> servo_configs_{};

};
}

#endif // SO101_HARDWARE__SO101SYSTEM_HPP_
