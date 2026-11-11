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
  bool write_ram(
    std::size_t i, std::uint8_t address, const std::uint8_t * data, std::uint8_t length,
    std::uint8_t & servo_error);
  bool torque_off_all();
  void log_supply_stats(const char * when);

  StsBus bus_;

  // Cached in on_configure, so read() writes through them without name lookups.
  std::array<hardware_interface::StateInterface::SharedPtr, kNumJoints> pos_states_;
  std::array<hardware_interface::StateInterface::SharedPtr, kNumJoints> vel_states_;
  std::array<hardware_interface::CommandInterface::SharedPtr, kNumJoints> pos_cmds_;
  // Raw Present_Position (counts) from the last good read(); on_activate seeds Goal_Position.
  std::array<std::int32_t, kNumJoints> present_counts_{};

  // write(): the goal sent last cycle (rad), seeded by on_activate, and its wire bytes.
  std::array<double, kNumJoints> last_goal_{};
  std::array<std::uint8_t, kNumJoints * 2> goal_raw_{};

  // Supply watch since the last on_activate: lowest Present_Voltage (0.1 V), hottest servo (C).
  std::array<std::uint8_t, kNumJoints> min_voltage_dv_{};
  std::array<std::uint8_t, kNumJoints> max_temperature_c_{};

  // One SYNC READ fills this: per servo [pos lo, pos hi, vel lo, vel hi, load lo, load hi].
  std::array<std::uint8_t, kNumJoints * sts::reg::kPresentBlockLength> present_raw_{};

  std::array<std::int32_t, kNumJoints> expected_homing_offsets_{};
  std::array<sts::ServoConfig, kNumJoints> servo_configs_{};

};
}

#endif // SO101_HARDWARE__SO101SYSTEM_HPP_
