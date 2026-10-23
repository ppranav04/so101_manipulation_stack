#include "so101_hardware/SO101System.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdlib>
#include <string>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace{
    constexpr long kMaxServoId = 253;
    constexpr long kMaxAbsOffsetCounts = 4096;
    constexpr std::array<long, 8> kAllowedBaudRates = {
        1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};


// Strict integer parse: the whole string must be a number. Optional single leading '+'.
bool parse_long(const std::string & text, long & out){
    const char * first = text.data();
    const char * last = text.data()  + text.size();
    if (first != last && *first == '+'){
        ++first;
        if (first != last && *first == '-'){
            return false;
        }
    }

    if (first == last){
        return false;
    }

    const auto [ptr, ec] = std::from_chars(first, last, out);
    return ec == std::errc{} && ptr == last;
}

// Required parameter lookup. Returns false if the key is missing.
bool find_param(
    const std::unordered_map<std::string, std::string> & params,
    const std::string & key, std::string & out){
        const auto it = params.find(key);
        if (it == params.end()){
            return false;
        }
        out = it->second;
        return true;
    }
}

namespace so101_hardware {
        hardware_interface::CallbackReturn SO101System::on_init(const hardware_interface::HardwareComponentInterfaceParams & params){
            using hardware_interface::HW_IF_EFFORT;
            using hardware_interface::HW_IF_POSITION;
            using hardware_interface::HW_IF_VELOCITY;

            // 1. Base class parses the <ros2_control> block into info_ and builds interface descriptions
            if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS){
                return CallbackReturn::ERROR;
            }

            std::string value;

            // 2. Hardware level params
            if (!find_param(info_.hardware_parameters, "serial_port", serial_port_) || serial_port_.empty()){
                RCLCPP_ERROR(get_logger(), "Missing or empty <hardware> param 'serial_port'");
                return CallbackReturn::ERROR;
            }

            long baud = 0;
            if (!find_param(info_.hardware_parameters, "baud_rate", value) || !parse_long(value, baud)){
                RCLCPP_ERROR(get_logger(), "Missing or non-integer <hardware> param 'baud_rate'");
                return CallbackReturn::ERROR;
            }

            if (std::find(kAllowedBaudRates.begin(), kAllowedBaudRates.end(),  baud) == kAllowedBaudRates.end()){
                RCLCPP_ERROR(get_logger(), "baud_rate %ld is not a rate the STS3215 supports", baud);
                return CallbackReturn::ERROR;
            }

            baud_rate_ = static_cast<int>(baud);

            // 3, Joint count
            if (info_.joints.size() != kNumJoints){
                RCLCPP_ERROR(
                    get_logger(), "Expected %zu joints, got %zu", kNumJoints, info_.joints.size()
                );
                return CallbackReturn::ERROR;
            }

            // 4. Per-joint interface contract + params.
            std::array<bool, kMaxServoId + 1> id_taken{};
            for (std::size_t i = 0; i < kNumJoints; ++i){
                const auto & joint = info_.joints[i];
                const char * jname = joint.name.c_str();
                const bool is_gripper = (i == kGripperIndex);

                // Command has exactly one, position
                if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != HW_IF_POSITION){
                    RCLCPP_ERROR(get_logger(), "Joint '%s': expected only one command interface 'position'", jname);
                    return CallbackReturn::ERROR;
                }

                // States: position, velocity and effort ( only for gripper ), in that order
                const auto & states = joint.state_interfaces;
                const std::size_t expected_states = is_gripper ? 3 : 2;
                const bool states_ok = states.size() == expected_states && states[0].name == HW_IF_POSITION && states[1].name == HW_IF_VELOCITY && (!is_gripper || states[2].name == HW_IF_EFFORT);
                if (!states_ok){
                    RCLCPP_ERROR(get_logger(), "Joint '%s': expected state interfaces [position, velocity%s] in that order "
                    "(gripper must be the last_joint)",
                    jname, is_gripper ? ", effort" : "");
                    return CallbackReturn::ERROR;
                }

                // Servo ID: present, interger , in range, unique.
                long id = 0;
                if(!find_param(joint.parameters, "ID", value) || !parse_long(value, id) || id < 0 || id > kMaxServoId){
                    RCLCPP_ERROR(
                        get_logger(), "Joint '%s': missing or invalid 'ID' (must be 0..%ld)", jname, kMaxServoId);
                    return CallbackReturn::ERROR;
                }
                if (id_taken[static_cast<std::size_t>(id)]){
                    RCLCPP_ERROR(get_logger(), "Joint '%s': servo ID %ld is used by another joint", jname, id);
                    return CallbackReturn::ERROR;
                }
                id_taken[static_cast<std::size_t>(id)] = true;

                // Sign: exactly +1 or -1
                long sign = 0;
                if (!find_param(joint.parameters, "sign", value) || !parse_long(value, sign) || (sign != 1 && sign != -1)){
                    RCLCPP_ERROR(get_logger(), "Joint '%s': missing or invalid 'sign' (must be +1 or -1)", jname);
                    return CallbackReturn::ERROR;
                }

                //Offset: present, integer, same magnitude (counts).
                long offset = 0;
                if (!find_param(joint.parameters, "offset", value) || !parse_long(value, offset) || std::labs(offset) > kMaxAbsOffsetCounts){
                    RCLCPP_ERROR(get_logger(), "Joint '%s': missing or invalid 'offset' (|offset| <= %ld counts)", jname, kMaxAbsOffsetCounts);
                    return CallbackReturn::ERROR;
                }

                servo_ids_[i] = static_cast<std::uint8_t>(id);
                signs_[i] = static_cast<int>(sign);
                offsets_[i] = static_cast<std::int32_t>(offset);

                pos_state_names_[i] = joint.name + "/" + HW_IF_POSITION;
                vel_state_names_[i] = joint.name +  "/" + HW_IF_VELOCITY;
                pos_cmd_names_[i] = joint.name + "/" + HW_IF_POSITION;
            }

            gripper_effort_name_ = info_.joints[kGripperIndex].name + "/" + HW_IF_EFFORT;

            RCLCPP_INFO(get_logger(), "on_init OK: %zu joints and port %s @ %d baud", kNumJoints, serial_port_.c_str(), baud_rate_);

            return CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn SO101System::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
            return CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn SO101System::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
            return CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn SO101System::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
            return CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn SO101System::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
            return CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn SO101System::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) {
            return CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn SO101System::on_error(const rclcpp_lifecycle::State & /*previous_state*/) {
            return CallbackReturn::SUCCESS;
        }

        hardware_interface::return_type SO101System::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
            return hardware_interface::return_type::OK;
        }

        hardware_interface::return_type SO101System::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
            return hardware_interface::return_type::OK;
        }
}

PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101System, hardware_interface::SystemInterface)
