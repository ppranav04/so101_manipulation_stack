#include "so101_hardware/SO101System.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace so101_hardware {
        hardware_interface::CallbackReturn SO101System::on_init(const hardware_interface::HardwareComponentInterfaceParams & params){
            if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS){
                return CallbackReturn::ERROR;
            }

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
