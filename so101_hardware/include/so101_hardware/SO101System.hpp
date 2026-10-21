#ifndef SO101_HARDWARE__SO101SYSTEM_HPP_
#define SO101_HARDWARE__SO101SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <vector>
#include <string>



namespace so101_hardware {
    class SO101System : public hardware_interface::SystemInterface{
    public:
        CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;


        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write( const rclcpp::Time & time, const rclcpp::Duration & period) override;



    };
}

#endif
