#include "uvc_robot_controller/uvc_controller.hpp"

namespace uvc_controller {
    UVC_Controller::UVC_Controller()
    {
    }
    
    std::vector<hardware_interface::StateInterface> UVC_Controller::on_export_state_interfaces()
    {
        return std::vector<hardware_interface::StateInterface>();
    }

    std::vector<hardware_interface::CommandInterface> UVC_Controller::on_export_reference_interfaces()
    {
        return std::vector<hardware_interface::CommandInterface>();
    }

    bool UVC_Controller::on_set_chained_mode(bool chained_mode)
    {
        return false;
    }

    controller_interface::return_type UVC_Controller::update_reference_from_subscribers(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        return controller_interface::return_type();
    }

    controller_interface::return_type UVC_Controller::update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        return controller_interface::return_type();
    }


    controller_interface::InterfaceConfiguration UVC_Controller::command_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration();
    }

    controller_interface::InterfaceConfiguration UVC_Controller::state_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration();
    }

    controller_interface::CallbackReturn UVC_Controller::on_init()
    {
        return controller_interface::CallbackReturn();
    }

    controller_interface::CallbackReturn UVC_Controller::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn();
    }

    controller_interface::CallbackReturn UVC_Controller::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn();
    }

    controller_interface::CallbackReturn UVC_Controller::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn();
    }

    controller_interface::CallbackReturn UVC_Controller::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn();
    }

    controller_interface::CallbackReturn UVC_Controller::on_error(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn();
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    uvc_controller::UVC_Controller, controller_interface::ChainableControllerInterface
)