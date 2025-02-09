#ifndef _UVC_CONTROLLER__UVC_CONTROLLER_HPP_
#define _UVC_CONTROLLER__UVC_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <chrono>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace uvc_controller
{
    class UVC_Controller : public controller_interface::ChainableControllerInterface
    {
        public:
            UVC_Controller();

            std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

            bool on_set_chained_mode(bool chained_mode) override;

            controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time &time, const rclcpp::Duration &period) override;

            controller_interface::return_type update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) override;

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn on_error(
                const rclcpp_lifecycle::State & previous_state) override;
    };
}

#endif