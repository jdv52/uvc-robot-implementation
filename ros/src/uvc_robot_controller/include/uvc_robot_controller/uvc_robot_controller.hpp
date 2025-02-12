#ifndef _UVC_CONTROLLER__UVC_CONTROLLER_HPP_
#define _UVC_CONTROLLER__UVC_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <chrono>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <uvc_robot_controller/uvc_robot_controller_params.hpp>
#include <uvc_robot_controller/uvc_controller_fsm.hpp>

namespace uvc_robot_controller
{
    class UVC_Controller : public controller_interface::ControllerInterface
    {
        public:
            UVC_Controller();

            controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

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

        private:
            std::vector<std::string> exported_reference_interface_names_;
            std::vector<std::string> exported_state_interface_names_;

            std::unique_ptr<UVC_Controller_FSM> fsm;

            std::shared_ptr<ParamListener> param_listener_;
            Params params_;
    };
}

#endif