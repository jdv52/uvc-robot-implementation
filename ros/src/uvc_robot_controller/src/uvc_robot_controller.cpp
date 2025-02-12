#include "uvc_robot_controller/uvc_robot_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace uvc_robot_controller {
    UVC_Controller::UVC_Controller() : controller_interface::ChainableControllerInterface() {}

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
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::InterfaceConfiguration UVC_Controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      
        state_interfaces_config.names.reserve(exported_state_interface_names_.size());
      
        for (int i = 0; i < 22; ++i)
        {
            state_interfaces_config.names.push_back(exported_state_interface_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
        }

        for (int i = 22; i < exported_state_interface_names_.size(); ++i)
        {
            state_interfaces_config.names.push_back(params_.imu.sensor_name + "/" + exported_state_interface_names_[i]);
        }
      
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn UVC_Controller::on_init()
    {
        try
        {
            param_listener_ = std::make_shared<ParamListener>(get_node());
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
            return controller_interface::CallbackReturn::ERROR;
        }
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UVC_Controller::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        params_ = param_listener_->get_params();

        auto prepare_lists_with_joint_names =
            [&state_joints = this->exported_state_interface_names_](
            const std::size_t index, const std::string & state_joint_name)
        {
            state_joints[index] = state_joint_name;
        };

        exported_state_interface_names_.resize(22 + 4 + 3 + 3);
        // exported_state_interface_names_.resize(1);
        prepare_lists_with_joint_names(
            0, params_.joint_names.head_joint_name
        );
        prepare_lists_with_joint_names(
            1, params_.joint_names.waist_joint_name
        );
        prepare_lists_with_joint_names(
            2, params_.joint_names.left_shoulder_joint_name
        );
        prepare_lists_with_joint_names(
            3, params_.joint_names.left_arm_joint_name
        );
        prepare_lists_with_joint_names(
            4, params_.joint_names.left_elbow_joint_name
        );
        prepare_lists_with_joint_names(
            5, params_.joint_names.left_forearm_joint_name
        );
        prepare_lists_with_joint_names(
            6, params_.joint_names.right_shoulder_joint_name
        );
        prepare_lists_with_joint_names(
            7, params_.joint_names.right_arm_joint_name
        );
        prepare_lists_with_joint_names(
            8, params_.joint_names.right_elbow_joint_name
        );
        prepare_lists_with_joint_names(
            9, params_.joint_names.right_forearm_joint_name
        );
        prepare_lists_with_joint_names(
            10, params_.joint_names.left_hip1_joint_name
        );
        prepare_lists_with_joint_names(
            11, params_.joint_names.left_hip2_joint_name
        );
        prepare_lists_with_joint_names(
            12, params_.joint_names.left_leg_joint_name
        );
        prepare_lists_with_joint_names(
            13, params_.joint_names.left_knee_joint_name
        );
        prepare_lists_with_joint_names(
            14, params_.joint_names.left_ankle_joint_name
        );
        prepare_lists_with_joint_names(
            15, params_.joint_names.left_foot_joint_name
        );
        prepare_lists_with_joint_names(
            16, params_.joint_names.right_hip1_joint_name
        );
        prepare_lists_with_joint_names(
            17, params_.joint_names.right_hip2_joint_name
        );
        prepare_lists_with_joint_names(
            18, params_.joint_names.right_leg_joint_name
        );
        prepare_lists_with_joint_names(
            19, params_.joint_names.right_knee_joint_name
        );
        prepare_lists_with_joint_names(
            20, params_.joint_names.right_ankle_joint_name
        );
        prepare_lists_with_joint_names(
            21, params_.joint_names.right_foot_joint_name
        );
        prepare_lists_with_joint_names(
            22, params_.imu.state_interface_names.orientation.x
        );
        prepare_lists_with_joint_names(
            23, params_.imu.state_interface_names.orientation.y
        );
        prepare_lists_with_joint_names(
            24, params_.imu.state_interface_names.orientation.z
        );
        prepare_lists_with_joint_names(
            25, params_.imu.state_interface_names.orientation.w
        );
        prepare_lists_with_joint_names(
            26, params_.imu.state_interface_names.angular_velocity.x
        );
        prepare_lists_with_joint_names(
            27, params_.imu.state_interface_names.angular_velocity.y
        );
        prepare_lists_with_joint_names(
            28, params_.imu.state_interface_names.angular_velocity.z
        );
        prepare_lists_with_joint_names(
            29, params_.imu.state_interface_names.linear_acceleration.x
        );
        prepare_lists_with_joint_names(
            30, params_.imu.state_interface_names.linear_acceleration.y
        );
        prepare_lists_with_joint_names(
            31, params_.imu.state_interface_names.linear_acceleration.z
        );

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UVC_Controller::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UVC_Controller::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UVC_Controller::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UVC_Controller::on_error(const rclcpp_lifecycle::State &previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    uvc_robot_controller::UVC_Controller, controller_interface::ChainableControllerInterface
)