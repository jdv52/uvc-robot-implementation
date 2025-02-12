#include "uvc_robot_controller/uvc_robot_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace uvc_robot_controller {
    UVC_Controller::UVC_Controller(
    ) : controller_interface::ControllerInterface()
    {}

    controller_interface::return_type UVC_Controller::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        return fsm->update_FSM(
            command_interfaces_,
            state_interfaces_
        );
    }


    controller_interface::InterfaceConfiguration UVC_Controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration reference_interfaces_config;
        reference_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      
        reference_interfaces_config.names.reserve(exported_reference_interface_names_.size());
      
        for (std::size_t i = 0; i < 22; ++i)
        {
            reference_interfaces_config.names.push_back(exported_reference_interface_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
        }

        RCLCPP_INFO(get_node()->get_logger(), "Sucessfully configured command interfaces!");
        return reference_interfaces_config;
    }

    controller_interface::InterfaceConfiguration UVC_Controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      
        state_interfaces_config.names.reserve(exported_state_interface_names_.size());
      
        for (std::size_t i = 0; i < 22; ++i)
        {
            state_interfaces_config.names.push_back(exported_state_interface_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
        }

        for (std::size_t i = 22; i < exported_state_interface_names_.size(); ++i)
        {
            state_interfaces_config.names.push_back(params_.imu.sensor_name + "/" + exported_state_interface_names_[i]);
        }
      
        RCLCPP_INFO(get_node()->get_logger(), "Sucessfully configured state interfaces!");
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
        
        RCLCPP_INFO(get_node()->get_logger(), "Sucessfully initialized controller!");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UVC_Controller::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        params_ = param_listener_->get_params();

        auto prepare_lists_with_joint_names =
            [&command_joints = this->exported_reference_interface_names_, &state_joints = this->exported_state_interface_names_](
            const std::size_t index, const std::string & interface_joint_name)
        {
            state_joints[index] = interface_joint_name;
            command_joints[index] = interface_joint_name;
        };

        exported_state_interface_names_.resize(22 + 4 + 3 + 3);
        exported_reference_interface_names_.resize(22 + 4 + 3 + 3);
        // exported_state_interface_names_.resize(1);
        prepare_lists_with_joint_names(
            HEAD, params_.joint_names.head_joint_name
        );
        prepare_lists_with_joint_names(
            WAIST, params_.joint_names.waist_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_SHOULDER, params_.joint_names.left_shoulder_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_ARM, params_.joint_names.left_arm_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_ELBOW, params_.joint_names.left_elbow_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_FOREARM, params_.joint_names.left_forearm_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_SHOULDER, params_.joint_names.right_shoulder_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_ARM, params_.joint_names.right_arm_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_ELBOW, params_.joint_names.right_elbow_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_FOREARM, params_.joint_names.right_forearm_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_HIP1, params_.joint_names.left_hip1_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_HIP2, params_.joint_names.left_hip2_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_LEG, params_.joint_names.left_leg_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_KNEE, params_.joint_names.left_knee_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_ANKLE, params_.joint_names.left_ankle_joint_name
        );
        prepare_lists_with_joint_names(
            LEFT_FOOT, params_.joint_names.left_foot_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_HIP1, params_.joint_names.right_hip1_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_HIP2, params_.joint_names.right_hip2_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_LEG, params_.joint_names.right_leg_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_KNEE, params_.joint_names.right_knee_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_ANKLE, params_.joint_names.right_ankle_joint_name
        );
        prepare_lists_with_joint_names(
            RIGHT_FOOT, params_.joint_names.right_foot_joint_name
        );
        prepare_lists_with_joint_names(
            IMU_ORIENTATION_X, params_.imu.state_interface_names.orientation.x
        );
        prepare_lists_with_joint_names(
            IMU_ORIENTATION_Y, params_.imu.state_interface_names.orientation.y
        );
        prepare_lists_with_joint_names(
            IMU_ORIENTATION_Z, params_.imu.state_interface_names.orientation.z
        );
        prepare_lists_with_joint_names(
            IMU_ORIENTATION_W, params_.imu.state_interface_names.orientation.w
        );
        prepare_lists_with_joint_names(
            IMU_ANGULAR_VEL_X, params_.imu.state_interface_names.angular_velocity.x
        );
        prepare_lists_with_joint_names(
            IMU_ANGULAR_VEL_Y, params_.imu.state_interface_names.angular_velocity.y
        );
        prepare_lists_with_joint_names(
            IMU_ANGULAR_VEL_Z, params_.imu.state_interface_names.angular_velocity.z
        );
        prepare_lists_with_joint_names(
            IMU_LINEAR_ACC_X, params_.imu.state_interface_names.linear_acceleration.x
        );
        prepare_lists_with_joint_names(
            IMU_LINEAR_ACC_Y, params_.imu.state_interface_names.linear_acceleration.y
        );
        prepare_lists_with_joint_names(
            IMU_LINEAR_ACC_Z, params_.imu.state_interface_names.linear_acceleration.z
        );

        RCLCPP_INFO(get_node()->get_logger(), "Sucessfully configured controller!");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UVC_Controller::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        fsm = std::unique_ptr<UVC_Controller_FSM>(new UVC_Controller_FSM(get_node()));

        RCLCPP_INFO(get_node()->get_logger(), "Successfully activated!");
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
    uvc_robot_controller::UVC_Controller, controller_interface::ControllerInterface
)