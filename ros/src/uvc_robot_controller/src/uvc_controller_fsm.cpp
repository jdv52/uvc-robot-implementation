#include <uvc_robot_controller/uvc_controller_fsm.hpp>

namespace uvc_robot_controller {

UVC_Controller_FSM::UVC_Controller_FSM(
    rclcpp_lifecycle::LifecycleNode::SharedPtr n
) : node_ptr_(n)
{
    state = INITIALIZATION;
}

controller_interface::return_type UVC_Controller_FSM::update_FSM(
    std::vector<hardware_interface::LoanedCommandInterface> &cmd_ifs,
    std::vector<hardware_interface::LoanedStateInterface> &state_ifs
)
{
    float imuPitch = state_ifs[IMU_ORIENTATION_Y].get_value();
    float imuRoll = state_ifs[IMU_ORIENTATION_Y].get_value();

    switch (state)
    {
        case INITIALIZATION:
            RCLCPP_INFO(node_ptr_->get_logger(), "UVC Initialization..");

            state = ANGLE_CALIBRATION;
            break;
        case ANGLE_CALIBRATION:
            if ( fabs(imuPitch) >= 0.35 && fabs(imuRoll) >= 0.35)
            {
                state = FALL_DETECTED;
                RCLCPP_INFO(node_ptr_->get_logger(), "Fall Detected");
            } else {
                RCLCPP_INFO(node_ptr_->get_logger(), "No fall detected with IMU Pitch: %f, Yaw: %f.", imuPitch, imuRoll);
            }
            break;
            break;
        case TILT_CORRECTION:
            break;
        case IDLE:
            break;
        case WALKING_START:
            break;
        case WALKING_WAIT_FOR_VIBRATION_DAMPING:
            break;
        case WALKING_RECOVERY:
            break;
        case WALKING_POST_RECOVERY:
            break;
        case FALL_DETECTED:            
            cmd_ifs[RIGHT_SHOULDER].set_value(0.83);
            cmd_ifs[LEFT_SHOULDER].set_value(0.27);
            cmd_ifs[RIGHT_FOREARM].set_value(0.27);
            cmd_ifs[LEFT_FOREARM].set_value(0.27);
            cmd_ifs[RIGHT_LEG].set_value(1.24);
            cmd_ifs[LEFT_LEG].set_value(1.24);
            cmd_ifs[RIGHT_KNEE].set_value(1.44);
            cmd_ifs[LEFT_KNEE].set_value(1.44);
            cmd_ifs[RIGHT_ANKLE].set_value(1.9);
            cmd_ifs[LEFT_ANKLE].set_value(1.48);
            break;
        default:
            break;
    }

    return controller_interface::return_type::OK;
}

}
