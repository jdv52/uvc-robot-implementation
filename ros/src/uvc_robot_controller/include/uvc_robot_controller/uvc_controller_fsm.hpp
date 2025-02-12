#ifndef _UVC_CONTROLLER__UVC_CONTROLLER_FSM_HPP_
#define _UVC_CONTROLLER__UVC_CONTROLLER_FSM_HPP_

#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "controller_interface/controller_interface.hpp"

namespace uvc_robot_controller
{
    enum UVC_State {
        INITIALIZATION = 0,
        ANGLE_CALIBRATION = 1,
        TILT_CORRECTION = 2,
        IDLE = 3,
        WALKING_START = 4,
        WALKING_WAIT_FOR_VIBRATION_DAMPING = 5,
        WALKING_RECOVERY = 6,
        WALKING_POST_RECOVERY = 7,
        FALL_DETECTED = 8,
    };

    enum UVC_JointIndex : std::size_t {
        HEAD = 0,
        WAIST,
        LEFT_SHOULDER,
        LEFT_ARM,
        LEFT_ELBOW,
        LEFT_FOREARM,
        RIGHT_SHOULDER,
        RIGHT_ARM,
        RIGHT_ELBOW,
        RIGHT_FOREARM,
        LEFT_HIP1,
        LEFT_HIP2,
        LEFT_LEG,
        LEFT_KNEE,
        LEFT_ANKLE,
        LEFT_FOOT,
        RIGHT_HIP1,
        RIGHT_HIP2,
        RIGHT_LEG,
        RIGHT_KNEE,
        RIGHT_ANKLE,
        RIGHT_FOOT,
        IMU_ORIENTATION_X,
        IMU_ORIENTATION_Y,
        IMU_ORIENTATION_Z,
        IMU_ORIENTATION_W,
        IMU_ANGULAR_VEL_X,
        IMU_ANGULAR_VEL_Y,
        IMU_ANGULAR_VEL_Z,
        IMU_LINEAR_ACC_X,
        IMU_LINEAR_ACC_Y,
        IMU_LINEAR_ACC_Z
    };


    class UVC_Controller_FSM
    {
        public:
            UVC_Controller_FSM(
                rclcpp_lifecycle::LifecycleNode::SharedPtr n
            );

            controller_interface::return_type update_FSM(
                std::vector<hardware_interface::LoanedCommandInterface> &cmd_ifs,
                std::vector<hardware_interface::LoanedStateInterface> &state_ifs
            );

        private:

            rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr_;

            UVC_State state;
    };


}

#endif