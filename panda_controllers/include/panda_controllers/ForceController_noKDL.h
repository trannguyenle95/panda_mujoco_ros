/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 * Adopted and modified on: 15/10/19
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi>
 */

#ifndef EXERCISE5_EXERCISE5CONTROLLER_H
#define EXERCISE5_EXERCISE5CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/WrenchStamped.h>


class ForceController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

public:
    bool init(hardware_interface::EffortJointInterface *t, ros::NodeHandle &handle) override;

    void starting(const ros::Time &time1) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void stopping(const ros::Time &time1) override;
    /** @brief Obtain the FT sensor feedback from mujoco*/
    void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    /** @brief A low pass filter to smooth the force feedback signal */
    double first_order_lowpass_filter();
    /** @brief Compute Jacobian analytically for the given robot configuration q and a reference frame r specified w.r.t. ee */
    Eigen::MatrixXd calculateJacobian(Eigen::VectorXd &q_in);
    /** @brief Compute direct kinematics for the given joint angles q. Assume ee is offseted by vector r. */
    Eigen::Matrix4d forwardKinematic(Eigen::VectorXd &q, int start, int end);


private:
    constexpr static size_t NUM_OF_JOINTS = 7;
    ros::Subscriber sub_forcetorque_sensor_;
    /** @brief Array of joint handlers */
    std::array<hardware_interface::JointHandle, NUM_OF_JOINTS> joints;
    Eigen:: Matrix<double,6,1> err_force_int; // error in ft
    Eigen::VectorXd initial_pose;
    // Variable for low pass filter
    double tau_ = 1.0/(2*3.14*9.0);
    double filt_old_ = 0.0;
    double filt_ = 0.0;
    double delta_time = 0.002;
    double f_cur_buffer_ = 0.0;

};

#endif //EXERCISE5_EXERCISE5CONTROLLER_H
