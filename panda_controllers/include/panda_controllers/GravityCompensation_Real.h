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

class GravityCompensation : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

public:
    bool init(hardware_interface::EffortJointInterface *t, ros::NodeHandle &handle) override;

    void starting(const ros::Time &time1) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void stopping(const ros::Time &time1) override;

private:
    constexpr static size_t NUM_OF_JOINTS = 7;
    /** @brief Array of joint handlers */
    std::array<hardware_interface::JointHandle, NUM_OF_JOINTS> joints;
};

#endif //EXERCISE5_EXERCISE5CONTROLLER_H
