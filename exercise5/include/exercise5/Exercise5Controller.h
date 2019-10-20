/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#ifndef EXERCISE5_EXERCISE5CONTROLLER_H
#define EXERCISE5_EXERCISE5CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>


class Exercise5Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

public:
    bool init(hardware_interface::EffortJointInterface *t, ros::NodeHandle &handle) override;

    void starting(const ros::Time &time1) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void stopping(const ros::Time &time1) override;
    void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    double first_order_lowpass_filter();
    /** @brief Get array of dh parameters for the panda robot. Order: theta, d, a, alpha */
    std::array<std::array<double, 4>, 7> getDHParameters() const;

    /** @brief Compute Jacobian analytically for the given robot configuration q and a reference frame r specified w.r.t. ee */
    Eigen::Matrix<double, 6, 7> computeJacobian(const Eigen::VectorXd &q, const Eigen::Vector3d &r = Eigen::Vector3d::Zero()) const;
    Eigen::MatrixXd calculateJacobian(Eigen::VectorXd &q_in);
    /** @brief Compute direct kinematics for the given joint angles q. Assume ee is offseted by vector r. */
    Eigen::Affine3d dkt(const Eigen::VectorXd &q, const Eigen::Vector3d &r = Eigen::Vector3d::Zero()) const;
    Eigen::Matrix4d forwardKinematic(Eigen::VectorXd &q, int start, int end);
    /**
    * @brief Return affine transformation from DH parameters
    * @param theta rotation around z axis
    * @param d translation in z axis
    * @param a translation in x axis
    * @param alpha rotation around x axis
    * @return Rz * Tz * Tx * Rx  */
    static Eigen::Affine3d DH(double theta, double d, double a, double alpha);

private:
    constexpr static size_t NUM_OF_JOINTS = 7;

    /** @brief Array of joint handlers */
    std::array<hardware_interface::JointHandle, NUM_OF_JOINTS> joints;
    Eigen:: Matrix<double,6,1> err_force_int; // error in ft
    // Eigen::Matrix<double,6,1> f_current;
    Eigen::VectorXd initial_pose;
    ros::Subscriber sub_forcetorque_sensor_;

};

#endif //EXERCISE5_EXERCISE5CONTROLLER_H
