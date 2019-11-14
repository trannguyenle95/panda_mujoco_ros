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
// from kdl packages
 #include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
 #include <boost/scoped_ptr.hpp>
 #include <boost/lexical_cast.hpp>

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

private:
    constexpr static size_t NUM_OF_JOINTS = 7;
    ros::Subscriber sub_forcetorque_sensor_sim;
    ros::Subscriber sub_forcetorque_sensor_real;

    double sim;
    /** @brief Array of joint handlers */
    std::array<hardware_interface::JointHandle, NUM_OF_JOINTS> joints;
    Eigen:: Matrix<double,6,1> err_force_int; // error in ft
    // Variable for low pass filter
    double tau_ = 1.0/(2*3.14*9.0);
    double filt_old_ = 0.0;
    double filt_ = 0.0;
    double delta_time = 0.002;
    double f_cur_buffer_ = 0.0;

    //KDL Tree and KDL chain
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    // Current joint posistion and cartesian pos
    KDL::JntArray q_kdl, initial_pose;
    KDL::Frame cart_pos_current,cart_pos_desired;
    KDL::Twist cart_err;
    struct quaternion{
      KDL::Vector v;
      double a;
    } quat_current, quat_desired;
    Eigen::Matrix<double,3,3> skew;
    KDL::Vector v_temp;
    KDL::Vector pos_current, pos_desired;


    // kdl M,C,G
    KDL::Vector gravity;
    KDL::JntSpaceInertiaMatrix M; // intertia matrix
    KDL::JntArray C;              // coriolis
    KDL::JntArray G;              // gravity torque vector
    KDL::JntArray comp_d;

    // kdl and Eigen Jacobian
    KDL::Jacobian J;

    //Solver to compute the jacobian
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    //Solver to compute the forward kinematics (position)
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
    //Solver to compute the inverse dynamics
    boost::scoped_ptr<KDL::ChainDynParam> id_solver;

};

#endif //EXERCISE5_EXERCISE5CONTROLLER_H
