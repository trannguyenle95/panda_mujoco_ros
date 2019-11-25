/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 25/10/19
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi>
 */
 #include <panda_controllers/ForceController_Sim.h>
 #include <pluginlib/class_list_macros.h>
 #include <ros/ros.h>
 #include <ros/package.h>
 #include <geometry_msgs/WrenchStamped.h>
 #include <std_msgs/Float64.h>

 #include <urdf/model.h>
 // from kdl packages
 #include <kdl/tree.hpp>
 #include <kdl/kdl.hpp>
 #include <kdl/chain.hpp>
 #include <kdl_parser/kdl_parser.hpp>
 #include <kdl/chaindynparam.hpp>              // inverse dynamics
 #include <kdl/chainjnttojacsolver.hpp>        // jacobian
 #include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
 // #include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics
 #include <boost/scoped_ptr.hpp>
 #include <boost/lexical_cast.hpp>
 #include <math.h>
 #include <Eigen/LU>
 #include <utils/pseudo_inversion.h>
 #include <utils/skew_symmetric.h>

Eigen::Matrix<double,6,1> f_current;
KDL::JntArray initial_pose_save(7);

bool ForceController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &handle) {
    const auto names = hw->getNames();
    // subscribers for simulation and real robot cases.
    sub_forcetorque_sensor_sim = handle.subscribe<geometry_msgs::WrenchStamped>("/lumi_mujoco/F_ext", 1, &ForceController::updateFTsensor, this,ros::TransportHints().reliable().tcpNoDelay());
    sub_forcetorque_sensor_real = handle.subscribe<geometry_msgs::WrenchStamped>("/franka_state_controller/F_ext", 1, &ForceController::updateFTsensor, this,ros::TransportHints().reliable().tcpNoDelay());
    force_pub = handle.advertise<std_msgs::Float64>("force_data", 1000);

    //Pass the sim parameter. If sim = 1, we are using simulation. If sim = 0, we are using the real robot
    if (!handle.getParam("/lumi_mujoco/force_controller/sim", sim)){
        ROS_ERROR("Could not find sim parameter");
        return false;
    }
    std::string default_name = "";
    auto jname = default_name;
    for (size_t i = 0; i < joints.size(); ++i) {
        if (sim == 1){jname = std::string("lumi_joint") + std::to_string(i + 1);}
        if (sim == 0){jname = std::string("panda_joint") + std::to_string(i + 1);}
        if (std::find(names.begin(), names.end(), jname) == names.end()) {
            ROS_ERROR_STREAM("Joint not found: " << jname);
            ROS_ERROR_STREAM("Available joints: ");
            for (const auto &name : names) {
                ROS_ERROR_STREAM(name);
            }
            return false;
        }
        joints[i] = hw->getHandle(jname);
    }
    // ** Parse urdf file to kdl_tree **
    std::string urdf_file;
    if (sim == 1){urdf_file = "/home/trannguyenle/catkin_ws/src/panda_controllers/model/robots/sim.urdf";}
    if (sim == 0){urdf_file = "/home/tran/catkin_ws/src/panda_controllers/model/robots/sim.urdf";}
    urdf::Model model;
    if (!model.initFile(urdf_file)){
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }
    ROS_INFO("Successfully parsed urdf file");
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)){
       ROS_ERROR("Failed to construct kdl tree");
       return false;
    }
    ROS_INFO("Successfully construct kdl tree");
    // ====================================
    // ** Construct kdl_chain from kdl_tree **
    kdl_tree.getChain("base_link", "lumi_ee", kdl_chain);
    std::cout << "Number of joints in kdl chain: " << kdl_chain.getNrOfJoints() << std::endl;
    // ====================================

    fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain)); //forward kinematics solver init
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain)); //jacobian solver init
    gravity = KDL::Vector::Zero(); // Get the gravity vector (direction and magnitude)
    gravity(2) = -9.81; // 0: x-axis 1: y-axis 2: z-axis
    id_solver.reset(new KDL::ChainDynParam(kdl_chain, gravity));
    // Dynamics parameters M (mass matrix), C (coriolis), G (gravity)
    J.resize(kdl_chain.getNrOfJoints()); //jacobian matrix
    M.resize(kdl_chain.getNrOfJoints());
    C.resize(kdl_chain.getNrOfJoints());
    G.resize(kdl_chain.getNrOfJoints());
    return Controller::init(hw, handle);
}

void ForceController::starting(const ros::Time &time1) {
    err_force_int.setZero(); //reset integration term
    f_current.setZero();     //reset the force feedback
    // Get the initial pose of the robot
    for (int i = 0; i < 7; ++i){
        initial_pose_save(i) =  joints[i].getPosition();
    }
    ControllerBase::starting(time1);
}
// ** This function is used to obtain F/T sensor data.
void ForceController::updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  geometry_msgs::Wrench f_meas = msg->wrench;
	f_cur_buffer_ = f_meas.force.x;
  f_current(1) = f_meas.force.y;
  f_current(2) = f_meas.force.z;
  f_current(3) = f_meas.torque.x;
  f_current(4) = f_meas.torque.y;
  f_current(5) = f_meas.torque.z;
  f_current(0) = first_order_lowpass_filter();
}

void ForceController::update(const ros::Time &time, const ros::Duration &period) {
  //// get join position and velocity of current time step
   KDL::JntArray q_kdl(7), q_vel(7);
   for (int i = 0; i < 7; ++i){
       q_kdl(i) =  joints[i].getPosition();
       q_vel(i) = joints[i].getVelocity();
   }
   KDL::JntArray initial_pose(7);
   initial_pose = initial_pose_save;
   // *** 3.2 Compute model(M,C,G) ***
   id_solver->JntToMass(q_kdl, M);
   id_solver->JntToCoriolis(q_kdl, q_vel, C);
   id_solver->JntToGravity(q_kdl, G);
   //// calculate jacobian
   jnt_to_jac_solver->JntToJac(q_kdl, J);

   // forward kinematics
   fk_pos_solver->JntToCart(q_kdl, cart_pos_current);
   pos_current = cart_pos_current.p;
   cart_pos_current.M.GetQuaternion(quat_current.v(0),quat_current.v(1),quat_current.v(2),quat_current.a);

   fk_pos_solver->JntToCart(initial_pose, cart_pos_desired);
   pos_desired = cart_pos_desired.p;
   cart_pos_desired.M.GetQuaternion(quat_desired.v(0),quat_desired.v(1),quat_desired.v(2),quat_desired.a);
   skew_symmetric(quat_desired.v, skew);
   for (int i = 0; i < skew.rows(); i++)
   {
       v_temp(i) = 0.0;
       for (int k = 0; k < skew.cols(); k++)
           v_temp(i) += skew(i,k)*(quat_current.v(k));
   }
   //// defining and initializing PD controller variables
   // motion control
   Eigen::Matrix<double, 3,3> kp_pos,kp_ori;
   Eigen::Matrix<double, 6,6> kp_force,ki_force;
   kp_pos.setIdentity();
   kp_ori.setIdentity();
   kp_force.setIdentity();
   ki_force.setIdentity();
   if (sim == 1){
     kp_pos(0,0)= 300;
     kp_pos(1,1)= 2000;
     kp_pos(2,2)= 2000; //works
     kp_ori *= 1000;
     // PI forcefeedback gains
     kp_force *= 0.005;
     ki_force *= 0.009;
   }
   if (sim == 0){
     kp_pos(0,0)= 0; // 10 each
     kp_pos(1,1)= 0;
     kp_pos(2,2)= 25; //25
     kp_ori *= 0;
     kp_force *= 0.005;
     ki_force *= 0.008;
   }
   //// set the desired force and calculate the force error.
   Eigen:: Matrix<double,6,1> f_desired;
   Eigen:: Matrix<double,6,1> err_force;
   f_desired << 12,0,0,0,0,0; //force desired in z axis
   err_force = f_desired - f_current;
   err_force_int += err_force*period.toSec(); //integral term of force error

  //// calculate new pos in global frame
   Eigen::VectorXd force_ctrl(6);
   force_ctrl = kp_force*err_force + ki_force*err_force_int;
   pos_desired[0] += force_ctrl[0];

   //// motion control
   Eigen::VectorXd err_pos(3); err_pos.setZero(); // error in position
   Eigen::Quaterniond err_quat; // error in orientation (quaternion)
   Eigen::VectorXd err_ori(3); // error in orientation
   cart_err.vel = pos_desired - pos_current;
   cart_err.rot = quat_current.a*quat_desired.v - quat_desired.a*quat_current.v - v_temp;
   err_pos(0) = cart_err.vel(0); err_pos(1) = cart_err.vel(1); err_pos(2) = cart_err.vel(2);
   err_ori(0) = cart_err.rot(0); err_ori(1) = cart_err.rot(1); err_ori(2) = cart_err.rot(2);

   Eigen::VectorXd Fp(3); // position part of the controller in task space
   Eigen::VectorXd Fr(3); // orientation part of the controller in task space
   Fp = kp_pos*err_pos ;
   Fr = kp_ori*err_ori ;
   Eigen::VectorXd F_ts_ctrl(6);
   F_ts_ctrl << Fp, Fr;

   Eigen::VectorXd tau(7);
   comp_d.data = G.data + C.data; //gravity and coriolis compensation
   if (sim == 1){
     tau =  J.data.transpose() *F_ts_ctrl + comp_d.data; //tau to command the robot
     std::cout << "f_current: " << f_current[0] << " --- "<< "err_force: " << err_force[0] << std::endl;
   }
   if (sim == 0){
     tau =  J.data.transpose() *F_ts_ctrl; //in real robot, the gravity is compensated by the robot
   }
   Eigen::VectorXd torque_limits(7);
   torque_limits << 87, 87, 87, 87, 12, 12, 12; // torque limits for joints
   for (int i = 0; i < 7; ++i){
       // check torque limitations
       if (tau[i] > torque_limits[i])
           tau[i] = torque_limits[i];
       else if (tau[i] < -torque_limits[i])
           tau[i] = -torque_limits[i];

   }
   for (size_t i = 0; i < joints.size(); ++i) {
      joints[i].setCommand(tau[i]); //send torque command to control the robot
    }
    std_msgs::Float64 msg;
    msg.data = f_current(0);
    ROS_INFO_STREAM("PUBLISHING FORCE DATA");
    force_pub.publish(msg);
}
void ForceController::stopping(const ros::Time &time1) {
    ControllerBase::stopping(time1);
}
// Simple first order low pass filter for the F/T sensor readings.
double ForceController::first_order_lowpass_filter()
{
    filt_ = (tau_ * filt_old_ + delta_time*f_cur_buffer_)/(tau_ + delta_time);
    filt_old_ = filt_;
    return filt_;
}
PLUGINLIB_EXPORT_CLASS(ForceController, controller_interface::ControllerBase)
