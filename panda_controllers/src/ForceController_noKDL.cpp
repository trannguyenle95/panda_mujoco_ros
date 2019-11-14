/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 15/10/19
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi>
 */

#include <exercise5/ForceController_noKDL.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/WrenchStamped.h>
Eigen::Matrix<double,6,1> f_current;
Eigen::VectorXd initial_pose_save(7);

bool ForceController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &handle) {
    const auto names = hw->getNames();
    sub_forcetorque_sensor_ = handle.subscribe<geometry_msgs::WrenchStamped>("/lumi_mujoco/array", 1, &ForceController::updateFTsensor, this,ros::TransportHints().reliable().tcpNoDelay());
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto jname = std::string("lumi_joint") + std::to_string(i + 1);
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

    return Controller::init(hw, handle);
}

void ForceController::starting(const ros::Time &time1) {
    err_force_int.setZero(); //reset integration term
    f_current.setZero();     //reset the force feedback
    for (int i = 0; i < 7; ++i){
        initial_pose_save(i) =  joints[i].getPosition();
    }
    ControllerBase::starting(time1);
}
void ForceController::updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  geometry_msgs::Wrench f_meas = msg->wrench;
	f_current(0) = f_meas.force.x;
  f_current(1) = f_meas.force.y;
  f_cur_buffer_ = f_meas.force.z;
  f_current(3) = f_meas.torque.x;
  f_current(4) = f_meas.torque.y;
  f_current(5) = f_meas.torque.z;
  f_current(2) = first_order_lowpass_filter();
}

void ForceController::update(const ros::Time &time, const ros::Duration &period) {
  // initial_pose << 0, 0.688, 0, -1.6, 0, 2.25, 0.75;
  // initial_pose << 0, 0.11, 0, -2.4, 0, 2.54, 0.84;
  Eigen::VectorXd initial_pose(7);
  initial_pose = initial_pose_save;
  //// get q of current time step
   Eigen::VectorXd q_current(7);
   for (int i = 0; i < 7; ++i){
       q_current(i) =  joints[i].getPosition();
   }
   //// calculate jacobian
   Eigen::MatrixXd J = calculateJacobian(q_current);

      //// forward kinematics
   Eigen::Matrix4d T09; // transformation matrix of frame 7 relative to frame 0 (world frame)
   T09 = forwardKinematic(q_current, 0, 9);
   Eigen::VectorXd pos_current(3); // current position of end effector
   pos_current << T09(0, 3), T09(1, 3), T09(2, 3);
   Eigen::Matrix3d r_current; // current orientation of end effector
   r_current = T09.block(0, 0, 3, 3); // extract rot matrix from T07
   Eigen::Quaterniond quat_current; // quaternion of end effector orientation
   quat_current = r_current; //R2Q

   Eigen::Matrix4d T09_desired; // transformation matrix of frame 7 relative to frame 0 (world frame)
   T09_desired = forwardKinematic(initial_pose, 0, 9);
   Eigen::VectorXd pos_desired(3); // current position of end effector
   pos_desired << T09_desired(0, 3), T09_desired(1, 3), T09_desired(2, 3);
   Eigen::Matrix3d r_desired; // current orientation of end effector
   r_desired = T09_desired.block(0, 0, 3, 3); // extract rot matrix from T07
   Eigen::Quaterniond quat_desired; // quaternion of end effector orientation
   quat_desired = r_desired; //R2Q

   //// defining and initializing PD controller variables
   // motion control
   Eigen::Matrix<double, 3,3> kp_pos,kp_ori,kd_pos,kd_ori;
   kp_pos.setIdentity();
   kd_pos.setIdentity();
   kp_ori.setIdentity();
   kd_ori.setIdentity();

   kp_pos(0,0)= 2000;
   kp_pos(1,1)= 2000;
   kp_pos(2,2)= 300; //works

   kp_ori *= 1000;

   // PI forcefeedback gains
   Eigen::Matrix<double, 6,6> kp_force,ki_force;
   kp_force.setIdentity();
   ki_force.setIdentity();
   kp_force *= 0.005;
   ki_force *= 0.004;

   //// set the desired forece and calculate the force error.
   Eigen:: Matrix<double,6,1> f_desired;
   Eigen:: Matrix<double,6,1> err_force;
   f_desired << 0,0,-20,0,0,0; //force desired in z axis
   err_force = f_desired - f_current;
   err_force_int += err_force*period.toSec(); //integral term of force error

  //// calculate new pos in global frame
   Eigen::VectorXd force_ctrl(6);
   force_ctrl = kp_force*err_force + ki_force*err_force_int;
   pos_desired[2] += force_ctrl[2];
   std::cout << "f_current: " << f_current[2] << std::endl;
   std::cout << "err_force: " << err_force[2] << std::endl;

   //// motion control
   Eigen::VectorXd err_pos(3); err_pos.setZero(); // error in position
   Eigen::Quaterniond err_quat; // error in orientation (quaternion)
   Eigen::VectorXd err_ori(3); // error in orientation

   err_pos = pos_desired - pos_current;
   err_quat = quat_desired * quat_current.inverse();
   err_ori[0] = err_quat.x(); err_ori[1] = err_quat.y(); err_ori[2] = err_quat.z();
   std::cout << "err_pos: " << err_pos.transpose() << std::endl;
   std::cout << "err_ori: " << err_ori.transpose() << std::endl;

   Eigen::VectorXd Fp(3); // position part of the controller in task space
   Eigen::VectorXd Fr(3); // orientation part of the controller in task space
   Fp = kp_pos*err_pos ;
   Fr = kp_ori*err_ori ;

   //// tau = Jp^T * Fp + Jr^T * Fr + J^T * foce_controller_term + qfrc_bias
   Eigen::VectorXd F_ts_ctrl(6);
   F_ts_ctrl << Fp, Fr;

   Eigen::VectorXd tau(7);
   tau =  J.transpose() *F_ts_ctrl; //gravity is added due to the code in RobotHWMujoco
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
   std::cout << " -------- \n";
}


void ForceController::stopping(const ros::Time &time1) {
    ControllerBase::stopping(time1);
}

Eigen::Matrix4d ForceController::forwardKinematic(Eigen::VectorXd &q, int start, int end)
{
    /*
     * This function calculate forward kinematics for panda.
     * inputs:
     *  - q: joint angles
     *  - start: start frame
     *  - end: end frame
     *  for example for calculation of transformation matrix of frame 5 relative to 2 -> start: 2, end: 5
     *
     * output:
     *  - transformation matrix
     */

    // DH parameters - 0:world - 1to7:joints - 8:ft sensor (flange) - 9: end effector
    Eigen::VectorXd a(10);
    Eigen::VectorXd d(10);
    Eigen::VectorXd alfa(10);
    Eigen::VectorXd theta(10);

    a     << 0  , 0       ,  0      , 0.0825 , -0.0825  , 0      ,  0.088, 0   , 0       , 0      ;
    d     << 0  , 0.333   , 0       ,  0.316 , 0        , 0.384  , 0     , 0   , 0.107   , 0.112; //0.1034 ;
    alfa  << 0  , -M_PI/2 , M_PI/2  , M_PI/2 , -M_PI/2  , M_PI/2 , M_PI/2, 0   , 0       , 0      ;
    theta << 0  , q(0)    , q(1)    ,  q(2)  , q(3)     , q(4)   , q(5)  , q(6), -M_PI/4 , 0      ;

    Eigen::Matrix4d Ttemp = Eigen::Matrix4d::Identity();  // initialize Ttemp
    for (int i = start; i < end; i++)
    {
        Eigen::Matrix4d T;

        // modified dh
        T <<    cos(theta(i+1))              , -sin(theta(i+1))                 ,            0  , a(i),
                sin(theta(i+1))*cos(alfa(i)) , cos(theta(i+1)) * cos(alfa(i))   , -sin(alfa(i)) , -d(i+1) * sin(alfa(i)),
                sin(theta(i+1))*sin(alfa(i)) , cos(theta(i+1)) * sin(alfa(i))   , cos(alfa(i))  , d(i+1)*cos(alfa(i)),
                0                                 ,         0                              ,         0     , 1 ;

        Ttemp = Ttemp * T;
    }
    return Ttemp;
}
Eigen::MatrixXd ForceController::calculateJacobian(Eigen::VectorXd &q_in)
{
    /*
     * This function is provided by Dr. Fares Abu-Dakka
     * This function calculates Jtilde for Franka robot.
     * inputs:
     *  - q_in: joint angles
     * output:
     *  - camculated Jtilde Matrix
     */

    double q1 = q_in[0]; double q2 = q_in[1]; double q3 = q_in[2]; double q4 = q_in[3];
    double q5 = q_in[4]; double q6 = q_in[5]; double q7 = q_in[6];

    Eigen::MatrixXd Jtilde = Eigen::MatrixXd::Zero(6,7);
    double c1=cos(q1); double s1=sin(q1);
    double c2=cos(q2); double s2=sin(q2);
    double c3=cos(q3); double s3=sin(q3);
    double c4=cos(q4); double s4=sin(q4);
    double c5=cos(q5); double s5=sin(q5);
    double c6=cos(q6); double s6=sin(q6);
    double c7=cos(q7); double s7=sin(q7);

    double t7 = c1*s3;
    double t10 = s1*c2*c3;
    double t12 = s4*(t7+t10);
    double t35 = s1*s2*c4;
    double t18 = (t7+t10)*c4;
    double t21 = c5*(t18+s1*s2*s4);
    double t25 = s5*(c1*c3-s3*s1*c2);
    double t29 = c5*(s4*c2-s2*c3*c4);
    double t30 = s3*s2*s5;
    double t33 = s2*s4*c3;
    double t36 = (t12-t35)*s6*(11.0/125.0);
    double t38 = (t7+t10)*c4*(33.0/400.0);
    double t39 = s4*(t7+t10)*(48.0/125.0);
    double t40 = s1*s2*s4*(33.0/400.0);
    double t41 = c2*(79.0/250.0);
    double t42 = c2*c4*(48.0/125.0);
    double t44 = c6*(t29+t30)*(11.0/125.0);
    double t45 = s6*(t29+t30)*(107.0/1000.0);
    double t46 = s6*(c2*c4+t33)*(11.0/125.0);
    double t47 = s2*c3*c4*(33.0/400.0);
    double t48 = s2*s4*c3*(48.0/125.0);
    double t50 = s4*c2*(33.0/400.0);
    double t51 = c6*(c2*c4+t33)*(107.0/1000.0);
    double t70 = s2*c3*(33.0/400.0);
    double t49 = t41+t42+t44+t45+t46+t47+t48-t50-t51-t70;
    double t52 = (t12-t35)*c6*(107.0/1000.0);
    double t53 = c6*(t21+t25)*(11.0/125.0);
    double t54 = s6*(t21+t25)*(107.0/1000.0);
    double t55 = s1*s2*c4*(48.0/125.0);
    double t56 = s3*s1;
    double t61 = c1*c2*c3;
    double t58 = s4*(t56-t61);
    double t59 = c1*s2*c4;
    double t62 = c4*(t56-t61);
    double t73 = c1*s2*s4;
    double t64 = c5*(t62-t73);
    double t65 = s1*c3;
    double t66 = c1*s3*c2;
    double t68 = s5*(t65+t66);
    double t71 = c1*s2*(79.0/250.0);
    double t72 = s6*(t58+t59)*(11.0/125.0);
    double t74 = c4*(t56-t61)*(33.0/400.0);
    double t75 = s4*(t56-t61)*(48.0/125.0);
    double t76 = c1*c2*c3*(33.0/400.0);
    double t77 = c1*s2*c4*(48.0/125.0);
    double t80 = c6*(t58+t59)*(107.0/1000.0);
    double t81 = c6*(t64+t68)*(11.0/125.0);
    double t82 = s6*(t64+t68)*(107.0/1000.0);
    double t83 = c1*s2*s4*(33.0/400.0);
    double t91 = s3*s1*(33.0/400.0);
    double t78 = t71+t72+t74+t75+t76+t77-t80-t81-t82-t83-t91;
    double t79 = t42+t44+t45+t46+t47+t48-t50-t51;
    double t84 = t42+t44+t45+t46+t48-t51;
    double t85 = t44+t45+t46-t51;
    double t86 = s5*(s4*c2-s2*c3*c4);
    double t113 = s3*s2*c5;
    double t89 = s6*(t29+t30);
    double t114 = c6*(c2*c4+t33);
    double t92 = c1*s3*(33.0/400.0);
    double t93 = s1*s2*(79.0/250.0);
    double t94 = s1*c2*c3*(33.0/400.0);
    double t95 = -t36-t38-t39-t40+t52+t53+t54+t55+t92+t93+t94;
    double t96 = t36+t38+t39+t40-t52-t53-t54-t55;
    double t97 = -t36-t39+t52+t53+t54+t55;
    double t98 = c5*(c1*c3-s3*s1*c2);
    double t99 = -t72+t80+t81+t82;
    double t112 = (t18+s1*s2*s4)*s5;
    double t101 = c5*(t65+t66);
    double t102 = -t36+t52+t53+t54;
    double t103 = (t12-t35)*c6;
    double t104 = s6*(t21+t25);
    double t108 = c6*(t58+t59);
    double t109 = s6*(t64+t68);
    double t111 = t101-s5*(t62-t73);

    Jtilde(0,0) = t36+t38+t39+t40-c1*s3*(33.0/400.0)-s1*s2*(79.0/250.0)-(t12-t35)*c6*(107.0/1000.0)-c6*(t21+t25)*(11.0/125.0)-s6*(t21+t25)*(107.0/1000.0)-s1*c2*c3*(33.0/400.0)-s1*s2*c4*(48.0/125.0);
    Jtilde(1,0)=t78;
    Jtilde(2,0)=0.0;
    Jtilde(3,0)=0.0;
    Jtilde(4,0)=0.0;
    Jtilde(5,0)=1.0;

    Jtilde(0,1)=c1*t49;
    Jtilde(1,1)=s1*t49;
    Jtilde(2,1)=-c1*t78-s1*t95;
    Jtilde(3,1)=-s1;
    Jtilde(4,1)=c1;
    Jtilde(5,1)=0.0;

    Jtilde(0,2)=-c2*t95+s1*s2*t49;
    Jtilde(1,2)=c2*t78-c1*s2*t49;
    Jtilde(2,2)=-s1*s2*t78+c1*s2*t95;
    Jtilde(3,2)=c1*s2;
    Jtilde(4,2)=s1*s2;
    Jtilde(5,2)=c2;

    Jtilde(0,3)=-(c1*c3-s3*s1*c2)*t79-s3*s2*t96;
    Jtilde(1,3)=-(t65+t66)*t79-s3*s2*(t72+t74+t75+t77-t80-t81-t82-t83);
    Jtilde(2,3)=-(t65+t66)*t96+(c1*c3-s3*s1*c2)*(t72+t74+t75+t77-t80-t81-t82-t83);
    Jtilde(3,3)=(t65+t66);
    Jtilde(4,3)=-c1*c3+s3*s1*c2;
    Jtilde(5,3)=-s3*s2;

    Jtilde(0,4)=-(t12-t35)*t84-(c2*c4+t33)*t97;
    Jtilde(1,4)=(c2*c4+t33)*(t72+t75+t77-t80-t81-t82)-(t58+t59)*t84;
    Jtilde(2,4)=(t12-t35)*(t72+t75+t77-t80-t81-t82)+(t58+t59)*t97;
    Jtilde(3,4)=(t58+t59);
    Jtilde(4,4)=-t12+t35;
    Jtilde(5,4)=(c2*c4+t33);


    Jtilde(0,5)=-t85*(t98-t112)-(t86-t113)*t102;
    Jtilde(1,5)=-(t86-t113)*t99-t85*t111;
    Jtilde(2,5)=-t99*(t98-t112)+t102*t111;
    Jtilde(3,5)=t111;
    Jtilde(4,5)=-t98+t112;
    Jtilde(5,5)=(t86-t113);

    Jtilde(0,6)=(t45-t51)*(t103+t104)-(t89-t114)*(t52+t54);
    Jtilde(1,6)=-(t89-t114)*(t80+t82)+(t45-t51)*(t108+t109);
    Jtilde(2,6)=(t103+t104)*(t80+t82)-(t52+t54)*(t108+t109);
    Jtilde(3,6)=-t108-t109;
    Jtilde(4,6)=(t103+t104);
    Jtilde(5,6)=(t89-t114);

    return Jtilde;
}
double ForceController::first_order_lowpass_filter()
{
    filt_ = (tau_ * filt_old_ + delta_time*f_cur_buffer_)/(tau_ + delta_time);
    filt_old_ = filt_;
    return filt_;
}
PLUGINLIB_EXPORT_CLASS(ForceController, controller_interface::ControllerBase)
