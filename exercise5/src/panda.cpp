//
// Created by isaac on 7/22/19.
//

#include "exercise5/panda.h"

Panda::Panda()
{
    num_joints = 7;
}

MatrixXd Panda::calculateJacobian(VectorXd q_in)
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

    MatrixXd Jtilde = MatrixXd::Zero(6,7);
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

Matrix4d Panda::forwardKinematic(VectorXd q, int start, int end)
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
    VectorXd a(10);
    VectorXd d(10);
    VectorXd alfa(10);
    VectorXd theta(10);

    a     << 0  , 0       ,  0      , 0.0825 , -0.0825  , 0      ,  0.088, 0   , 0       , 0      ;
    d     << 0  , 0.333   , 0       ,  0.316 , 0        , 0.384  , 0     , 0   , 0.107   , 0.112; //0.1034 ;
    alfa  << 0  , -M_PI/2 , M_PI/2  , M_PI/2 , -M_PI/2  , M_PI/2 , M_PI/2, 0   , 0       , 0      ;
    theta << 0  , q(0)    , q(1)    ,  q(2)  , q(3)     , q(4)   , q(5)  , q(6), -M_PI/4 , 0      ;

    Matrix4d Ttemp = Eigen::Matrix4d::Identity();  // initialize Ttemp
    for (int i = start; i < end; i++)
    {
        Matrix4d T;

        // modified dh
        T <<    cos(theta(i+1))              , -sin(theta(i+1))                 ,            0  , a(i),
                sin(theta(i+1))*cos(alfa(i)) , cos(theta(i+1)) * cos(alfa(i))   , -sin(alfa(i)) , -d(i+1) * sin(alfa(i)),
                sin(theta(i+1))*sin(alfa(i)) , cos(theta(i+1)) * sin(alfa(i))   , cos(alfa(i))  , d(i+1)*cos(alfa(i)),
                0                                 ,         0                              ,         0     , 1 ;

        Ttemp = Ttemp * T;
    }
    return Ttemp;
}

VectorXd Panda::inverseKinematic(VectorXd q_init, VectorXd x_desired, Quaterniond quat_desired)
{
    /*
     * This function calculates inverse kinematics for panda
     * inputs:
     *  - q_init: initial guess for joint angles
     *  - x_desired: desired position
     *  - quat_desired: desired orientation
     */

    double joint_tr_l[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}; //position lower limits
    double joint_tr_h[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}; //position higher limits

    VectorXd q_final(7); // variable for final calculated joint angles
    VectorXd err_x(3); // error in position
    VectorXd err_ori(3); // error in orientation

    Quaterniond quat_err; // error in orientation- quaternion

    double a = 0.1; // learning rate in gradient descent algorithm

    double first_cond; // flag for the first condition of the do-while loop for inverse kinematics calculation
    double second_cond; // flag for the second condition of the do-while loop for inverse kinematics calculation

    do{
        //// FK
        Matrix4d T07 = forwardKinematic(q_init, 0, 7); // transformation matrix of frame 7 with respect to frame 0
        // extract translation vector from transformation matrix
        VectorXd p(3);
        p << T07(0, 3), T07(1, 3), T07(2, 3);
        // extract rotation matrix from transformation matrix
        Matrix3d r;
        r = T07.block(0, 0, 3, 3);
        // convert rotation matrix to quaternion
        Quaterniond quat;
        quat = r; //R2Q

        std::cout << "x_desired: " << x_desired.transpose() << std::endl;
//        std::cout << "ori_desired: " << quat_desired << std::endl;

        // err calculation
        err_x = x_desired - p;
        quat_err = quat_desired * quat.inverse();
        err_ori[0] = quat_err.x();
        err_ori[1] = quat_err.y();
        err_ori[2] = quat_err.z();

        //// Jacobian calculation
        MatrixXd J = calculateJacobian(q_init);
//        MatrixXd Jp = J.block(0, 0, 3, 7); // position part of Jacobian
//        MatrixXd Jr = J.block(3, 0, 3, 7); // rotation part of Jacobian


        //// update q
        VectorXd err(6);
        err << err_x, err_ori;

        //// pseudoinverse of jacobian
        MatrixXd J_pinv = J.transpose() * (J * J.transpose()).inverse();
        q_final = q_init + J_pinv * err; // use this for newton method
//        q_final = q_init + a * J.transpose() * err; // use this for gradient descent method

        //// update q_init
        q_init = q_final;

        //// conditions for finishing IK
        first_cond = err_x.norm();
//        double theta = quat_xyz.transpose() * ori_desired;
//        second_cond = abs(2 * acos((theta > 1 ? 1 : (theta < -1 ? -1 : theta))));
        second_cond = err_ori.norm();

        std::cout << "position condition: " << first_cond << std::endl;
        std::cout << "orientation condition: " << second_cond << std::endl;

    }while(first_cond > 0.001 || second_cond > 0.0001);

    // TODO: check result of ik with joint limitations and raise an error if violated
    for (int i = 0; i < num_joints; ++i)
    {
        if(q_final[i] > joint_tr_h[i] || q_final[i] < joint_tr_l[i])
        {
            std::cerr << "calculated joint angle from IK is out of range!!! please select a valid cartesian pos" << std::endl;
        }
    }

    return q_final;
}
