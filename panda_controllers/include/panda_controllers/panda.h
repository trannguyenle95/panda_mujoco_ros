//
// Created by isaac on 7/22/19.
//

#ifndef PANDA_MUJOCO_PANDA_H
#define PANDA_MUJOCO_PANDA_H

#include <iostream>

//#include <mujoco.h>
#include "cstdio"
#include "cstdlib"
#include "cstring"
//#include "glfw3.h"

#include <math.h>
#include <algorithm>
#include <cstdlib>

#include <chrono>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <fstream>

using namespace Eigen;

class Panda{

    int num_joints;

public:
    Panda();
    MatrixXd calculateJacobian(VectorXd q_in);
    Matrix4d forwardKinematic(VectorXd q, int start, int end);
    VectorXd inverseKinematic(VectorXd q_init, VectorXd x_desired, Quaterniond quat_desired);
};


#endif //PANDA_MUJOCO_PANDA_H
