/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/8/19
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 * Adopted and modified on: 15/10/19
 *     Author: Tran Nguyen Le <tran.nguyenle@aalto.fi>
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>
#include <mujoco.h>
#include <mujoco_ros_control/RobotHWMujoco.h>
#include <mujoco_ros_control/RenderImage.h>
#include <image_transport/image_transport.h>
#include <controller_manager/controller_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <mujoco_ros_control/ChangePose.h>


std::unique_ptr<RobotHWMujoco> hw;
std::unique_ptr<controller_manager::ControllerManager> cm;

/** @brief Update controller manager and current time. If curent time is not initialized than call reset. */
void cb_controller(const mjModel *m, mjData *d, bool reset_controller = false) {
    hw->read(*d);
    cm->update(ros::Time::now(), ros::Duration(m->opt.timestep), reset_controller);
    hw->write(*d);
}

void reset_joints(ros::NodeHandle &node, const mjModel &m, mjData &d) {
    const auto n = (size_t) m.njnt;
    for (size_t i = 0; i < n; ++i) {
        const auto joint_type = m.jnt_type[i];
        if (joint_type == mjJNT_FREE || joint_type == mjJNT_BALL) {
            continue;
        }

        const auto joint_name = mj_id2name(&m, mjOBJ_JOINT, i);
        const auto value = node.param(std::string(joint_name) + "_init", 0.0);
        d.qpos[(size_t) m.jnt_qposadr[i]] = value;
    }
}
// =================================================================================
Eigen::Matrix4d forwardKinematic(Eigen::VectorXd &q, int start, int end)
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
Eigen::VectorXd read_ft_data(const mjModel *m, const mjData *d, Eigen::Matrix4d &T0F,Eigen::Matrix4d &T7F,Eigen::Matrix3d &r0F){
    //// read torque data in 7th joint (wrist)
    Eigen::VectorXd torque_data(3); // variable to hold torque data in world frame
    int torque_sensor_id = 7; // torque sensor id
    int adr_torque = m->sensor_adr[torque_sensor_id]; // get address of torque sensor
    int dim_torque = m->sensor_dim[torque_sensor_id]; // get dimension of torque sensor
    mjtNum torque_data_temp[dim_torque]; //mjtNum variable to hold torque data from sensor in sensor frame
    mju_copy(torque_data_temp, &d->sensordata[adr_torque], dim_torque); // copy data from mjData.sensordata
    Eigen::VectorXd torque_data_temp_vec(3); //vector variable to hold torque data from sensor in sensor frame
    torque_data_temp_vec << torque_data_temp[0], torque_data_temp[1], torque_data_temp[2]; // copy torque data from mjtNum into Vector

    // read force data in 7th joint (wrist)
    int force_sensor_id = 8; //force sensor id
    Eigen::Vector3d force_data; // variable to hold force data in world frame
    int adr_force = m->sensor_adr[force_sensor_id]; // get address of force sensor
    int dim_force = m->sensor_dim[force_sensor_id]; // get dimension of force sensor
    mjtNum force_data_temp[dim_force]; //mjtNum variable to hold force data from sensor in sensor frame
    mju_copy(force_data_temp, &d->sensordata[adr_force], dim_force); // copy data from mjData.sensordata
    Eigen::VectorXd force_data_temp_vec(3); //vector variable to hold force data from sensor in sensor frame
    force_data_temp_vec << force_data_temp[0], force_data_temp[1], force_data_temp[2]; // copy force data from mjtNum into Vector
    Eigen::VectorXd force_data_temp_vec_global(3); //vector variable to hold force data from sensor in world frame
    force_data_temp_vec_global = r0F * force_data_temp_vec; // transform force data from sensor frame into world frame
    force_data << force_data_temp_vec_global(0), force_data_temp_vec_global(1), force_data_temp_vec_global(2); // copy force data in world frame into force_data vector

    //// convert local torque to global torque
    Eigen::VectorXd d_7(4); d_7 << 0.00370456 ,0.00370629, 0.111054, 1; // position of COM of hand in frame 7 (from inertial tag in xml file for lumi_link7)
    Eigen::VectorXd d_F(4); d_F = T7F.inverse() * d_7; //COM position in sensor frame
    Eigen::VectorXd d_0(4); d_0 = T0F * d_F; // COM position in global frame

    Eigen::Vector3d d_0_; d_0_ << d_0(0), d_0(1), d_0(2); // cross product command needs Vector3d type vector. So we copy data in a new vector
    torque_data = d_0_.cross(force_data); // cross product of distance in world frame and force in world frame to calculate torque in world frame

    // put force-torque data in world frame together in one vector
    Eigen::VectorXd ft_data(6); // force-torque data in world frame
    ft_data << force_data, torque_data;
    Eigen::VectorXd ft_data_calib(6); // offset of ft readings
    ft_data_calib << 0.00187174, -0.000114789, 11.1045, -0.0579166, -8.66481, -7.98075e-05;
    ft_data = ft_data - ft_data_calib;
    return ft_data;
}

//========================================================================================================
int main(int argc, char **argv) {
    ros::init(argc, argv, "mujoco_control");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    const auto default_model_path = ros::package::getPath("mujoco_ros_control") + "/model/simple_robot.urdf";
    // const auto default_model_path = ros::package::getPath("lumi_description") + "/robots/robot_table_env.xml";
    const auto model_path = node.param("model", default_model_path);


    // Grasp object
    std::string grasp_object;
    const bool graspable = node.getParam("grasp_name", grasp_object);

    // The camera look at vector defines direction of the camera which it points to.
    const float default_camera_lookat_at[3] = {0.44, -2.32, -1};

    float camera_look_at[3];
    camera_look_at[0] = node.param<float>("look_at_x", default_camera_lookat_at[0]);
    camera_look_at[1] = node.param<float>("look_at_y", default_camera_lookat_at[1]);
    camera_look_at[2] = node.param<float>("look_at_z", default_camera_lookat_at[2]);

    const auto key_path = std::string(getenv("HOME")) + "/.mujoco/mjkey.txt";

    if (!mj_activate(key_path.c_str())) {
        ROS_ERROR_STREAM("Cannot activate mujoco with key: " << key_path);
        return -1;
    }

    char error[1000] = "";
    auto m = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    if (!m) {
        ROS_ERROR_STREAM("Cannot load model: " << model_path);
        ROS_ERROR_STREAM(error);
        return -1;
    }

    auto d = mj_makeData(m);
    if (!d) {
        ROS_ERROR_STREAM("Cannot make data structure for model.");
        return -1;
    }

    hw.reset(new RobotHWMujoco(*m));
    cm.reset(new controller_manager::ControllerManager(hw.get(), node));
    mjcb_control = [](const mjModel *m, mjData *d) { cb_controller(m, d); };

    reset_joints(node, *m, *d);
    std::mutex mutex_data;



    const auto timer = node.createTimer(ros::Duration(m->opt.timestep), [&](const ros::TimerEvent &e) {
        std::lock_guard<std::mutex> l(mutex_data);
        mj_step(m, d);
    });

    boost::function<bool(std_srvs::Empty::Request &, std_srvs::Empty::Response &)> reset =
            [&](std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
                std::lock_guard<std::mutex> l(mutex_data);
                mj_resetData(m, d);
                reset_joints(node, *m, *d);
                cb_controller(m, d, true);
                return true;
            };
    const auto reset_server = node.advertiseService("reset", reset);

     // Reset the robot pose and an a graspable object position. Currently, does not change z-position of the graspable object.
     boost::function<bool(mujoco_ros_control::ChangePose::Request &, mujoco_ros_control::ChangePose::Response &)> reset_table =
             [&](mujoco_ros_control::ChangePose::Request &request, mujoco_ros_control::ChangePose::Response &response) {

                 {
                     std::lock_guard<std::mutex> l(mutex_data);
                     mj_resetData(m, d);
                     reset_joints(node, *m, *d);
                     cb_controller(m, d, true);

                     int graspable_id =  mj_name2id(m, mjOBJ_JOINT, grasp_object.c_str());

                     if( graspable_id >= 0 ) {
                         d -> qpos[m->jnt_qposadr[graspable_id]] = request.x; // Object stays on table (at least): -0.5 and 0.5
                         d -> qpos[m->jnt_qposadr[graspable_id] + 1] = request.y; // Object stays on table (at least): -0.2 and -0.4
                         response.success = true;
                     } else {
                         // If a grasp object has not been given. (could not wrap the whole service within a condition ?? )
                         ROS_ERROR_STREAM("The graspable object " << grasp_object.c_str() << " cannot be found.");
                         response.success = false;
                     }

                 }
                 return true;
             };

     const auto reset_table_server = node.advertiseService("reset_table", reset_table);

    const auto init_reset_delay = node.param("mujoco_initial_reset_delay", 1.0);
    const auto initial_reset = node.createTimer(ros::Duration(init_reset_delay), [&](const ros::TimerEvent &e) {
        std_srvs::Empty srv;
        reset(srv.request, srv.response);
    }, true);

    image_transport::ImageTransport it(node);
    image_transport::Publisher pub_rgb = it.advertise("rgb", 1);
    image_transport::Publisher pub_depth = it.advertise("depth", 1);
    ros::Publisher pub_ftsensor = node.advertise<geometry_msgs::WrenchStamped>("/lumi_mujoco/F_ext", 1);
    ros::Publisher pub_massmatrix = node.advertise<std_msgs::Float64MultiArray>("/lumi_mujoco/MassMatrix", 1);

    const auto timer_rendered = node.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent &e) {

        thread_local RenderImage renderer(m, camera_look_at);

        if ((pub_rgb.getNumSubscribers() == 0) && (pub_depth.getNumSubscribers() == 0)) {
            return;
        }

        {
            std::lock_guard<std::mutex> l(mutex_data);
            renderer.updateScene(m, d);
        }
        cv::Mat rgb, depth;
        std::tie(rgb, depth) = renderer.render();

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        if (pub_rgb.getNumSubscribers() != 0) {
            header.frame_id = "rgb";
            pub_rgb.publish(cv_bridge::CvImage(header, "rgb8", rgb).toImageMsg());
        }

        if (pub_depth.getNumSubscribers() != 0) {
            header.frame_id = "depth";
            pub_depth.publish(cv_bridge::CvImage(header, "depth", depth).toImageMsg());
        }
        if (pub_ftsensor.getNumSubscribers() != 0) {
            header.frame_id = "ftsensor";
            geometry_msgs::WrenchStamped array;
            Eigen::VectorXd q(7);
            for (int i = 0; i < 7; ++i){
              q(i) = d->sensordata[i];
            }
            Eigen::Matrix4d T0F = forwardKinematic(q, 0, 8); // get transformation matrix of sensor frame relative to base frame
            Eigen::Matrix3d r0F;
            r0F = T0F.block(0, 0, 3, 3); // get rotation matrix of sensor frame relative to base frame
            Eigen::Matrix4d T7F = forwardKinematic(q, 7, 8); // get transformation matrix of sensor frame relative to frame 7
            Eigen::VectorXd ft_sensor = read_ft_data(m, d,T0F,T7F,r0F);
            array.wrench.force.x = ft_sensor[0];
            array.wrench.force.y = ft_sensor[1];
            array.wrench.force.z = ft_sensor[2];
            array.wrench.torque.x = ft_sensor[3];
            array.wrench.torque.y = ft_sensor[4];
            array.wrench.torque.z = ft_sensor[5];
            pub_ftsensor.publish(array);
        }
        // if (pub_massmatrix.getNumSubscribers() != 0) {
        //     header.frame_id = "massmatrix";
        //     geometry_msgs::WrenchStamped mass_array;
        //     Eigen::MatrixXd mass_matrix_vector = calculateMass(m, d);
        //     pub_ftsensor.publish(mass_matrix_vector);
        // }
    });

    ros::waitForShutdown();
    mj_deleteModel(m);
    mj_deleteData(d);
    mj_deactivate();

    return 0;
}
