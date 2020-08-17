#pragma once

/* public include */
#include "ros/ros.h"
#include "boost/shared_ptr.hpp"

// hardware interface
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

/* private include */
#include "micro_driver.h"

class Micro_hw : public hardware_interface::RobotHW
{
public:
    Micro_hw(ros::NodeHandle* nh);
    Micro_hw(Micro_hw &&) = default;
    Micro_hw(const Micro_hw &) = default;
    Micro_hw &operator=(Micro_hw &&) = default;
    Micro_hw &operator=(const Micro_hw &) = default;
    ~Micro_hw(){};

    void run();

private:
    ros::NodeHandle nh_;
    std::string name_;
    XmlRpc::XmlRpcValue config_id_;
    int ser_port_;

    ros::Timer my_control_loop_;
    double loop_hz_;
    ros::Duration elapsed_time_;

    std::vector<std::shared_ptr<MyDriver>> motor_;

    //hardware interface
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    hardware_interface::JointStateInterface jnt_st_int_;
    hardware_interface::PositionJointInterface pos_jnt_int_;
    hardware_interface::VelocityJointInterface vel_jnt_int_;
    hardware_interface::EffortJointInterface eft_jnt_int_;

    struct JOINT_INFO
    {
        double pos;
        double vel;
        double eft;
    };

    struct JOINT
    {
        JOINT_INFO state;
        JOINT_INFO cmd;
    };

    std::vector<JOINT> joint_;
    /* function */
    int ros_init();
    int motor_init();
    int hardware_init();

    void read();
    void update(const ros::TimerEvent& e);
    void write();

};
