#if !defined(__VISION_SERVO_H__)
#define __VISION_SERVO_H__

/* public */
// ros
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

// std
#include <sstream>

/* privita */
#include "micro_manipulate/pospub.h"

struct joint
{
    float cmd;
    float stat;
    float error;
    float vis_stat;
};


class VisionServo
{
public:
    VisionServo(ros::NodeHandle &nh);
    ~VisionServo(){};

    void calibrate();

    void run();

private:
    /* variable */
    /* ros */
    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Subscriber joint_sub_;
    std::vector<ros::Publisher> motor_pub_;    
    micro_manipulate::pospub camera_msgs_;

    std::vector<control_toolbox::Pid> pid_controllers_;

    /* algorithm */
    std::vector<joint> joint_;
    Eigen::Matrix2f jaco_raw_;
    Eigen::Vector2f pixel_error_;
    Eigen::Vector2f motor_error_;
    /* function */
    void init();
    void cameraCallback(const micro_manipulate::pospub &msg);
    void jointCallback(const sensor_msgs::JointStateConstPtr &msg);
    void pub_msgs(void);

    const float init_pos_[3];
};


#endif // __VISION_SERVO_H__

