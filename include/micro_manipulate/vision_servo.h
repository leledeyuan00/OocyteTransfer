#if !defined(__VISION_SERVO_H__)
#define __VISION_SERVO_H__

/* public */
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include "std_msgs/String.h"
#include <sstream>

/* privita */
#include "micro_manipulate/pospub.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

struct joint
{
    float cmd;
    float stat;
    float error;
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

    /* algorithm */
    std::vector<joint> joint_;
    /* function */
    void init();
    void cameraCallback(const micro_manipulate::pospub &msg);
    void jointCallback(const sensor_msgs::JointStateConstPtr &msg);
    void pub_msgs(void);

    const float init_pos_[3];
};


#endif // __VISION_SERVO_H__

