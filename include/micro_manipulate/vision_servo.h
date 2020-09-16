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
#include "micro_manipulate/camera_servo.h"


class VisionServo
{
public:
    VisionServo(ros::NodeHandle &nh):nh_(nh)
    {
        init();
    }
    ~VisionServo(){};

    void run();

private:
    /* variable */
    /* ros */
    ros::NodeHandle nh_;
    ros::ServiceClient pixel_parse_srv_;

    ros::Rate loop_rate_;

    micro_manipulate::camera_servo camera_msg_;

    
    /* function */
    void init();
};


#endif // __VISION_SERVO_H__

