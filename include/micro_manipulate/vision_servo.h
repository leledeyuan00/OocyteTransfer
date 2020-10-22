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
#include "micro_manipulate/switch_machine.h"

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
    XmlRpc::XmlRpcValue config_id_;
    std::string name_;
    double away_pos_[3];


    ros::ServiceServer state_machine_srv_;    

    std::vector<control_toolbox::Pid> pid_controllers_;

    /* algorithm */
    std::vector<joint> joint_;
    Eigen::Matrix2f jaco_raw_;
    Eigen::Vector2f pixel_error_;
    Eigen::Vector2f motor_error_;
    bool parse_new_pixel_;
    float start_time_;
    int fsmc_;
    bool transfer_case_;
    float transfer_start_time_;
    float transfer_start_pos_;
    /* function */
    void init();
    void cameraCallback(const micro_manipulate::pospub &msg);
    void jointCallback(const sensor_msgs::JointStateConstPtr &msg);
    void pub_msgs(void);
    bool switch_state_machine_service(micro_manipulate::switch_machine::Request &req, micro_manipulate::switch_machine::Response &res);

    double init_pos_[3];
    float vel_;
};


#endif // __VISION_SERVO_H__

