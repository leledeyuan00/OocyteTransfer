#include "vision_servo.h"

VisionServo::VisionServo(ros::NodeHandle &nh):nh_(nh),init_pos_{7,29,5}
{
    init();
}

void VisionServo::init()
{
    camera_sub_ = nh_.subscribe("/camera_pub", 10, &VisionServo::cameraCallback,this);
    joint_sub_ = nh_.subscribe("/joint_states", 10, &VisionServo::jointCallback,this);
    motor_pub_.push_back(nh_.advertise<std_msgs::Float64>("/joint3_position_controller/command", 10));
    motor_pub_.push_back(nh_.advertise<std_msgs::Float64>("/joint2_position_controller/command", 10));
    motor_pub_.push_back(nh_.advertise<std_msgs::Float64>("/joint1_position_controller/command", 10));
    joint_.resize(3);

    while (!motor_pub_[0].getNumSubscribers())
    {
        
    }
    ros::Duration(0.5).sleep();
    for (size_t i = 0; i < motor_pub_.size(); i++)
    {
        std_msgs::Float64 cmd;
        cmd.data = init_pos_[i];
        motor_pub_[i].publish(cmd);
        ros::Duration(0.001).sleep();
    }    
}

void VisionServo::cameraCallback(const micro_manipulate::pospub &msg)
{
    joint_[0].error = msg.carrying[1] - msg.pippet[1];
    joint_[1].error = -(msg.carrying[0] - msg.pippet[0]);
    ROS_INFO("Eror is %f, %f",joint_[0].error,joint_[1].error);
    // ROS_INFO("peppet is [%f %f] \r\n carrying is [%f %f]\r\n", msg.pippet[0],msg.pippet[1],msg.carrying[0],msg.carrying[1]);
}

void VisionServo::jointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    for (size_t i = 0; i < joint_.size(); i++)
    {
        joint_[i].stat = msg->position[i];
    }
    
}

void VisionServo::pub_msgs(void)
{
    for (size_t i = 0; i < motor_pub_.size(); i++)
    {
        /* code */
    }
    
}

void VisionServo::calibrate()
{
    uint8_t cal_axis = 0;
    bool success = false;
    while (!success)
    {
        switch (cal_axis)
        {
            // x axis
        case 0:
        {
            
            break;
        }
            // y axis
        case 1:
        {
            success = true;
        }
            break;
        default:
            break;
        }
    }
    
}

void VisionServo::run()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        /* code for loop body */
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "vision_servo");
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner(3);
    // spinner.start();

    VisionServo vs(nh);
    vs.run();
}
