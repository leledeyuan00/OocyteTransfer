#include "vision_servo.h"

VisionServo::VisionServo(ros::NodeHandle &nh):nh_(nh),init_pos_{7,29,5}
{
    jaco_raw_.resize(2,2);
    pid_controllers_.push_back(control_toolbox::Pid(0.1,0,0)); //x
    pid_controllers_.push_back(control_toolbox::Pid(0.1,0,0)); //y
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
        joint_[i].cmd = init_pos_[i];
        cmd.data = joint_[i].cmd;
        motor_pub_[i].publish(cmd);
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }  
}

void VisionServo::cameraCallback(const micro_manipulate::pospub &msg)
{
    joint_[0].vis_stat = msg.carrying[1];
    joint_[1].vis_stat = msg.carrying[0];
    joint_[0].error = (msg.carrying[1] - msg.pippet[1]);
    joint_[1].error = (msg.carrying[0] - msg.pippet[0]);
    // ROS_INFO("Eror is %f, %f",joint_[0].error,joint_[1].error);
    // ROS_INFO("peppet is [%f %f] \r\n carrying is [%f %f]\r\n", msg.pippet[0],msg.pippet[1],msg.carrying[0],msg.carrying[1]);
}

void VisionServo::jointCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    for (size_t i = 0; i < joint_.size(); i++)
    {
        joint_[2-i].stat = msg->position[i];
    }
    
}

void VisionServo::pub_msgs(void)
{
    for (size_t i = 0; i < motor_pub_.size(); i++)
    {
        std_msgs::Float64 cmd;
        cmd.data = joint_[i].cmd;
        motor_pub_[i].publish(cmd);
    }
    
}

void VisionServo::calibrate()
{
    uint8_t cal_axis = 0;
    bool success = false;
    int init_count = 0;
    Eigen::Vector2f Jaco_x;
    Eigen::Vector2f Jaco_y;
    Eigen::Vector2f state1( joint_[0].vis_stat , joint_[1].vis_stat);
    Eigen::Vector2f state2;
    Eigen::Vector2f state3;
    std::cout << "cur state \r\n" << state1 << std::endl;
    while (!success)
    {
        ros::Rate loop_rate(40);
        switch (cal_axis)
        {
            // x axis
        case 0:
        {
            joint_[0].cmd = init_pos_[0] + 1;
            if(init_count > 80)
            {
                cal_axis++;
                init_count = 0;
                state2 = Eigen::Vector2f( joint_[0].vis_stat , joint_[1].vis_stat);
                std::cout << "aft_state2 \r\n" << state2 << std::endl;
                Jaco_x = state2 - state1;
            }
            else init_count++;

            break;
        }
            // y axis
        case 1:
        {
            joint_[1].cmd = init_pos_[1] - 1;
            if(init_count > 80)
            {
                cal_axis++;
                init_count = 0;
                state3 = Eigen::Vector2f( joint_[0].vis_stat , joint_[1].vis_stat);
                std::cout << "aft_state3 \r\n" << state3 << std::endl;
                Jaco_y = state2 - state3;
            }
            else init_count++;
            break;
        }
        case 2:
        {
            jaco_raw_ << Jaco_x , Jaco_y;
            std::cout << "Jacobian \r\n" << jaco_raw_ << std::endl;
            success = true;
            break;
        }
        default:
            break;
        }
        pub_msgs();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}

void VisionServo::run()
{
    int init_cout = 0;
    bool init_flag= false;
    ros::Rate loop_rate(40);
    ros::Time last_time,curr_time;
    ros::Duration control_duration;
    while (ros::ok())
    {
        /* code for loop body */
        // calibrate
        curr_time = ros::Time::now();
        control_duration = curr_time - last_time;
        if (init_cout > 80 && !init_flag)
        {
            calibrate();
            init_flag = true;
        }
        else init_cout++;

        // pid error
        if(init_flag)
        {
            for (size_t i = 0; i < pid_controllers_.size(); i++)
            {
                pixel_error_(i) = pid_controllers_[i].computeCommand(joint_[i].error,control_duration);
            }
            motor_error_ = jaco_raw_.inverse() * pixel_error_;
            std::cout << "motor_error after jacobian \r\n" << motor_error_ << std::endl;
            joint_[0].cmd = joint_[0].stat - motor_error_[0];
            joint_[1].cmd = joint_[1].stat - motor_error_[1];
            pub_msgs();
        }
        
        last_time = curr_time;
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
