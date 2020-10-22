#include "vision_servo.h"

VisionServo::VisionServo(ros::NodeHandle &nh):nh_(nh),name_("micro_manipulate")
{
    jaco_raw_.resize(2,2);
    pid_controllers_.push_back(control_toolbox::Pid(0.025,0,0.001)); //x
    pid_controllers_.push_back(control_toolbox::Pid(0.025,0,0.001)); //y
    start_time_ = 0;
    fsmc_ = 0;
    parse_new_pixel_ = false;
    transfer_case_ = false;
    away_pos_[0] = 0;
    away_pos_[1] = 0;
    away_pos_[2] = 30;
    init();

}

void VisionServo::init()
{
    camera_sub_ = nh_.subscribe("/camera_pub", 10, &VisionServo::cameraCallback,this);
    joint_sub_ = nh_.subscribe("/joint_states", 10, &VisionServo::jointCallback,this);
    state_machine_srv_ = nh_.advertiseService("/servo_sm", &VisionServo::switch_state_machine_service,this);
    motor_pub_.push_back(nh_.advertise<std_msgs::Float64>("/joint1_position_controller/command", 10));
    motor_pub_.push_back(nh_.advertise<std_msgs::Float64>("/joint2_position_controller/command", 10));
    motor_pub_.push_back(nh_.advertise<std_msgs::Float64>("/joint3_position_controller/command", 10));
    joint_.resize(3);
    if (!nh_.getParam(name_+"/config_id", config_id_))
        { ROS_ERROR("No num param"); }
    for (size_t i = 0; i < 3; i++)
    {
        init_pos_[i] = config_id_[i]["init_pos"];
        joint_[i].cmd = init_pos_[i];
    }
    

    while (!motor_pub_[0].getNumSubscribers())
    {
        // must run after roslaunch micro_manipulate micro_control.launch
    }
    ros::Duration(0.5).sleep();
}

bool VisionServo::switch_state_machine_service(micro_manipulate::switch_machine::Request &req, micro_manipulate::switch_machine::Response &res)
{
    int cmd;
    cmd = req.cmd;
    if(cmd == 7 || cmd ==8 || cmd ==9)
    {
        if(cmd == 7)
        {
            vel_ += 0.001;
        }
        else if(cmd == 8)
        {
            vel_ -= 0.001;
        }
        else if(cmd == 9)
        {
            vel_ = 0;
        }
        fsmc_ = 7;
    }
    else{
        fsmc_ = cmd;
    }

    res.success = true;
    return true;
}

void VisionServo::cameraCallback(const micro_manipulate::pospub &msg)
{
    static float last_error[2] = {0,0};
    joint_[0].vis_stat = msg.carrying[1];
    joint_[1].vis_stat = msg.carrying[0];
    // joint_[0].error = (msg.carrying[1] - (msg.pippet[1] )) * 0.5 + (last_error[0] * 0.5);
    // joint_[1].error = (msg.carrying[0] - (msg.pippet[0] )) * 0.5 + (last_error[1] * 0.5);
    joint_[0].error = (msg.carrying[1] - (301.0 ))* 0.5 + (last_error[0] * 0.5);
    joint_[1].error = (msg.carrying[0] - (374.0 ))* 0.5 + (last_error[1] * 0.5);
    parse_new_pixel_ = true;
    last_error[0] = joint_[0].error;
    last_error[1] = joint_[1].error;
    // ROS_INFO("Eror is %f, %f",joint_[0].error,joint_[1].error);
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
    float loop_duration = 0.033;
    float rate = 1.0/loop_duration;
    ros::Rate loop_rate(rate);
    ros::Time last_time,curr_time;
    ros::Duration control_duration;
    float start_state[2];
    Eigen::Vector2f pixel_error_temp;
    float error_threshold = 100;

    // run loop
    while (ros::ok())
    {
        /* code for loop body */
        switch (fsmc_)
        {
        case 0:
            {
                // calibrate
                curr_time = ros::Time::now();
                control_duration = curr_time - last_time;
                // for a stable loop duration
                if (init_cout > 80 && !init_flag)
                {
                    // for Catch a Jacobian raw
                    calibrate();
                    init_flag = true;
                }
                else init_cout++;

                // pid error
                if(init_flag)
                {

                    // if(parse_new_pixel_)
                    // {
                    //     parse_new_pixel_ = false;
                    //     start_time_ = 0.1;
                    //     for (size_t i = 0; i < pid_controllers_.size(); i++)
                    //     {
                    //         start_state[i] = joint_[i].stat;
                    //         pixel_error_(i) = pid_controllers_[i].computeCommand(joint_[i].error,control_duration);
                    //     }
                    // }
                    // else{
                    //     start_time_ += 0.3;
                    // }
                    // if(start_time_ <= 1.0)
                    // {
                    //     pixel_error_temp = pixel_error_*start_time_;
                    //     motor_error_ = jaco_raw_.inverse() * pixel_error_temp;
                    //     for (size_t i = 0; i < pid_controllers_.size(); i++)
                    //     {
                    //         joint_[i].cmd = start_state[i] - motor_error_[i];
                    //     }
                    // }

                    for (size_t i = 0; i < pid_controllers_.size(); i++)
                        {
                            // start_state[i] = joint_[i].stat;
                            pixel_error_(i) = pid_controllers_[i].computeCommand(joint_[i].error,control_duration);
                            if(pixel_error_(i) >=error_threshold)
                                pixel_error_(i) = error_threshold;
                            else if(pixel_error_(i)<=-error_threshold)
                                pixel_error_(i) = -error_threshold;
                        }
                    motor_error_ = jaco_raw_.inverse() * pixel_error_;
                    joint_[0].cmd = joint_[0].stat - motor_error_[0];
                    joint_[1].cmd = joint_[1].stat - motor_error_[1];
                    std::cout << "motor_error after jacobian \r\n" << motor_error_ << std::endl;
                    last_time = curr_time;
                }       
            break;
            }
        case 1 :
        {
            for (size_t i = 0; i < 3; i++)
            {
                joint_[i].cmd = away_pos_[i];
            }
            break;
        }
        case 2 :
        {
            for (size_t i = 0; i < pid_controllers_.size(); i++)
            {
                joint_[i].cmd = init_pos_[i];
            }
            joint_[2].cmd = init_pos_[2] - 0.5;
            break;
        }
        case 3:
        {
            for (size_t i = 0; i < pid_controllers_.size(); i++)
            {
                joint_[i].cmd = init_pos_[i];
            }
            joint_[2].cmd = init_pos_[2];
            break;
        }
        case 4:
        {
            if(!transfer_case_){
                transfer_case_ = true;
                transfer_start_time_ = 0;
                transfer_start_pos_ = joint_[0].stat;
            }
            if (transfer_start_time_ < 20)
            {
                joint_[0].cmd = transfer_start_pos_ - (2 *(transfer_start_time_ /20));
                transfer_start_time_ += loop_duration;
            }
            break;
        }
        case 5:
        {
            transfer_case_ = false;
            break;
        }
        case 6:
        {
            joint_[0].cmd = transfer_start_pos_;
            break;
        }
        case 7:
        {
            joint_[0].cmd = joint_[0].stat + vel_;
            break;
        }
        case 8:
        {
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

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "vision_servo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();

    VisionServo vs(nh);
    vs.run();
}
