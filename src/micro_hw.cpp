#include "micro_hw.h"

Micro_hw::Micro_hw(ros::NodeHandle* nh):
    nh_(*nh),name_(ros::this_node::getName())
{   
    ros_init();
    motor_init();
    hardware_init();

    controller_manager_.reset(new controller_manager::ControllerManager(this,nh_));

    loop_hz_ = 100;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    my_control_loop_ = nh_.createTimer(update_freq,&Micro_hw::update,this);

}

int Micro_hw::ros_init()
{
    if (!nh_.getParam(name_+"/config_id", config_id_))
        { ROS_ERROR("No num param"); }
    if (!nh_.getParam(name_+"/port", ser_port_))
        { ROS_ERROR("No ser_port_ param"); }
}

int Micro_hw::motor_init()
{
    for (size_t i = 0; i < config_id_.size(); i++)
    {
        const std::string motor_name = config_id_[i]["name"];
        const int id = config_id_[i]["id"];
        
        init_pos_[i] = config_id_[i]["init_pos"];
        std::shared_ptr<MyDriver> motor_poll(new MyDriver(ser_port_,id,motor_name));
        motor_.push_back(motor_poll);
    }
    return 0;
}

int Micro_hw::hardware_init()
{
    joint_.resize(config_id_.size());
    for (size_t joint_id = 0; joint_id < config_id_.size(); joint_id++)
    {
        hardware_interface::JointStateHandle joint_state_handle(
            config_id_[joint_id]["name"],&joint_[joint_id].state.pos,&joint_[joint_id].state.vel,&joint_[joint_id].state.eft
        );
        jnt_st_int_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle_position(
            jnt_st_int_.getHandle(config_id_[joint_id]["name"]),&joint_[joint_id].cmd.pos
        );
        pos_jnt_int_.registerHandle(joint_handle_position);

        hardware_interface::JointHandle joint_handle_velocity(
            jnt_st_int_.getHandle(config_id_[joint_id]["name"]),&joint_[joint_id].cmd.vel
        );
        vel_jnt_int_.registerHandle(joint_handle_velocity);

        hardware_interface::JointHandle joint_handle_effort(
            jnt_st_int_.getHandle(config_id_[joint_id]["name"]),&joint_[joint_id].cmd.eft
        );
        eft_jnt_int_.registerHandle(joint_handle_effort);
        joint_[joint_id].cmd.pos = init_pos_[joint_id];
    }
    registerInterface(&jnt_st_int_);
    registerInterface(&pos_jnt_int_);
    registerInterface(&vel_jnt_int_);
    registerInterface(&eft_jnt_int_);
}

void Micro_hw::read()
{
    for (size_t i = 0; i < config_id_.size(); i++)
    {
        motor_[i]->read(&joint_[i].state.pos,&joint_[i].state.vel,&joint_[i].state.eft);
    }
}

void Micro_hw::update(const ros::TimerEvent& e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_ -> update(ros::Time::now(),elapsed_time_);
    write();
}

void Micro_hw::write()
{
    for (size_t i = 0; i < config_id_.size(); i++)
    {
        motor_[i]->write(joint_[i].cmd.pos,elapsed_time_);
    }
}

void Micro_hw::run()
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {    
        loop_rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "micro_hw");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    Micro_hw myMotor(&nh);
    myMotor.run();

    return 0;    
}