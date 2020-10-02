#include "micro_driver.h"

MyDriver::MyDriver(const int port, const int id, const std::string name):
    port_(port), id_(id), name_(name),cm_sleep(0.003)
{
    init();
}

int MyDriver::init()
{
    std::string msg;
    msg = {"/dev/ttyUSB"+std::to_string(port_)};
    const char * port_msg = msg.c_str();
    
    handle_ = rm_open_axis_modbus_rtu(port_msg,115200,id_);

    rm_set_servo_on_off(handle_,true);
    rm_reset_error(handle_);
    rm_go_home(handle_);
    ros::Duration(cm_sleep).sleep();

    std::cout << "initializing" << std::endl;
}

void MyDriver::write(double cmd,ros::Duration elapsed_time)
{
    pos_cmd_ = cmd;
    rm_move_absolute(handle_,(float)cmd,100,100,100,0.001);
    ros::Duration(cm_sleep).sleep();
}

int MyDriver::read(double *pos,double *vel,double *eft)
{
    // pos_ = (double)rm_read_current_position(handle_);
    // vel_ = (double)rm_read_current_velocity(handle_);
    // eft_ = (double)rm_read_current_torque(handle_);
    pos_ = pos_cmd_;

    *pos = pos_;
    // *vel = vel_;
    // *eft = eft_;
    // ros::Duration(cm_sleep).sleep();
    return 0;
}

const int MyDriver::get_id()
{
    return id_;
}

const std::string MyDriver::get_name()
{
    return name_;
}

MyDriver::~MyDriver()
{
    rm_set_servo_on_off(handle_,false);
    rm_close_axis(handle_);
    ros::Duration(cm_sleep).sleep();
}