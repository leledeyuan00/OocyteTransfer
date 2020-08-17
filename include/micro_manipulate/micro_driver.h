#include <sstream>
#include <iostream>
#include "motion.h"
#include "ros/ros.h"


class Driver_Base
{
public:
    Driver_Base();
    Driver_Base(Driver_Base &&) = default;
    Driver_Base(const Driver_Base &) = default;
    Driver_Base &operator=(Driver_Base &&) = default;
    Driver_Base &operator=(const Driver_Base &) = default;
    ~Driver_Base(){};
    

private:
    static bool init_flag;
};

Driver_Base::Driver_Base()
{
    if (!init_flag)
    {
        rm_init();
    }
    init_flag = true;
}
bool Driver_Base::init_flag = false;


class MyDriver : public Driver_Base
{
public:
    MyDriver(const int port, const int id, const std::string name);
    MyDriver(MyDriver &&) = default;
    MyDriver(const MyDriver &) = default;
    MyDriver &operator=(MyDriver &&) = default;
    MyDriver &operator=(const MyDriver &) = default;
    ~MyDriver();

    void write(double cmd,ros::Duration elapsed_time);
    int read(double* pos,double* vel,double* eft);

    const int get_id();
    const std::string get_name();

private:
    const int port_;
    const int id_;
    const std::string name_;

    rm_axis_handle handle_;

    double pos_;
    double vel_;
    double eft_;

    const float cm_sleep;

    // function
    int init();
};