#include "vision_servo.h"

void VisionServo::init()
{

    pixel_parse_srv_ = nh_.serviceClient<micro_manipulate::camera_servo>("/pixel_get");

}

void VisionServo::run()
{
    camera_msg_.request.path = "/home/jiang/micro_ws/src/3axis_platform/python/materials/imgs/2.jpg";
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        /* code for loop body */
        ROS_INFO("test");
        pixel_parse_srv_.call(camera_msg_);
        ROS_INFO("after service pixel is \r\n pippete [%f,%f] \r\n carrying [%f,%f]",camera_msg_.response.pippet[0],camera_msg_.response.pippet[1],
            camera_msg_.response.carrying[0],camera_msg_.response.carrying[1]
        );
        ROS_INFO("test2");
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("test3");
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
