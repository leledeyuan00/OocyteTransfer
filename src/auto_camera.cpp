#include "micro_manipulate/camera_cmd.h"
#include "std_msgs/Float64.h"
#include <ros/ros.h>

const static double center[3] = {10.4,24.6,8.3};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "auto_camera");
  ros::NodeHandle nh;

  ros::ServiceClient camera_cmd;
  camera_cmd = nh.serviceClient<micro_manipulate::camera_cmd>("/camera_cmd");

  micro_manipulate::camera_cmd camera_srv;

  ros::Publisher motor_pub1;
  motor_pub1 = nh.advertise< std_msgs::Float64>("/joint3_position_controller/command", 10);

  ros::Publisher motor_pub2;
  motor_pub2 = nh.advertise< std_msgs::Float64>("/joint2_position_controller/command", 10);

  ros::Publisher motor_pub3;
  motor_pub3 = nh.advertise< std_msgs::Float64>("/joint1_position_controller/command", 10);

  std_msgs::Float64 motor_cmd[3];

  double cmd[3];

  for (size_t i = 0; i < 3; i++)
  {
    cmd[i] = center[i];
  }

  double size = 3;
  double step = 0.5;
  double depth_step = 0.2;
  double depth = 2;
  int loop =1;

  int dir =0;

  ros::Rate loop_rate(10);

  /* code for loop body */
  for (double dp = center[2]; dp > (center[2] - depth); dp-=depth_step)
  {
    //init 
    cmd[0] = center[0];
    cmd[1] = center[1];
    loop = 1;
    dir = 0;
    while ((loop*step)<=size)
    {
      switch (dir)
      {
        case 0:{
          cmd[0] +=step;
          if ((cmd[0]-center[0]) >= (loop*step))
          {
            dir = 1;
          }
          break;
        }
        case 1:{
          cmd[1] -=step;
          if ((center[1] - cmd[1]) >= (loop*step))
          {
            dir = 2;
          }
          break;
        }
        case 2:{
          cmd[0] -=step;
          if ((center[0] - cmd[0]) >= (loop*step))
          {
            dir = 3;
          }
          break;
        }
        case 3:{
          cmd[1] +=step;
          if ((cmd[1] - center[1]) >= (loop*step))
          {
            dir = 0;
            loop++;
          }
          break;
        }

      default:
        break;
      }
      // parse
      cmd[2] = dp;
      for (size_t i = 0; i < 3; i++)
      {
        motor_cmd[i].data = cmd[i];
      }
      motor_pub1.publish(motor_cmd[0]);
      motor_pub2.publish(motor_cmd[1]);
      motor_pub3.publish(motor_cmd[2]);
      ROS_INFO("motor cmd is [%f , %f , %f]", cmd[0],cmd[1],cmd[2]);

      // catch camera
      camera_srv.request.z_axis =  center[2] - dp;
      camera_cmd.call(camera_srv);
      ros::spinOnce();
      loop_rate.sleep();
    }
    if (!ros::ok())
    {
      break;
    }
  }
  ROS_INFO_STREAM("camera parse is finished"); 
  return 0;  
}