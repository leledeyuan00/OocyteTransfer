#!/usr/bin/env python
from micro_manipulate.srv import *
import rospy

if __name__ == "__main__":
  rospy.init_node("name_node")
  rospy.loginfo("Starting name_node.")

  
  while not rospy.is_shutdown():
    pass
