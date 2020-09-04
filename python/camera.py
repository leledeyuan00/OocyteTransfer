#!/usr/bin/env python
from __future__ import print_function

import gxipy as gx
from PIL import Image


# from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse
from micro_manipulate.srv import *


#!/usr/bin/env python
import rospy

class ROSNode: 

  z_depth = 0
  num = 0
  z_last = z_depth

  def camera_init(self):
    self.device_manager = gx.DeviceManager()
    dev_num, dev_info_list = self.device_manager.update_device_list()
    if dev_num == 0:
      sys.exit(1)

    strSN = dev_info_list[0].get("sn")

    self.cam = self.device_manager.open_device_by_sn(strSN)
    self.cam.stream_on()


  def acquire(self):
    self.cam.TriggerSoftware.send_command()

    raw_image = self.cam.data_stream[0].get_image()

    numpy_image = raw_image.get_numpy_array()
    if numpy_image is None:
      pass

    image = Image.fromarray(numpy_image, 'L')
    image.save("pics/"+str(self.z_depth) + "mm" + "-" +str(self.num)+".png")

    

  def handle_camera_cmd(self,req):

    self.z_depth = round(req.z_axis * 1000) / 1000
    if self.z_last == self.z_depth:
      self.num = self.num +1   
    else:
      self.num = 0
      self.z_last = self.z_depth
      
    self.acquire()
    return camera_cmdResponse(True)

  def __init__(self):
    self.camera_init()
    rospy.init_node("camera_cmd")
    rospy.loginfo("Starting ROSNode as camera_cmd.")
    camera_srv = rospy.Service('camera_cmd', camera_cmd, self.handle_camera_cmd)
    print("Ready to receive camera cmd.")


if __name__ == "__main__":
  camera_cmd_node = ROSNode()
  rospy.spin()