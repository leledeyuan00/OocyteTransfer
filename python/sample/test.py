import gxipy as gx
import sys
  device_manager = gx.DeviceManager()
  dev_num, dev_info_list = device_manager.update_device_list()
  if dev_num == 0:
  sys.exit(1)

  strSN = dev_info_list[0].get("sn")
  cam = device_manager.open_device_by_sn(strSN)

  cam.stream_on()

  num = 1
  for i in range(num):
  raw_image = cam.data_stream[0].get_image()
  rgb_image = raw_image.convert("RGB")
  if rgb_image is None:
  continue
  numpy_image = rgb_image.get_numpy_array()
  if numpy_image is None:
  continue
  image = Image.fromarray(numpy_image, 'RGB')
  image.show()
  image.save("image.jpg")
  cam.stream_off()
  cam.close_device()