import gxipy as gx
# 枚举设备。dev_info_list 是设备信息列表,列表的元素个数为枚举到的设备个数,列表元素是
device_manager = gx.DeviceManager()
dev_num, dev_info_list = device_manager.update_device_list()
if dev_num == 0:
  sys.exit(1)

str_sn = dev_info_list[0].get("sn")
# 通过序列号打开设备
cam = device_manager.open_device_by_sn(str_sn)

cam.stream_on()

num = 1
for i in range(num):
  # 打开第 0 通道数据流
  raw_image = cam.data_stream[0].get_image()
  # 从黑白原始图像获取 numpy 数组
  numpy_image = raw_image.get_numpy()
  if numpy_image is None:
    continue
  
  image = Image.fromarray(numpy_image, 'L')
  image.show()
  image.save("acquisition_mono_image.jpg")
# 停止采集
cam.stream_off()