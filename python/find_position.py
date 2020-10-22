"""查找图片中的玻璃管和载杆位置
"""

import time
from typing import List, Optional
import cv2
import os
from matplotlib import pyplot as plt
import numpy as np
from collections import defaultdict
from numba import jit

## camera
import gxipy as gx
from PIL import Image

## ros
import rospy
from micro_manipulate.msg import *

from micro_manipulate.srv import *

@jit(nopython=True)
def carrying_for1(gray,roi_col_range0,roi_col_range1,roi_row_range0,roi_row_range1,carrying_ignor_thre,pipet_img):
    last_col_sum = 0
    cur_total_sum = 0
    cur_key = []
    col_intensity = []
    for j in range(roi_col_range0, roi_col_range1):
        cur_col_points = 0
        cur_col_sum = 0
        for i in range(roi_row_range0, roi_row_range1):
            if gray[i, j] > carrying_ignor_thre or pipet_img[i, j] == 255:
                continue
            cur_col_points+=1
            cur_col_sum += ((255 -gray[i, j]) >> 5)**3
        if last_col_sum == 0 and cur_col_points < 3:  # 起始列的点数需要大于3, 可以作为可调参数
            cur_col_sum = 0
        cur_total_sum += cur_col_sum
        if cur_col_sum != 0:
            if last_col_sum == 0:
                cur_key.append(j)
        else:
            if cur_total_sum != 0:
                col_intensity.append(cur_total_sum)
                # col_intensity[cur_key] = cur_total_sum
                cur_total_sum = 0
        last_col_sum = cur_col_sum
    return cur_key,cur_total_sum,col_intensity

@jit(nopython = True)
def carrying_for2(gray,roi_row_range0,roi_row_range1,min_value,carrying_pos):
    for i in range(roi_row_range0, roi_row_range1):
        if gray[i, carrying_pos] < min_value:
            min_value = gray[i, carrying_pos]
            min_value_idx = i
    return min_value,min_value_idx

@jit(nopython = True)
def carrying_for3(gray,min_value_idx,diff_value,min_value,carrying_pos,roi_row_range):
    for i in range(min_value_idx, roi_row_range, -1):
        if gray[i, carrying_pos] > min_value + diff_value:
            return i

@jit(nopython = True)
def carrying_for4(gray,min_value_idx,first_row,diff_value,min_value,roi_row_range,carrying_pos1):
    for i in range(min_value_idx, roi_row_range):
        if gray[i, carrying_pos1] > min_value + diff_value:
            carrying_pos0 = int((i + first_row) / 2)
            return carrying_pos0


class FindPosition:
    """查找图片中玻璃管和载杆的位置

    首先应该设置 `roi_center`, `roi_box` 等属性。
    所有图片的玻璃管位置都是相同的，因此只需要在开始时调用一次 `get_pipet_pos()`
    获取玻璃管的位置即可。接下来通过不断的调用 `get_carrying_pos()` 获取载杆的位置即可。

    Attributes:
        roi_center(List[int]): ROI 中心坐标,[row, col]
        roi_box(List[int]): ROI 长宽,[width, length]
        binaryzation_thre(int): 二值化图像阈值
        fuzzy_len(int): 横向模糊的长度
        pipet_img(Optional[np.ndarray]): ROI 里二值化后的玻璃管图，调用 `get_pipet_pos()` 后生成，可以通过显示这个图来判断玻璃管识别的效果
        pipet_pos(List[int]): 玻璃管的位置, 调用 `get_pipet_pos()` 后生成[row, col]
        carrying_ignor_thre(int): 忽略载杆图像像素的灰度阈值。灰度值大于该值认为是白色背景(255)
        carrying_pos(List[int]): 载杆的位置，调用 `get_carrying_pos()` 后生成[row, col]
    """
    _save_pic = False

    def __init__(self) -> None:
        self._roi_center: List[int] = [0, 0]
        self._roi_box: List[int] = [0, 0]
        self._binaryzation_thre: int = 5
        self._fuzzy_len: int = 10
        self._pipet_img: Optional[np.ndarray] = None
        self._pipet_pos: List[int] = [0, 0]
        self._carrying_ignor_thre: int = 70
        self._carrying_pos: List[int] = [0, 0]
        self._img_dir =  "materials/imgs2"
        self._resized_size : tuple = (600,800)
        self.camera_init()
        self.find_init()
        

    @property
    def roi_center(self) -> List[int]:
        return self._roi_center

    @roi_center.setter
    def roi_center(self, roi_center: List[int]):
        self._roi_center = roi_center
        roi_min_row = int(self._roi_center[0]-self._roi_box[0]/2)
        self._roi_row_range = [roi_min_row, roi_min_row + self._roi_box[0]]
        roi_min_col = int(self._roi_center[1] - self._roi_box[1] / 2)
        self._roi_col_range = [roi_min_col, roi_min_col + self._roi_box[1]]

    @property
    def roi_box(self) -> List[int]:
        return self._roi_box

    @roi_box.setter
    def roi_box(self, roi_box: List[int]):
        self._roi_box = roi_box
        roi_min_row = int(self._roi_center[0]-self._roi_box[0]/2)
        self._roi_row_range = [roi_min_row, roi_min_row + self._roi_box[0]]
        roi_min_col = int(self._roi_center[1] - self._roi_box[1] / 2)
        self._roi_col_range = [roi_min_col, roi_min_col + self._roi_box[1]]

    @property
    def binaryzation_thre(self) -> int:
        return self._binaryzation_thre

    @binaryzation_thre.setter
    def binaryzation_thre(self, binaryzation_thre: int):
        self._binaryzation_thre = binaryzation_thre

    @property
    def fuzzy_len(self) -> int:
        return self._fuzzy_len

    @fuzzy_len.setter
    def fuzzy_len(self, fuzzy_len: int):
        self._fuzzy_len = fuzzy_len

    @property
    def pipet_img(self) -> Optional[np.ndarray]:
        return self._pipet_img

    @property
    def pipet_pos(self) -> List[int]:
        return self._pipet_pos

    @property
    def carrying_ignor_thre(self) -> int:
        return self._carrying_ignor_thre

    @carrying_ignor_thre.setter
    def carrying_ignor_thre(self, carrying_ignor_thre: int):
        self._carrying_ignor_thre = carrying_ignor_thre

    @property
    def carrying_pos(self) -> List[int]:
        return self._carrying_pos

    def get_pipet_pos(self, img: np.ndarray) -> List[int]:
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = img
        laplacian = cv2.Laplacian(gray, cv2.CV_8U)
        # 使用 ROI 并二值化图像
        for j in range(self._roi_col_range[0], self._roi_col_range[1]):
            first_idx = None
            end_idx = None
            for i in range(self._roi_row_range[0], self._roi_row_range[1]):
                if laplacian[i, j] > self._binaryzation_thre:
                    laplacian[i, j] = 255
                    first_idx = i
                    break
            if first_idx is None:
                continue
            for i in range(self._roi_row_range[1], self._roi_row_range[0], -1):
                if laplacian[i, j] > self._binaryzation_thre:
                    laplacian[i, j] = 255
                    end_idx = i
                    break
            if end_idx is None:
                continue
            for i in range(first_idx, end_idx):
                laplacian[i, j] = 255
        # plt.imshow(laplacian, cmap="gray")
        # plt.show()
        # 横向模糊
        self._pipet_img = np.copy(laplacian)
        for j in range(self._roi_col_range[1], self._roi_col_range[0], -1):
            for i in range(self._roi_row_range[0], self._roi_row_range[1]):
                if laplacian[i, j] != 255:
                    for k in range(1, self._fuzzy_len + 1):
                        if laplacian[i, j + k] == 255:
                            self._pipet_img[i, j] = 255
                            break
                    if self._pipet_img[i, j] != 255:
                        self._pipet_img[i, j] = 0
        # plt.imshow(self._pipet_img, cmap="gray")
        # plt.show()
        # 计算位置
        col_intensity = defaultdict(int)
        last_col_num = 0
        cur_total_num = 0
        cur_key = None
        for j in range(self._roi_col_range[1], self._roi_col_range[0], -1):
            cur_col_num = 0
            for i in range(self._roi_row_range[0], self._roi_row_range[1]):
                if (self._pipet_img[i, j] == 255):
                    cur_col_num += 1
            cur_total_num += cur_col_num
            if cur_col_num != 0:
                if last_col_num == 0:
                    cur_key = j
            else:
                if cur_total_num != 0:
                    col_intensity[cur_key] = cur_total_num
                    cur_total_num = 0
            last_col_num = cur_col_num
        if cur_total_num != 0:
            col_intensity[cur_key] = cur_total_num

        self._pipet_pos[1] = max(col_intensity, key=col_intensity.get)   # type: ignore
        first_row = 0
        for i in range(self._roi_row_range[0], self._roi_row_range[1]):
            if self._pipet_img[i, self._pipet_pos[1]] == 255:
                first_row = i
                break
        for i in range(self._roi_row_range[1], self._roi_row_range[0], -1):
            if self._pipet_img[i, self._pipet_pos[1]] == 255:
                self._pipet_pos[0] = int((i + first_row) / 2)
                break
        return self._pipet_pos
        
    
    def get_carrying_pos(self, img: np.ndarray) -> List[int]:
        gray = img
        [cur_key,cur_total_sum,col_intensity] = carrying_for1(gray,self._roi_col_range[0],self._roi_col_range[1],self._roi_row_range[0],self._roi_row_range[1],self._carrying_ignor_thre,self._pipet_img)
        if cur_total_sum != 0:
            cur_total_sum = 0
        self._carrying_pos[1] = cur_key[col_intensity.index(max(col_intensity))]
        min_value = 256
        min_value_idx = 0
        [min_value,min_value_idx] = carrying_for2(gray,self._roi_row_range[0],self._roi_row_range[1],min_value,self._carrying_pos[1])

        first_row = None
        diff_value = 5  # 这个色差也可以作为一个可调参数
        # for3
        first_row = carrying_for3(gray,min_value_idx,diff_value,min_value,self._carrying_pos[1],self._roi_row_range[0])
        ## for4
        self._carrying_pos[0] = carrying_for4(gray,min_value_idx,first_row,diff_value,min_value,self._roi_row_range[1],self._carrying_pos[1])
        # print("self._carrying_pos[0]",self._carrying_pos[0])

        return self._carrying_pos

    ## use a pre-catched img to get pippet pos
    def find_init(self):
        img = self.camera_acquire()
        self._resized_size = (600, 800)
        pipet_img_resized = cv2.resize(img, (self._resized_size[1], self._resized_size[0]))
        rows = img.shape[0]
        cols = img.shape[1]
        row_rate = rows/self._resized_size[0]
        col_rate = cols/self._resized_size[1]
        self.binaryzation_thre = 40
        self.roi_center = [350, 270]
        self.roi_box = [130, 400]
        pipet_pos = self.get_pipet_pos(pipet_img_resized)
        real_pipet_pos = [0, 0]
        real_pipet_pos[0] = int(pipet_pos[0]*row_rate)
        real_pipet_pos[1] = int(pipet_pos[1]*col_rate)

    def camera_init(self):
        self._device_manager = gx.DeviceManager()
        dev_num, dev_info_list = self._device_manager.update_device_list()
        if dev_num == 0:
            sys.exit(1)
        strSN = dev_info_list[0].get("sn")
        self._cam = self._device_manager.open_device_by_sn(strSN)
        
        # set exposure
        self._cam.ExposureTime.set(10000)

        # set gain
        self._cam.Gain.set(10.0)

        # send software trigger command
        self._cam.TriggerMode.set(gx.GxSwitchEntry.ON)
        self._cam.TriggerSource.set(gx.GxTriggerSourceEntry.SOFTWARE)
        self._cam.stream_on()
        return self._cam

    def camera_acquire(self):
        self._cam.TriggerSoftware.send_command()
        raw_image = self._cam.data_stream[0].get_image()
        numpy_image = raw_image.get_numpy_array()
        if numpy_image is None:
            return None
        else:
            if self._save_pic:
                image = Image.fromarray(numpy_image, 'L')
                image.save("pics/data.png")
                self._save_pic = False

            img = cv2.resize(numpy_image, (self._resized_size[1], self._resized_size[0]))
            self.roi_center = [int(self._resized_size[0] * 1 / 2), int(self._resized_size[1]*1 / 2)]
            self.roi_box = [int(self._resized_size[0]*2/3), int(self._resized_size[1]*2/3)]
        return img
    
    def handle_camera_cmd(self,req):
        self._save_pic = True
        return camera_cmdResponse(True)

    def ros_init(self):
        rospy.init_node("cameraPub")
        rospy.loginfo("Starting camera node as cameraPub.")
        
        camera_pub = rospy.Publisher("camera_pub", pospub, queue_size=10)
        pub_msgs = pospub()
        pub_msgs.pippet = (self._pipet_pos[0],self._pipet_pos[1])
        camera_srv = rospy.Service('camera_cmd', camera_cmd, self.handle_camera_cmd)
        print("Ready to receive camera cmd.")

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            start_time = time.time() 
            img = self.camera_acquire()
            cal_pipet_pos_time = time.time() - start_time
            print("running time Duration is {}".format(cal_pipet_pos_time))
            if img is None:
                rospy.loginfo("camera recieve img is None. Please check your device")
            else:
                carrying_pos = self.get_carrying_pos(img)
                if carrying_pos[0] is not None:
                    pub_msgs.carrying = (carrying_pos[0],carrying_pos[1])
                
                camera_pub.publish(pub_msgs)
            rate.sleep()
        
        self._cam.close_device()


if __name__ == "__main__":
    
    fp = FindPosition()
    fp.ros_init()
