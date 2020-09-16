"""查找图片中的吸管和载杆位置
"""

import time
from typing import List, Optional
import cv2
import os
from matplotlib import pyplot as plt
import numpy as np
from collections import defaultdict

## ros
# from 3axis_platform.srv import camera_cmd,camera_cmdResponse
from micro_manipulate.srv import *
import rospy


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
        pipet_img(Optional[np.ndarray]): ROI 里二值化后的吸管图，调用 `get_pipet_pos()` 后生成，可以通过显示这个图来判断玻璃管识别的效果
        pipet_pos(List[int]): 玻璃管的位置, 调用 `get_pipet_pos()` 后生成[row, col]
        carrying_ignor_thre(int): 忽略载杆图像像素的灰度阈值。灰度值大于该值认为是白色背景(255)
        carrying_pos(List[int]): 载杆的位置，调用 `get_carrying_pos()` 后生成[row, col]
    """

    def __init__(self) -> None:
        self._roi_center: List[int] = [0, 0]
        self._roi_box: List[int] = [0, 0]
        self._binaryzation_thre: int = 5
        self._fuzzy_len: int = 10
        self._pipet_img: Optional[np.ndarray] = None
        self._pipet_pos: List[int] = [0, 0]
        self._carrying_ignor_thre: int = 70
        self._carrying_pos: List[int] = [0, 0]

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
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
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
        self._pipet_pos[1] = max(col_intensity, key=col_intensity.get)
        first_row = None
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
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        col_intensity = defaultdict(int)
        last_col_sum = 0
        cur_total_sum = 0
        cur_key = None
        for j in range(self._roi_col_range[0], self._roi_col_range[1]):
            cur_col_sum = 0
            for i in range(self._roi_row_range[0], self._roi_row_range[1]):
                if gray[i, j] > self._carrying_ignor_thre or self._pipet_img[i, j] == 255:
                    continue
                cur_col_sum += gray[i, j] >> 4
            cur_total_sum += cur_col_sum
            if cur_col_sum != 0:
                if last_col_sum == 0:
                    cur_key = j
            else:
                if cur_total_sum != 0:
                    col_intensity[cur_key] = cur_total_sum
                    cur_total_sum = 0
            last_col_sum = cur_col_sum
        if cur_total_sum != 0:
            col_intensity[cur_key] = cur_total_sum
            cur_total_sum = 0
        self._carrying_pos[1] = max(col_intensity, key=col_intensity.get)
        min_value = 256
        min_value_idx = 0
        for i in range(self._roi_row_range[0], self._roi_row_range[1]):
            if gray[i, self._carrying_pos[1]] < min_value:
                min_value = gray[i, self._carrying_pos[1]]
                min_value_idx = i
        first_row = None
        diff_value = 5  # 这个色差也可以作为一个可调参数
        for i in range(min_value_idx, self._roi_row_range[0], -1):
            if gray[i, self._carrying_pos[1]] > min_value + diff_value:
                first_row = i
                break
        for i in range(min_value_idx, self._roi_row_range[1]):
            if gray[i, self._carrying_pos[1]] > min_value + diff_value:
                self._carrying_pos[0] = int((i + first_row) / 2)
                break
        return self._carrying_pos
    
    def position_handle(self,req):
        path = req.path
        # carrying_img_path = os.path.join(os.getcwd(), img_dir, str(6) + ".jpg")
        carrying_img = cv2.imread(path)
        # 注意要重新设置 ROI
        start_time = time.time()
        self.get_carrying_pos(carrying_img)
        res = camera_servoResponse()
        res.pippet = tuple(self._pipet_pos)
        res.carrying = tuple(self.carrying_pos)
        cal_carrying_pos_time = time.time() - start_time
        print("caculate carrying position time: {0}".format(cal_carrying_pos_time))
        return res

    def position_get_server(self):
        rospy.init_node('pixel_get_server')
        srv = rospy.Service('pixel_get',camera_servo,self.position_handle)
        print("Ready to get image pixel.")
        rospy.spin()

if __name__ == "__main__":
    # 获得图片中玻璃管的位置
    img_dir = "materials/imgs"
    pipet_img_path = os.path.join(os.getcwd(), img_dir, "initial" + ".jpg")
    print(pipet_img_path)
    pipet_img = cv2.imread(pipet_img_path)
    rows = pipet_img.shape[0]
    cols = pipet_img.shape[1]

    start_time = time.time()
    fp = FindPosition()
    fp.roi_center = [350, 270]
    fp.roi_box = [130, 400]
    pipet_pos = fp.get_pipet_pos(pipet_img)
    cal_pipet_pos_time = time.time() - start_time
    print("caculate pipet position time: {0}".format(cal_pipet_pos_time))

    # show image
    # for i in range(rows):
    #     pipet_img[i, pipet_pos[1]-1] = [255, 0, 0]
    #     pipet_img[i, pipet_pos[1]] = [255, 0, 0]
    #     pipet_img[i, pipet_pos[1] + 1] = [255, 0, 0]
    # for j in range(cols):
    #     pipet_img[pipet_pos[0]-1, j] = [255, 0, 0]
    #     pipet_img[pipet_pos[0], j] = [255, 0, 0]
    #     pipet_img[pipet_pos[0]+1, j] = [255, 0, 0]
    # plt.imshow(pipet_img)
    # plt.figure()
    # plt.imshow(fp.pipet_img, cmap='gray')

    # 计算其他图片载杆位置
    carrying_img_path = pipet_img_path
    # carrying_img_path = os.path.join(os.getcwd(), img_dir, str(6) + ".jpg")
    carrying_img = cv2.imread(carrying_img_path)
    # 注意要重新设置 ROI
    fp.roi_center = [rows / 2, cols / 2]
    fp.roi_box = [int(rows*2/3), int(cols*2/3)]
    start_time = time.time()
    carrying_pos = fp.get_carrying_pos(carrying_img)
    cal_carrying_pos_time = time.time() - start_time
    print("caculate carrying position time: {0}".format(cal_carrying_pos_time))
    # print("carrying position", carrying_pos[0], carrying_pos[1])
    # print("pipet position", pipet_pos[0], pipet_pos[1])
    # show image
    # plt.figure()
    # for i in range(rows):
    #     carrying_img[i, pipet_pos[1]-1] = [255, 0, 0]
    #     carrying_img[i, pipet_pos[1]] = [255, 0, 0]
    #     carrying_img[i, pipet_pos[1] + 1] = [255, 0, 0]
    # for j in range(cols):
    #     carrying_img[pipet_pos[0]-1, j] = [255, 0, 0]
    #     carrying_img[pipet_pos[0], j] = [255, 0, 0]
    #     carrying_img[pipet_pos[0] + 1, j] = [255, 0, 0]

    # for i in range(rows):
    #     carrying_img[i, carrying_pos[1]-1] = [0, 255,  0]
    #     carrying_img[i, carrying_pos[1]] = [0, 255,  0]
    #     carrying_img[i, carrying_pos[1] + 1] = [0, 255,  0]
    # for j in range(cols):
    #     carrying_img[carrying_pos[0]-1, j] = [0, 255,  0]
    #     carrying_img[carrying_pos[0], j] = [0, 255,  0]
    #     carrying_img[carrying_pos[0]+1, j] = [0, 255,  0]
    # plt.imshow(carrying_img)
    # plt.show()
    fp.position_get_server()

