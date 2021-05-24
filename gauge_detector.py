import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

class GaugeDetect():
  def __init__(self, target_img, unit_length, is_debug=False):
    self.is_debug = is_debug
    self.target = cv2.cvtColor(target_img, cv2.COLOR_BGR2RGB)
    self.target_hsv = cv2.cvtColor(target_img, cv2.COLOR_BGR2HSV)
    self.unit_length = unit_length


  def execute(self):
    # Check Unit Length
    if self.unit_length==0:
      return 0
    red_mask = self.__red_region_detection()
    # ubound, lbound = self.__calc_mask_bound(red_mask)
    ubound, lbound = self.__calc_mask_bound_continuous(red_mask)
    guage_result = (ubound - lbound) * self.unit_length
    # Debug
    if self.is_debug:
      h,w = red_mask.shape
      debug_img = self.target.copy()
      debug_img[np.where(red_mask==0)] = 0
      cv2.line(debug_img, (0,int(ubound)),(w,int(ubound)),(255,0,0),5)
      cv2.line(debug_img, (0,int(lbound)),(w,int(lbound)),(255,0,0),5)
      plt.imshow(debug_img)
      plt.show()
    return guage_result


  def __calc_mask_bound(self, mask):
    h,w = mask.shape
    upper_bound = []
    lower_bound = []
    # Search in 2/5 to 3/5 region
    for j in np.arange(int(w/5))+int(w/5*2):
      max_i, min_i = 0, h
      for i in range(h):
        if mask[i][j] == 255:
          max_i = i if i > max_i else max_i
          min_i = i if i < min_i else min_i
      upper_bound.append(max_i)
      lower_bound.append(min_i)
    ubound = np.average(upper_bound)
    lbound = np.average(lower_bound)
    return ubound, lbound


  def __calc_mask_bound_continuous(self, mask):
    h, w = mask.shape
    regions = []
    j = int(w/2)
    region_flag = 0
    region_start = 0
    for i in range(h):
      pixel = mask[i][j]
      if region_flag:
        if pixel:
          continue
        else:
          regions.append([i, region_start])
          region_flag = 0
      else:
        if pixel:
          region_flag = 1
          region_start = i
        else:
          continue
    regions.sort(reverse=True, key=lambda x:x[0]-x[1])
    return regions[0]


  def __red_region_detection(self):
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(self.target_hsv, lower_red, upper_red)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(self.target_hsv, lower_red, upper_red)
    return mask0+mask1