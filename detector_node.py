import cv2
import math
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from meter_detector import MeterDetect
from gauge_detector import GaugeDetect

class FinalProcess():
  def __init__(self, is_debug=False):
    self.is_debug = is_debug
    self.pub = rospy.Publisher("FinalResImg", Image, queue_size=1)
    self.textresimg = None
    self.resultimg = None
    self.gauge_unit = 0
    self.bridge = CvBridge()


  def textresimg_cb(self, img):
    rospy.loginfo("Get textresimg!")
    self.textresimg = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
  

  def detectres_cb(self, data):
    id_meter_temp = "temperature"
    id_meter_pres = "pressure"
    id_number = "digital"
    id_gauge = "level"
    # Check text result image
    if not self.textresimg is None:
      output_img = self.textresimg.copy()
      rospy.loginfo("Get TextResImg!")
      # Get Json data from message
      try:
        area_json = json.loads(data.data)
        area_text_list = area_json.keys()
      # Set empty list if no data
      except:
        area_text_list = []
      # Process data
      for text in area_text_list:
        lt_x = area_json[text]['lt_x']
        lt_y = area_json[text]['lt_y']
        rb_x = area_json[text]['rb_x']
        rb_y = area_json[text]['rb_y']
        # Temperature Meter:
        if text == id_meter_temp:
          meterD = MeterDetect(output_img[lt_y:rb_y,lt_x:rb_x,:], is_debug=self.is_debug)
          target_angle= meterD.execute()
          output_str = "Pointer_Temperature: %.2f" % target_angle
          output_img = cv2.rectangle(output_img, (lt_x,lt_y), (rb_x, rb_y), (0,0,255), 2)
          output_img = cv2.putText(output_img, output_str,(lt_x,rb_y+30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)

        elif text == id_meter_pres:
          meterD = MeterDetect(output_img[lt_y:rb_y,lt_x:rb_x,:], is_debug=self.is_debug, is_pressure=True)
          target_angle= meterD.execute()
          output_str = "Pointer_Pressure: %.2f" % target_angle
          output_img = cv2.rectangle(output_img, (lt_x,lt_y), (rb_x, rb_y), (0,0,255), 2)
          output_img = cv2.putText(output_img, output_str,(lt_x,rb_y+30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)

        elif text == id_number:
          output_str = "Number_Temperature"
          output_img = cv2.rectangle(output_img, (lt_x,lt_y), (rb_x, rb_y), (0,0,255), 2)
          output_img = cv2.putText(output_img, output_str,(lt_x,rb_y+30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)

        elif text == id_gauge:
          output_img = cv2.rectangle(output_img, (lt_x,lt_y), (rb_x, rb_y), (0,0,255), 2)
          if self.gauge_unit == 0:
            output_str = "Liquid_Gauge"
          else:
            gaugeD = GaugeDetect(output_img[lt_y:rb_y,lt_x:rb_x,:], self.gauge_unit, is_debug=self.is_debug)
            target_result = gaugeD.execute()
            output_str = "Liquid_Gauge: %.2f" % target_result
          output_img = cv2.putText(output_img, output_str,(lt_x,rb_y-30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2,cv2.LINE_AA)

      # image_msg = self.bridge.cv2_to_imgmsg(output_img, encoding="passthrough")
      # image_msg.header.stamp = rospy.Time.now()
      # image_msg.header.frame_id = "camera_frame"
      # self.pub.publish(image_msg)
      self.resultimg = output_img
    # No get Image
    else:
      rospy.loginfo("No FinalResImg!")


  def areainfo_cb(self, data):
    rospy.loginfo("Get Areainfo!")
    list10 = ["10","nu","no","eo","1o","1o0","1oo","100"]
    list20 = ["20","ro","2o0","2oo","roo","ro0","200"]
    list30 = ["30","3oo","3o0","300"]
    h10, h20, h30, unit = 0, 0, 0, 0

    try:
      area_json = json.loads(data.data)
      area_text_list = area_json.keys()
    except:
      area_text_list = [ ]
    for text in area_text_list:
      lt_y = area_json[text]['Box']['pt0']['y']
      if text in list10:
        h10 = lt_y
      if text in list20:
        h20 = lt_y
      if text in list30:
        h30 = lt_y

    if h10 == 0:
      if h20>0 and h30>0:
        unit = h20-h30
    else:
      if h20>0:
        unit = h10-h20
      elif h30>0:
        unit = (h10-h30)/2.0
    if unit>0:
      self.gauge_unit = 10.0/unit
    else:
      self.gauge_unit = 0
    

  def timer_cb(self, te):
    # Publish image result
    if not self.resultimg is None:
      image_msg = self.bridge.cv2_to_imgmsg(self.resultimg, encoding="bgr8")
      image_msg.header.stamp = rospy.Time.now()
      image_msg.header.frame_id = "camera_frame"
      self.pub.publish(image_msg)
      rospy.loginfo("ResImg Published!")
      self.resultimg = None
    # Publish text directly
    elif not self.textresimg is None:
      image_msg = self.bridge.cv2_to_imgmsg(self.textresimg, encoding="bgr8")
      image_msg.header.stamp = rospy.Time.now()
      image_msg.header.frame_id = "camera_frame"
      self.pub.publish(image_msg)
      rospy.loginfo("ResImg Published!")


  # def final_pub(self):
  #   if not self.resultimg is None:
  #     image_msg = self.bridge.cv2_to_imgmsg(self.resultimg, encoding="bgr8")
  #     image_msg.header.stamp = rospy.Time.now()
  #     image_msg.header.frame_id = "camera_frame"
  #     self.pub.publish(image_msg)
  #     rospy.loginfo("ResImg Published!")

def main():
  print("==========================================================")
  fp = FinalProcess(is_debug=False)
  rospy.Subscriber("/TextDetect/detectResImg", Image, fp.textresimg_cb)
  rospy.Subscriber("/areaInfo", String, fp.areainfo_cb)
  rospy.Subscriber("/ObjectDetect/detectRes", String, fp.detectres_cb)
  rospy.Timer(rospy.Duration(0.2), fp.timer_cb)
  # while not rospy.is_shutdown():
  #   fp.final_pub()
  rospy.spin()

if __name__ =='__main__':
  rospy.init_node('detector_node')
  main()