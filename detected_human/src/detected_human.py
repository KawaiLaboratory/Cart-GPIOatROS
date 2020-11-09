#!/usr/bin/env python
#-*- coding: utf-8 -*-

import roslib
import rospy
from keras.models import load_model
import numpy as np
import pandas as pd
from sensor_msgs.msg import LaserScan


model = load_model("detected_human_model.h5")
MAX_RANGE = 6
DATASIZE  = 725

def callback(data):
  l_data = pd.DataFrame(data.ranges).fillna(0)
  l_data = np.reshape(l_data["l"].values/MAX_RANGE)
  result = model.predict(l_data)

def detect_torso():
  rospy.init_node("detected_human")
  rospy.Subscriber("/scan", LaserScan, callback)
  rospy.spin()

