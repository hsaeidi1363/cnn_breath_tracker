#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import tensorflow as tf
from keras.models import Sequential
from tensorflow import keras
from tensorflow.keras import layers

import cv2 as cv
import numpy as np

# time horizon based on 7 Hz freq= > e.g. 14 means 2 seconds
global t_h
global t_h_1

t_h = 14
t_h_1 = t_h - 1

img_raw = np.zeros((1024, 1024, 1), dtype = "uint8")
img = np.zeros((128, 128, 1), dtype = "uint8")
img_ctr = 0

img_buff = np.zeros((t_h, 128, 128), dtype = "uint8")

def get_image(ros_img):
	global img
	global img_raw
	img_raw = CvBridge().imgmsg_to_cv2(ros_img, "mono8")
	img = cv.resize(img_raw, (img.shape[0] , img.shape[1]), interpolation = cv.INTER_LINEAR) 
	global img_ctr
	global img_buff
	global t_h
	global t_h_1
	if img_ctr < t_h:
#		print('making the initial buffer')
		img_buff[img_ctr] = img
	else:
#		print('updating the buffer')
		for i in range(0,t_h_1):
			img_buff[i] = img_buff[i+1]
		img_buff[t_h_1] = img  
	img_ctr += 1

#TODO: make sure all the steps of preprocessing is done here! like the image size, pixel ranges and etc

def pre_process():
	global img_buff
	global t_h
	global t_h_1
	x_d_all = img_buff[t_h_1]/float(255)
	x_sum_all = x_d_all 
	x_last2first = x_d_all - img_buff[0]/float(255)
	for i in range(0,t_h_1):
		x_d_all -= img_buff[i]/float(255)
		x_sum_all += img_buff[i]/float(255)
	img_combined = cv.merge((x_d_all, x_last2first, x_sum_all))
	return img_combined

if __name__ == '__main__':

	rospy.init_node('motion_detector', anonymous = True)
	img_sub = rospy.Subscriber("/see_scope/nir/image_raw", Image, get_image) 
	motion_pub = rospy.Publisher('/see_scope/motion_out3', Image, queue_size = 1)
	rate = rospy.Rate(7)
	model = tf.keras.models.load_model('../saved_models/my_model')
	print('loaded the model')
	model.summary()
	font = cv.FONT_HERSHEY_SIMPLEX 
  
	# org 
	org = (50, 50) 
  
	# fontScale 
	fontScale = 1
   
	# Blue color in BGR 
	color = (255, 0, 0) 
  
	# Line thickness of 2 px 
	thickness = 2
   	global t_h
	global t_h_1
	while not rospy.is_shutdown():
		if img_ctr >= t_h:
			#print('got enough images to start')
			img_cnn = pre_process()
			y = model.predict(img_cnn.reshape(1, 128, 128, 3), batch_size=1)
			if y[0][0] > y[0][1]: 
				str_msg = 'Moving'
			else:
				str_msg = 'Stopped'		
			image = cv.putText(img_raw, str_msg, org, font,  
                   fontScale, color, thickness, cv.LINE_AA) 
			img_msg = CvBridge().cv2_to_imgmsg(image)
			motion_pub.publish(img_msg)
			#print(y)
#		rospy.spinOnce() #causing issues?
		rate.sleep()
