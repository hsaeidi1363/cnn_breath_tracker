#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import tensorflow as tf
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
  try:
    # Currently, memory growth needs to be the same across GPUs
    for gpu in gpus:
      tf.config.experimental.set_memory_growth(gpu, True)
    logical_gpus = tf.config.experimental.list_logical_devices('GPU')
    print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
  except RuntimeError as e:
    # Memory growth must be set before GPUs have been initialized
    print(e)


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
	#print "pre-process started"
	global img_buff
	global t_h
	global t_h_1

	x_sum_all = img_buff[t_h_1]/float(255) 
	x_last2first = img_buff[t_h_1]/float(255) - img_buff[0]/float(255)

	for i in range(0,t_h_1):
		x_sum_all += img_buff[i]/float(255)
	img_combined = cv.merge((x_last2first, (x_sum_all-7)*2/float(t_h)))
	#print "pre-process completed"
	return img_combined

if __name__ == '__main__':

	rospy.init_node('motion_detector', anonymous = True)
	img_sub = rospy.Subscriber("/see_scope/nir/image_raw", Image, get_image) 
	motion_pub = rospy.Publisher('/see_scope/motion_out3', Image, queue_size = 1)
	xd_pub = rospy.Publisher('/see_scope/xd', Image, queue_size = 1)
	xlf_pub = rospy.Publisher('/see_scope/xlf', Image, queue_size = 1)
	xs_pub = rospy.Publisher('/see_scope/xs', Image, queue_size = 1)
	moving_pub = rospy.Publisher('/suture/cnn_output', Bool, queue_size = 1)

	rate = rospy.Rate(7)
	model = tf.keras.models.load_model('../saved_models/reduced_model')
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
			y = model.predict(img_cnn.reshape(1, 128, 128, 2), batch_size=1)
			if y[0][0] > y[0][1]: 
				str_msg = 'Moving'
				tissue_moving = True 
			else:
				str_msg = 'Stopped'	
				tissue_moving = False	
			image = cv.putText(img_raw, str_msg, org, font,  
                   fontScale, color, thickness, cv.LINE_AA) 
			img_msg = CvBridge().cv2_to_imgmsg(image)
			motion_pub.publish(img_msg)
			moving_pub.publish(tissue_moving)

			img_xlf = img_cnn[:,:,0] * 50
			print('min , max for xlf before uint8')
			print(np.amin(img_xlf))
			print(np.amax(img_xlf))
			img_xlf = img_xlf.astype(np.uint8)
			print('min , max for xlf After uint8')
			print(np.amin(img_xlf))
			print(np.amax(img_xlf))
			img_cnn_msg = CvBridge().cv2_to_imgmsg(img_xlf,encoding="mono8")
			xlf_pub.publish(img_cnn_msg)


			img_xs = img_cnn[:,:,1] * 50
			print('min , max for xs before uint8')
			print(np.amin(img_xs))
			print(np.amax(img_xs))
			img_xs = img_xs.astype(np.uint8)
			print('min , max for xs After uint8')
			print(np.amin(img_xs))
			print(np.amax(img_xs))
			img_cnn_msg = CvBridge().cv2_to_imgmsg(img_xs,encoding="mono8")
			xs_pub.publish(img_cnn_msg)
		rate.sleep()



