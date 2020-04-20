#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from skimage.measure import compare_ssim as ssim

import cv2 as cv
import numpy as np

#update according to the mono image?
img_buff = np.zeros((2, 1024, 1024), dtype = "uint8")

def get_image(ros_img):
#	img_raw = CvBridge().imgmsg_to_cv2(ros_img, "mono8")
#	img = cv.resize(img_raw, (img.shape[0] , img.shape[1]), interpolation = cv.INTER_LINEAR) 
	print("test")

if __name__ == '__main__':
	rospy.init_node('ssim_node', anonymous = True)
	img_sub = rospy.Subscriber("/suture/get_comparison_images", Image, get_image) 
	score_pub = rospy.Publisher('/see_scope/ssim_score', Float32, queue_size = 1)
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		(score, diff) = ssim(img_buff[0], img_buff[1],full=True, multichannel=True)
		score_pub.publish(score)
			#print(y)
		#rospy.spin()
		rate.sleep()
