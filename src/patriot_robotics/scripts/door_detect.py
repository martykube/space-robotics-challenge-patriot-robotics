#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point

class Door_Detector:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('multisense/camera/left/image_raw',
						  Image, self.image_callback)
		self.door_centroid_pub = rospy.Publisher('DoorCentroid', Point, queue_size=10)

	def image_callback(self, msg):
		# get the image retrieved from the message
		image = self.bridge.imgmsg_to_cv2(msg)

		# convert image to gray scale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# use Canny edge detection to get edges
		edges = cv2.Canny(gray, 100, 200)

		# find all the contours
		(contours, _) = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		# sort from largest contour to smallest
		contours = sorted(contours, key = cv2.contourArea, reverse = True)[:20]
		screenCnt = None

		# iterate through each contour and identify rectangles
		for c in contours:
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.10 * peri, True)

			# presumably, rectangle has 4 corners
			if len(approx) == 4:
				screenCnt = approx
			
				#cv2.drawContours(image, c, -1, (255, 0, 0), 3)

				# drawing a bounding recrtangle around the contour
				x, y, w, h = cv2.boundingRect(c)
				cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 10)

				# get the center of mass of the contour
				M = cv2.moments(c)

				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])

				# publish the centroid
				centroid = Point(cx, cy, 0)
				self.door_centroid_pub.publish(centroid)

				# draw the center of mass in the image
				cv2.circle(image, (cx, cy), 5, (0, 255, 0), 10)

				break

		# display the new image
		plt.subplot(121), plt.imshow(image)
		plt.title('Original Image'), plt.xticks([]), plt.yticks([])

		plt.show()


rospy.init_node('door_detector')
door_detector = Door_Detector()
rospy.spin()
