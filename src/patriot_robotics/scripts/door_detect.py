#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point

class Door_Detector:
	template = []

	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('multisense/camera/left/image_raw',
						  Image, self.image_callback)
		self.door_centroid_pub = rospy.Publisher('DoorCentroid', Point, queue_size=10)
		self.door_match_pub = rospy.Publisher('DoorMatchImage', Image, queue_size=5)

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


				# draw the center of mass in the image
				#cv2.circle(image, (cx, cy), 5, (0, 255, 0), 10)

				# get a rectangular region of interest (roi) around the centroid
				roi1 = image[cy-60:cy+60, cx-32:cx+32]

				#plt.subplot(122), plt.imshow(roi1)

				#plt.title('Template Image'), plt.xticks([]), plt.yticks([])

				#plt.show()

				# store the first image found in the roi in the template
				# global
				if self.template == []:
					self.template = roi1

				break

		if self.template != []:
			# match template in image
			# convert template to grayscale
			gray_template = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)

			# Initiate SIFT detector
			orb = cv2.ORB()
	
			# find the keypoints and descriptors with SIFT
			kp1, des1 = orb.detectAndCompute(gray_template, None)
			kp2, des2 = orb.detectAndCompute(gray, None)
	
			# create BFMatcher object
			bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
	
			# Match descriptors
			matches = bf.match(des1, des2)
	
			# Sort them in the order of their distance
			matches = sorted(matches, key = lambda x:x.distance)
	
			# Draw first 10 matches
			img3, x2, y2 = self.drawMatches(gray_template, kp1, gray, kp2, matches[:1])

			# publish the centroid
			centroid = Point(x2, y2, 0)
			self.door_centroid_pub.publish(centroid)

			img3_msg = self.bridge.cv2_to_imgmsg(img3, "bgr8")
			self.door_match_pub.publish(img3_msg)
			
#			plt.imshow(img3), plt.show()

		# display the new image
#		plt.subplot(121), plt.imshow(image)
#		plt.title('Original Image'), plt.xticks([]), plt.yticks([])

#		plt.show()


	def drawMatches(self, img1, kp1, img2, kp2, matches):
		#Create a new output image that concatenates the two images together
		rows1 = img1.shape[0]
		cols1 = img1.shape[1]
		rows2 = img2.shape[0]
		cols2 = img2.shape[1]

		out = numpy.zeros((max([rows1, rows2]), cols1 + cols2, 3), dtype = 'uint8')

		# Place the first image to the left
		out[:rows1, :cols1] = numpy.dstack([img1, img1, img1])

		# Place the next image to the right of it
		out[:rows2, cols1:] = numpy.dstack([img2, img2, img2])

		# For each pair of points we have between both images
		# draw circles, then connect a line between them
		for mat in matches:
			# Get the matching keypoints for each of the images
			img1_idx = mat.queryIdx
			img2_idx = mat.trainIdx

			# x - columns
			# y - rows
			(x1, y1) = kp1[img1_idx].pt
			(x2, y2) = kp2[img2_idx].pt

			# Draw a small circle at both co-ordinates
			# radius 4
			# colour blue
			# thickness = 1
			cv2.circle(out, (int(x1), int(y1)), 4, (255, 0, 0), 1)
			cv2.circle(out, (int(x2)+cols1, int(y2)), 4, (255, 0, 0), 1)

			# Draw a line in between the two points
			# thickness = 1
			# colour blue
			cv2.line(out, (int(x1), int(y1)), (int(x2) + cols1, int(y2)), (255, 0, 0), 1)

		return out, x2, y2

rospy.init_node('door_detector')
door_detector = Door_Detector()
rospy.spin()
