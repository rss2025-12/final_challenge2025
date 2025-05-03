import cv2
import imutils
import numpy as np
import os
import uuid

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img, output_dir="output"):
	"""
	Helper function to print out images, for debugging.
	Press any key to continue.
	"""
    # Ensure the output directory exists
	if not os.path.exists(output_dir):
		os.makedirs(output_dir)

	#generate a unique filename
	unique_filename = os.path.join(output_dir, f"output_{uuid.uuid4().hex}.jpg")

	winname = "Image"
	# cv2.namedWindow(winname)        # Create a named window
	# cv2.moveWindow(winname, 40,30)  # Move it to (40,30)
	#cv2.imshow(winname, img)- replaced this with the following line to see the image
	cv2.imwrite(unique_filename, img)
	print(f"Image saved to {unique_filename}")

	# cv2.waitKey()
	# cv2.destroyAllWindows()

def cd_sift_ransac(img, template):
	"""
	Implement the cone detection using SIFT + RANSAC algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	# Minimum number of matching features
	MIN_MATCH = 10 # Adjust this value as needed
	# Create SIFT
	sift = cv2.xfeatures2d.SIFT_create()

	# Compute SIFT on template and test image
	kp1, des1 = sift.detectAndCompute(template,None)
	kp2, des2 = sift.detectAndCompute(img,None)

	# Find matches
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2,k=2)

	# Find and store good matches
	good = []
	for m,n in matches:
		if m.distance < 0.75*n.distance:
			good.append(m)

	# If enough good matches, find bounding box
	if len(good) > MIN_MATCH:
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

		# Create mask
		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
		matchesMask = mask.ravel().tolist()

		h, w = template.shape
		pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

		########## YOUR CODE STARTS HERE ##########

		#transform the template corners ro the test image using the homography matrix
		new_pts = cv2.perspectiveTransform(pts, M)

		#Get the min and max x and y coordinates of the transformed points
		x_coords = [pt[0][0] for pt in new_pts]
		y_coords = [pt[0][1] for pt in new_pts]

		x_min = int(min(x_coords))
		x_max = int(max(x_coords))
		y_min = int(min(y_coords))
		y_max = int(max(y_coords))

		# #Andy's Code: 
		# good_pts = [pts[i] for i in range(len(pts)) if matchesMask[i] == 1]
		# new_pts = cv2.perspectiveTransform(np.float32(good_pts), M)

		# x_min = int(np.min(new_pts[:, 0, 0]))
		# y_min = int(np.min(new_pts[:, 0, 1]))
		# x_max = int(np.max(new_pts[:, 0, 0]))
		# y_max = int(np.max(new_pts[:, 0, 1]))

		cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
		image_print(img)

		########### YOUR CODE ENDS HERE ###########

		# Return bounding box
		return ((x_min, y_min), (x_max, y_max))
	else:

		print(f"[SIFT] not enough matches; matches: ", len(good))

		# Return bounding box of area 0 if no match found
		return ((0,0), (0,0))

def cd_template_matching(img, template):
	"""
	Implement the cone detection using template matching algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	#Compute Canny edges for the template
	template_canny = cv2.Canny(template, 50, 200)

	# Perform Canny Edge detection on test image
	grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img_canny = cv2.Canny(grey_img, 50, 200)

	# Get dimensions of template
	(img_height, img_width) = img_canny.shape[:2]

	# Keep track of best-fit match
	best_match = None
	best_match_score = -np.inf # Added: Initialize to negative infinity low score

	# Loop over different scales of image
	for scale in np.linspace(1.5, .5, 50):
		# Resize the image
		resized_template = imutils.resize(template_canny, width = int(template_canny.shape[1] * scale))
		(h,w) = resized_template.shape[:2]
		# Check to see if test image is now smaller than template image
		if resized_template.shape[0] > img_height or resized_template.shape[1] > img_width:
			continue

		########## YOUR CODE STARTS HERE ##########
		# Use OpenCV template matching functions to find the best match # across template scales.

		result = cv2.matchTemplate(img_canny, resized_template, cv2.TM_CCOEFF_NORMED)
		(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result) #get best match location and its score

		# If the current match is better than the best match, update the best match
		if maxVal > best_match_score:
			best_match = (maxLoc, (w,h), scale)
			best_match_score = maxVal

	if best_match is not None:
		(top_left, (w, h), scale) = best_match
		bounding_box = ((top_left[0], top_left[1]), (top_left[0] + w, top_left[1] + h))

		#dispalying the image with the bounding box
		img_with_box = img.copy()
		cv2.rectangle(img_with_box, bounding_box[0], bounding_box[1], (0, 255, 0), 2)
		image_print(img_with_box)

		
		# Remember to resize the bounding box using the highest scoring scale
		# x1,y1 pixel will be accurate, but x2,y2 needs to be correctly scaled
	else: 
		bounding_box = ((0,0),(0,0))
		########### YOUR CODE ENDS HERE ###########

	return bounding_box
