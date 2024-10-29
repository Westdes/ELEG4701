import numpy as np
import cv2
import os
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)
from RRT_Test import testWithAnimation


print(BASE_DIR)

start = [0, 0]

obj_img = cv2.imread('obj_map.bmp')
# convert RGB image to gray scale image
obj_img = cv2.cvtColor(obj_img, cv2.COLOR_BGR2GRAY)
print(obj_img.shape)

# TODO: Set a target point, in the range of obj_image.shape[]
# And draw a line(trajectory) with at least 2 turns
end = [416, 400]
mid1 = [29, 277]  # Example intermediate point
mid2 = [226, 389]  # Another intermediate point

# Draw lines between the points
cv2.line(obj_img, tuple(start), tuple(mid1),
         (255), 2)  # Line from start to mid1
cv2.line(obj_img, tuple(mid1), tuple(mid2),
         (255), 2)   # Line from mid1 to mid2
cv2.line(obj_img, tuple(mid2), tuple(end), (255), 2)    # Line from mid2 to end

# Display the image with the trajectory
cv2.imshow('Trajectory', obj_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
##

start, end = np.array(start), np.array(end)  # ensure they are numpy data

testWithAnimation([obj_img, start, end])
