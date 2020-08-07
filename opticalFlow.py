import cv2
import numpy as np
import argparse
parser = argparse.ArgumentParser(description='This sample demonstrates Lucas-Kanade Optical Flow calculation. \
											  The example file can be downloaded from: \
											  https://www.bogotobogo.com/python/OpenCV_Python/images/mean_shift_tracking/slow_traffic_small.mp4')
parser.add_argument('image', type=str, help='path to image file')
args = parser.parse_args()
cap = cv2.VideoCapture(args.image)
# cap = cv2.VideoCapture(0)

# Create old frame
_, frame = cap.read()
old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Lucas kanade params
lk_params = dict(winSize = (15, 15),
maxLevel = 4,
criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Mouse function
def select_point(event, x, y, flags, params):
	global pointTL,pointBR, point_selected, old_points
	if event == cv2.EVENT_LBUTTONDOWN:
		pointTL = (x, y)
	if event == cv2.EVENT_LBUTTONUP:
		pointBR = (x,y)
		point_selected = True
		old_points = np.array([list(pointTL),list(pointBR)], dtype=np.float32)
		print(old_points)
cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", select_point)

point_selected = False
pointTL,pointBR = (),()
old_points = np.array([[]])


_, frame = cap.read()
cv2.imshow("Frame", frame)
cv2.waitKey(0)
while True:
	_, frame = cap.read()
	gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	if(cv2.waitKey(1) == "p"):
		cv2.waitKey(-1)



	if point_selected is True:
		#cv2.circle(frame, point, 5, (0, 0, 255), 2)
		cv2.rectangle(frame,pointTL,pointBR,(0,0,255),2)
		new_points, status, error = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, old_points, None, **lk_params)
		old_gray = gray_frame.copy()
		old_points = new_points
		pointTL1 = tuple(new_points[0])
		pointBR1 = tuple(new_points[1])
		cv2.rectangle(frame,pointTL1,pointBR1,(0,255,0),2)
	cv2.imshow("Frame", frame)
	

	key = cv2.waitKey(60)
	# if key == 27:
	# 	break

cap.release()
cv2.destroyAllWindows()
