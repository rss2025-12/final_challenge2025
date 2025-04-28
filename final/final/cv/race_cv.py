import cv2
import numpy as np

def show_video(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)

	### Continuous streaming ###
	if cv2.waitKey(1) & 0xFF == ord('q'):
		cv2.destroyAllWindows()

out = None

def record_video(img):
    global out

    if img is None:
        if out is not None:
            out.release()
        return

    height, width, _ = img.shape
    fps = 30
    frame_size = (width, height)

    if out is None:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('src/final/vids/run1.mp4', fourcc, fps, frame_size)

    out.write(img)

def lane_following(img):
    pass
