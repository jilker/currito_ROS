import cv2
import time

# open camera
cap = cv2.VideoCapture(0)

# # set dimensions
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
# # cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

print( cap.isOpened() )
# take frame
ret, frame = cap.read()
print(frame,ret)
# write frame to file
cv2.imwrite('image.jpg', frame)
# release camera
cap.release()
