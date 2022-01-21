import cv2
import time

# open camera
<<<<<<< HEAD
cap = cv2.VideoCapture("/dev/video0")
cap.set(cv2.CAP_PROP_POS_FRAMES, 10)
=======
cap = cv2.VideoCapture(0)
>>>>>>> dev

# # set dimensions
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
# # cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

<<<<<<< HEAD
# print( cap.isOpened() )
# time.sleep(2)
# take frame
while True:
    ret, frame = cap.read()
    # print(frame,ret)
    # write frame to file
    cv2.imshow('im22age.jpg', frame)
    if cv2.waitKey(1) == ord('k'):
        break
=======

print( cap.isOpened() )
# take frame
ret, frame = cap.read()
print(frame,ret)
# write frame to file
cv2.imwrite('image.jpg', frame)
>>>>>>> dev
# release camera
cap.release()


