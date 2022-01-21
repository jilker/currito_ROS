import cv2
import numpy as np

def callback(x):
    pass

cap = cv2.VideoCapture("/dev/video0")
cv2.namedWindow('image')

ilowH = 160
ihighH = 175

ilowS = 100
ihighS = 255
ilowV = 20
ihighV = 255

# create trackbars for color change
cv2.createTrackbar('lowH','image',ilowH,179,callback)
cv2.createTrackbar('highH','image',ihighH,179,callback)

cv2.createTrackbar('lowS','image',ilowS,255,callback)
cv2.createTrackbar('highS','image',ihighS,255,callback)

cv2.createTrackbar('lowV','image',ilowV,255,callback)
cv2.createTrackbar('highV','image',ihighV,255,callback)

elowH = 0
ehighH = 10
elowS = 100
ehighS = 255
elowV = 10
ehighV = 255

# create trackbars for color change
cv2.createTrackbar('elowH','image',elowH,179,callback)
cv2.createTrackbar('ehighH','image',ehighH,179,callback)

cv2.createTrackbar('elowS','image',elowS,255,callback)
cv2.createTrackbar('ehighS','image',ehighS,255,callback)

cv2.createTrackbar('elowV','image',elowV,255,callback)
cv2.createTrackbar('ehighV','image',ehighV,255,callback)

while True:
    # grab the frame
    ret, frame = cap.read()

    # get trackbar positions
    ilowH = cv2.getTrackbarPos('lowH', 'image')
    ihighH = cv2.getTrackbarPos('highH', 'image')
    ilowS = cv2.getTrackbarPos('lowS', 'image')
    ihighS = cv2.getTrackbarPos('highS', 'image')
    ilowV = cv2.getTrackbarPos('lowV', 'image')
    ihighV = cv2.getTrackbarPos('highV', 'image')

    elowH = cv2.getTrackbarPos('elowH', 'image')
    ehighH = cv2.getTrackbarPos('ehighH', 'image')
    elowS = cv2.getTrackbarPos('elowS', 'image')
    ehighS = cv2.getTrackbarPos('ehighS', 'image')
    elowV = cv2.getTrackbarPos('elowV', 'image')
    ehighV = cv2.getTrackbarPos('ehighV', 'image')

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([ilowH, ilowS, ilowV])
    higher_hsv = np.array([ihighH, ihighS, ihighV])
    lower_hsv2 = np.array([elowH, elowS, elowV])
    higher_hsv2 = np.array([ehighH, ehighS, ehighV])
    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
    mask2 = cv2.inRange(hsv, lower_hsv2, higher_hsv2)

    mask = mask + mask2

    frame = cv2.bitwise_and(frame, frame, mask=mask)

    # show thresholded image
    cv2.imshow('image', frame)
    k = cv2.waitKey(10) & 0xFF # large wait time to remove freezing
    if k == 113 or k == 27:
        break


cv2.destroyAllWindows()
cap.release()