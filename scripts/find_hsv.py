import cv2
import numpy as np

class FindHSV:

    def __init__(self):
        
        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()
            image_ori = frame.copy()

            self.h, self.w = frame.shape[:2] 
            roi = [(int(self.w/2 * 0.85), int(self.h/2 * 0.8)), (int(self.w/2 * 1.15), int(self.h/2 * 1.2))]
            cv2.rectangle(frame, roi[0], roi[1], (0,0,255), 3)


            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            if key == ord("f"):

                self.image = image_ori
                break
            cv2.imshow("Place object inside the circle", frame)
    

        self.image_ori = self.image.copy()
        self.h, self.w = self.image.shape[:2] 


        self.do_hsv()     

    def find_circles(self, image):

        
        contours, _  = cv2.findContours(image, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            if contours.__len__() == 1:
                c = contours[0]
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                # print(contours.__len__(),x, y, radius, cv2.contourArea(c), c)
                return x, y, radius, np.sum(image == 255), c

            elif contours.__len__() > 1 and contours.__len__() < 5:
                c = max(contours, key=cv2.contourArea)

                ((x, y), radius) = cv2.minEnclosingCircle(c)
                # print(contours[0])
                # print(contours.__len__(),x, y, radius, cv2.contourArea(c), c)
                return x, y, radius, np.sum(image == 255), c
        
        return False, False, False, False, False


    def do_hsv(self):

        best_val = 0
        best_i = 0

        samples = 5000

        k = 4
        kernel = np.ones((k,k),np.uint8)

        h_low = np.random.rand(samples)*180
        s_low = np.random.rand(samples)*255
        v_low = np.random.rand(samples)*255
        h_high = np.random.rand(samples)*180
        s_high = np.random.rand(samples)*255
        v_high = np.random.rand(samples)*255

        for i in range(samples):

            if h_low[i] > h_high[i]:
                aux = h_low[i]
                h_low[i] = h_high[i]
                h_high[i] = aux

            if s_low[i] > s_high[i]:
                aux = s_low[i]
                s_low[i] = s_high[i]
                s_high[i] = aux

            if v_low[i] > v_high[i]:
                aux = v_low[i]
                v_low[i] = v_high[i]
                v_high[i] = aux

        for i in range(samples):

            ORANGE_MIN = np.array([h_low[i], s_low[i], v_low[i]],np.uint8)
            ORANGE_MAX = np.array([h_high[i], s_high[i], v_high[i]],np.uint8)
            hsv_in = cv2.cvtColor(self.image_ori,cv2.COLOR_BGR2HSV)


            threshed_in = cv2.inRange(hsv_in, ORANGE_MIN, ORANGE_MAX)

            threshed_in = cv2.morphologyEx(threshed_in, cv2.MORPH_CLOSE, kernel)


            x, y, radius, area, c = self.find_circles(threshed_in)


            if x:
                if x < self.w/2 * 1.15 and x > self.w/2 * 0.85  and y < self.h/2 * 1.2 and y > self.h/2 * 0.8 and radius <  self.w/2 * 0.2:
                    if area > best_val:
                        best_val = area
                        best_i = i


        roi = [(int(self.w/2 * 0.8), int(self.h/2 * 0.8)), (int(self.w/2 * 1.2), int(self.h/2 * 1.2))]
        min_hsv = [h_low[best_i], s_low[best_i], v_low[best_i]]
        max_hsv = [h_high[best_i], s_high[best_i], v_high[best_i]]
        ORANGE_MIN = np.array([h_low[best_i], s_low[best_i], v_low[best_i]],np.uint8)
        ORANGE_MAX = np.array([h_high[best_i], s_high[best_i], v_high[best_i]],np.uint8)



        hsv_image = cv2.cvtColor(self.image,cv2.COLOR_BGR2HSV)

        threshed_image = cv2.inRange(hsv_image, ORANGE_MIN, ORANGE_MAX)


        threshed_image = cv2.morphologyEx(threshed_image, cv2.MORPH_CLOSE, kernel)

        print([h_low[best_i], s_low[best_i], v_low[best_i]])
        print([h_high[best_i], s_high[best_i], v_high[best_i]])
        

        cv2.imshow('thr',threshed_image)
        cv2.imshow('hsv_image',hsv_image)


        contours, _  = cv2.findContours(threshed_image, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)

            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            try:

                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                cv2.circle(self.image, center, 5, (0, 0, 255), -1)
                cv2.circle(self.image, center, int(radius), (0, 255, 0), 2)
            except:
                pass

        cv2.rectangle(self.image, roi[0], roi[1], (255,0,0))
        cv2.imshow('circles', self.image)
        with open("hsv.txt", "w") as f:
            f.write(str(min_hsv) + "\n")
            f.write(str(max_hsv))
        cv2.waitKey(0)
      




mytool = FindHSV()

    

cv2.destroyAllWindows()

