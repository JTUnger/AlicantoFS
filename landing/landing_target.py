import cv2
import numpy as np

def contour_target(image):
    blurred = cv2.GaussianBlur(image, (3, 3), 0)
    thresh = cv2.threshold(blurred, 120, 250,cv2.THRESH_BINARY)[1]
    canny = cv2.Canny(thresh, 150, 255, 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    cnts = cv2.findContours(dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    contours = []

    threshold_min_area = 110
    threshold_max_area = 120

    for c in cnts:
        area = cv2.contourArea(c)
        if area > threshold_min_area and area < threshold_max_area:
            cv2.drawContours(image,[c], 0, (0,255,0), 3)
            contours.append(c)

def hough_target(image):
    im = cv2.medianBlur(image,5)
    canny = cv2.Canny(im,100,200)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    dilate = cv2.dilate(canny, kernel, iterations=1)
    circles = cv2.HoughCircles(dilate,cv2.HOUGH_GRADIENT_ALT,1,20,
                                param1=200,param2=0.7,minRadius=0,maxRadius=-1)
    circles = np.uint16(np.around(circles))
    x_avg = 0
    y_avg = 0
    if len(circles[0,:]) > 0:
        for circle in circles[0,:]:
            ## get averge of the circle center
            x_avg += circle[0]
            y_avg += circle[1]
        x_avg = x_avg/len(circles[0,:])
        y_avg = y_avg/len(circles[0,:])
        return x_avg, y_avg
    else:
        return None, None