import math 
import cv2
import numpy as np

def detect_letter_xy(photo):
    """Detects the letter in the photo, returns the x and y coordinates 
    of the letter and letter type. Asummes that the photo is already fliped
    to correct orientation"""
    #Camera Parameters
    horizontal_res = 1920
    vertical_res = 1080
    horizontal_fov = 59 * (math.pi/180)
    vertical_fov = 46 * (math.pi/180)

    #Camera Intrinsics(REMEBER TO RECALIBRATE FOR NEW RESOLUTION)
    cameraMatrix   = np.array([[933.9466925521707, 0, 358.59608943398365],
                [0, 935.0635463990791, 293.0721064675127],
                [0, 0, 1]])
    cameraDistortion   = np.array([-0.4530790972005633, 0.3951099938612813, 0.0037673873203789916, 
                                    0.0016363264710513889, -0.38177331299300393])

    pass