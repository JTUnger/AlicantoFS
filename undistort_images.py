import cv2
import numpy as np
import os

#(python3 SAR_main.py &)
#htop python3 SAR_main.py
#F3 search and then F9 to kill

## Reference: https://stackoverflow.com/questions/62923212/python-opencv-how-to-get-an-undistorted-image-without-the-cropping

def loadUndistortedImage(fileName):
  # load image
  image = cv2.imread(fileName)
  #print(image)

  # set distortion coeff and intrinsic camera matrix (focal length, centerpoint offset, x-y skew)
  cameraMatrix = np.array([[2339.0662877090776, 0, 1199.815925407665], [0, 2367.3154724881956, 607.0957703752879], [0, 0, 1]])
  distCoeffs   = np.array([-0.4643616561067709, 0.32342931446046447, -0.0036548702025194046, -0.015751779609361322, 0.07829950688584723])

  # create undistortion maps
  R = np.array([[1,0,0],[0,1,0],[0,0,1]])
  #map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, R, newCameraMatrix, imageSize, cv2.CV_16SC2)
  h,  w = image.shape[:2]
  newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (w,h), 1, (w,h))
  undistorted_image = cv2.undistort(image, cameraMatrix, distCoeffs, None, newcameramtx)

  ### SHOW IMAGE
  #cv2.imshow("undistorted", undistorted_image)
  #cv2.imwrite(fixed_filename, outputImage)
  #cv2.imshow('fix_img',outputImage)
  #cv2.waitKey(0)


  # remap
  save_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "SARPhotos_BW_2")
  save_name = "photo_" + str("rectificado") + "_1" + ".jpg"
  cv2.imwrite(os.path.join(save_path, save_name), undistorted_image)
  return

#Undistort the images, then save the restored images
loadUndistortedImage(r'C:\Users\Adminitrador\Desktop\mm\ajsdlksajdl\photo_31_17.426.png')