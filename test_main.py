from flight_functions import *
import cv2

photo_id = 1
cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)

vehicle = connectMyCopter()

while True:
    ret, frame = cap.read()
    #Save to SARPhoto folder with id and vehicle altitude
    save_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "SARPhotos")
    save_name = "photo_" + str(photo_id) + "_" + str(vehicle.location.global_relative_frame.alt) + ".jpg"
    cv2.imwrite(os.path.join(save_path, save_name), frame)
    photo_id += 1
    time.sleep(2)

