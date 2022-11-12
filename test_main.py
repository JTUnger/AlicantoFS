from flight_functions import *
from picamera import PiCamera
from time import sleep

photo_id = 1

camera = PiCamera()
camera.resolution = (1920, 1080)
camera.color_effects = (128,128)
camera.start_preview()
sleep(2)

vehicle = connectMyCopter()

while True:
    #ret, frame = cap.read()
    #Save to SARPhoto folder with id and vehicle altitude
    save_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "SARPhotosBW")
    save_name = "photo_" + str(photo_id) + "_" + str(vehicle.location.global_relative_frame.alt) + ".png"
    camera.capture(os.path.join(save_path, save_name))
    print("Photo saved: " + save_name)
    photo_id += 1
    time.sleep(0.7)