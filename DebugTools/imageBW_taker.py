import os
from picamera import PiCamera
from time import sleep

photo_id = 1

camera = PiCamera()
camera.resolution = (1920, 1080)
camera.color_effects = (128,128)
camera.start_preview()
sleep(2)

while True:
    #ret, frame = cap.read()
    #Save to images folder with id 
    save_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "images")
    save_name = "photo_" + str(photo_id) + ".jpg"
    camera.capture(os.path.join(save_path, save_name))
    print("Photo saved: " + save_name)
    photo_id += 1
    time.sleep(0.7)