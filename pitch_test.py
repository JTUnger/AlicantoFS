import json
from flight_functions import *
from cv_functions import *
from funciones_lora import *
from undistort_images import unidistort_cv2
from picamera import PiCamera
from serial import Serial
from time import sleep
from landing.landing_target import hough_target

class PitchTest():
    def __init__(self, port: str="/dev/ttyACM0", baud: int=9600, debug=False) -> None:
        print("PitchTest init!")
        self.max_images = 20
        self.debug = debug
        self.PORT = port
        self.BAUD = baud
        self.ser_samd21 = None
        self.horizontal_res = 1920
        self.vertical_res = 1080
        
        self.camera_matrix = np.array([
        [2339.0662877090776, 0.0, 1199.815925407665],
        [0.0, 2367.3154724881956, 607.0957703752879],
        [0.0, 0.0, 1.0]])
        self.distortion_coefficients = np.array([
            -0.4643616561067709,
            0.32342931446046447,
            -0.0036548702025194046,
            -0.015751779609361322,
            0.07829950688584723])

        self.vehicle = None
        self.camera_up = True
    
    def camera(self) -> None:
        # este thread toma fotos minetras el dron este a mas de 20 m
        # y guarda un json con informacion relativa a la foto cada 2 Hz
        # las fotos se llaman n.jpg y n.json, donde n es el numero de foto
        self.cam = PiCamera()
        self.cam.resolution = (self.horizontal_res, self.vertical_res)
        self.cam.color_effects = (128, 128)
        self.cam.start_preview()
        sleep(2)
        counter = 0
        while self.camera_up:
            # cutoff a 20 m para evitar confundir el landing pad con el objeto
            if self.vehicle.location.global_relative_frame.alt > 20.0:
                filename = f"{counter}.png"
                # TODO: plug undistort on {counter}.png
                self.cam(os.path.join(self.dir, filename))
                img_dat = {
                    "speed": self.vehicle.groundspeed,
                    "lat": self.vehicle.location.global_relative_frame.lat,
                    "lon": self.vehicle.location.global_relative_frame.lon,
                    "heading": self.vehicle.heading,
                    "pitch": self.vehicle.attitude.pitch,
                    "roll": self.vehicle.attitude.roll,
                    "yaw": self.vehicle.attitude.yaw,
                    "height": self.vehicle.location.global_relative_frame.alt,
                    "px": None,
                    "py": None,
                }
                with open(f'{counter}.json', 'w', encoding='utf8') as file:
                    json.dump(img_dat, file)
                counter += 1
                time.sleep(0.5)
            if counter > self.max_images:
                self.camera_up = False


    def run_pitch(self) -> None:
        # esta funcion crea una carpeta con la fecha y hora actual
        # se conecta al vehiculo, inicia el thread de la camara
        # y el thread del heartbeat. Cuando el dron sale de estado 
        # armed, se inicia el algoritmo de busqueda de imagenes
        # se revisan todas las imagenes, y se promedian sus ubicaciones
        # una vez que esto se tiene, se envia por el heartbeat 2 veces,
        # respalda esta informacion en un archivo JSON y luego acaba
        # el programa.
        print("Running pitch test mission")
        foldername = f"pitch_test"
        self.dir = os.path.join(os.getcwd(), "images", foldername)
        os.mkdir(self.dir)
        self.ser_samd21 = Serial(self.PORT, self.BAUD)
        print(f"Connected to LoRa on {self.PORT}:{self.BAUD}")
        self.vehicle = connectMyCopter()
        print("Connected to vehicle!")
        self.camera()
        for elem in range(self.max_images):
            json_path = os.path.join(self.dir, f"{elem}.json")
            with open(json_path, 'r', encoding='utf8') as file:
                img_dat = json.load(file)
            img_path = os.path.join(self.dir, f"{elem}.png")
            img = cv2.imread(img_path)
            img = unidistort_cv2(img)
            px, py = hough_target(img)
            img_dat["px"] = px
            img_dat["py"] = py
            with open(json_path, 'w', encoding='utf8') as file:
                json.dump(img_dat, file)
        print("Done!")
        self.ser_samd21.close()
        print("END OF PITCH TEST")	

if __name__ == "__main__":
    print("Starting SAR_main")
    pitch = PitchTest()
    pitch.run_pitch()