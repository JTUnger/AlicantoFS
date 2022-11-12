import json
from flight_functions import *
from cv_functions import *
#from funciones_lora import *
from picamera import PiCamera
from threading import Thread
from serial import Serial
from time import sleep
from sift.sift import SIFT
from datetime import datetime

class SarControl():
    def __init__(self, port="/dev/ttyACM0", baud=9600) -> None:
        self.PORT = port
        self.BAUD = baud
        self.ser_samd21 = None
        self.takeoff_height = 4
        self.x_area = 6
        self.y_area = 6
        self.x_landpad = 1
        self.y_landpad = 1
        self.search_height = 6
        self.horizontal_res = 1920
        self.vertical_res = 1080
        self.doRTL = False
        self.debug = True
        self.heart_up = True
        self.camera_up = True
        self.heart_data = {
            "status": 2,
            "objectA": None,
            "latA": None,
            "nsA": None,
            "lonA": None,
            "ewA": None,
            "objectB": None,
            "latB": None,
            "nsB": None,
            "lonB": None,
            "ewB": None,
            "id": "CALCH"  # TODO: verificar id equipo
        }
        # falta agregar un thread que monitoree el estado del dron
        self.status_values = {"Manual": 1, "Autonomous": 2, "Faulted": 3}
        self.status_format = ["objectA", "latA", "nsA", "lonA", "ewA", "objectB", "latB", "nsB", "lonB", "ewB", "id", "status", ]
        self.vehicle = None
        self.sift = SIFT()
        self.heart_thread = Thread(target=self.heartbeat)
        self.camera_thread = Thread(target=self.camera)
    
    def query_sift(self, query_img: cv2.Mat, letter: str) -> tuple:
        # esta funcion toma una imagen y una letra, y revisa cual de ambas es
        # retorna None en caso de no encontrar una respuesta o un match doble
        # en caso de encontrar una letra, retorna el centroide de los 
        # landmarks identificados
        kp_t = None
        good = None
        key_pts = None
        if letter == 'r':
            self.sift.set_query_img(self.sift.robo_r)
            kp_t, good = self.sift.get_sift_matches(query_img)
            if kp_t is not None and good is not None:
                key_pts = self.sift.get_keypoints(kp_t, good)
            else:
                return None
        elif letter == 'n':
            self.sift.set_query_img(self.sift.robo_n)
            kp_t, good = self.sift.get_sift_matches(query_img)
            if kp_t is not None and good is not None:
                key_pts = self.sift.get_keypoints(kp_t, good)
            else:
                return None
        else:
            raise ValueError("Letter must be 'n' or 'r'")
        if key_pts:
            x = 0
            y = 0
            for pt in key_pts:
                x += pt[0]
                y += pt[1]
            x /= len(key_pts)
            y /= len(key_pts)
            return (x, y)
        else:
            return None
    
    def heartbeat(self) -> None:
        # este thread envia el heartbeat al transmisor lora
        # cada 1 Hz
        # TODO: integrar vehicle status aca
        def parse_heart(data: dict) -> str:
            out = ""
            for val in self.status_format:
                if data[val] is not None:
                    out += f"{data[val]},"
                else:
                    out += ","
            return out
        def transmit_heart(data: str) -> None:
            self.ser_samd21.write(data.encode('utf8'))
        while self.heart_up:
            out = parse_heart(self.heart_data)
            transmit_heart(out)
            sleep(1)
    
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
            if self.vehicle.location.global_relative_frame > 20:
                filename = f"{counter}.png"
                self.cam(os.path.join(self.dir, filename))
                img_dat = {
                    "speed": self.vehicle.groundspeed,
                    "lat": self.vehicle.location.global_relative_frame.lat,
                    "lon": self.vehicle.location.global_relative_frame.lon,
                    "heading": self.vehicle.heading,
                    "height": self.vehicle.range_finder.distance,
                }
                with open(f'{counter}.json', 'w', encoding='utf8') as file:
                    json.dump(img_dat, file)
                counter += 1
                time.sleep(0.5)


    def run_sar(self) -> None:
        # esta funcion crea una carpeta con la fecha y hora actual
        # se conecta al vehiculo, inicia el thread de la camara
        # y el thread del heartbeat. Cuando el dron sale de estado 
        # armed, se inicia el algoritmo de busqueda de imagenes
        # se revisan todas las imagenes, y se promedian sus ubicaciones
        # una vez que esto se tiene, se envia por el heartbeat 2 veces,
        # respalda esta informacion en un archivo JSON y luego acaba
        # el programa.
        foldername = f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
        self.dir = os.path.join(os.getcwd(), "images", foldername)
        os.mkdir(self.dir)
        self.ser_samd21 = Serial(self.PORT, self.BAUD)
        self.vehicle = connectMyCopter()
        self.heart_thread.start()
        self.camera_thread.start()
        while self.vehicle.armed:
            sleep(1)
        self.camera_up = False
        self.camera_thread.join()
        positions = {'r': [], 'n': []}
        for i in range(int(len(os.listdir(self.dir))/2)):
            metadata = None
            json_path = os.path.join(os.getcwd(), f'{i}.json')
            img_path = os.path.join(os.getcwd(), f'{i}.png')
            with open(json_path, 'w', encoding='utf8') as file:
                metadata = json.load(file)
            query_img = cv2.imread(img_path)
            query_r = self.query_sift(query_img, 'r')
            query_n = self.query_sift(query_img, 'n')
            query_centroid = None
            if (query_r and query_n) or metadata["height"] < 20.0:
                pass  # descartar, R y N o esta a menos de 20 m
            elif query_r:
                query_centroid = query_r
                # TODO: transform relative point to polar
                positions['r'].append((None, None))
            elif query_n:
                query_centroid = query_n
                # TODO: transform relative point to polar
                positions['n'].append((None, None))
        averages = {'n': None, 'r': None}
        for key in positions.keys():
            # TODO: remove outliars?
            lat = 0
            lon = 0
            for elem in positions[key]:
                lat += elem[0]
                lon += elem[1]
            lat = lat/len(positions)
            lon = lon/len(positions)
            averages[key] = (lat, lon)
        if averages['r']:
            self.heart_data['objectA'] = 'r'
            self.heart_data['latA'] = averages['r'][0]
            self.heart_data['nsA'] = 'S'
            self.heart_data['lonA'] = averages['r'][0]
            self.heart_data['ewA'] = 'E'
        if averages['n']:
            self.heart_data['objectB'] = 'n'
            self.heart_data['latB'] = averages['n'][0]
            self.heart_data['nsB'] = 'S'
            self.heart_data['lonB'] = averages['n'][0]
            self.heart_data['ewB'] = 'E'
        
        with open('end_data.json', 'w', encoding='utf8') as file:
            json.dump(self.heart_data, file)

        self.heart_thread.join()
        self.heart_up = True
        self.ser_samd21.close()
        print("END OF SAR")	

if __name__ == "__main__":
    sar = SarControl()
    sar.run_sar()