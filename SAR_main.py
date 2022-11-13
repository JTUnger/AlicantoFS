import json
from flight_functions import *
from cv_functions import *
from funciones_lora import *
from Give_RN_coordinates import coordenadas_RN
from undistort_images import unidistort_cv2
from picamera import PiCamera
from threading import Thread
from serial import Serial
from time import sleep
from sift.sift import SIFT
from datetime import datetime

class SarControl():
    def __init__(self, port: str="/dev/ttyACM0", baud: int=9600) -> None:
        self.PORT = port
        self.BAUD = baud
        self.ser_samd21 = None
        self.horizontal_res = 1920
        self.vertical_res = 1080
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
            "id": "CHILE"  
        }
        
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
        def update_vehicle_status() -> None:
            current_status = reportar_status_SAR(self.vehicle)
            self.heart_data["status"] = self.status_values[current_status]
        
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
            update_vehicle_status()
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
            if self.vehicle.location.global_relative_frame.alt > 20.0:
                filename = f"{counter}.png"
                # TODO: plug undistort on {counter}.png
                self.cam(os.path.join(self.dir, filename))
                img_dat = {
                    "speed": self.vehicle.groundspeed,
                    "lat": self.vehicle.location.global_relative_frame.lat,
                    "lon": self.vehicle.location.global_relative_frame.lon,
                    "heading": self.vehicle.heading,
                    "height": self.vehicle.location.global_relative_frame.alt,
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
        while not self.vehicle.armed:
            sleep(1)
        self.heart_thread.start()
        self.camera_thread.start()
        while self.vehicle.armed:
            sleep(1)
            if self.vehicle.mode.name == "LAND":
                self.camera_up = False
                self.camera_thread.join()
                break
        #landingpad_precision_landing(self.vehicle) #Este puede fallar hay que debugear!
        positions = {'r': [], 'n': []}
        for i in range(int(len(os.listdir(self.dir))/2)):
            metadata = None
            json_path = os.path.join(os.getcwd(), f'{i}.json')
            img_path = os.path.join(os.getcwd(), f'{i}.png')
            with open(json_path, 'w', encoding='utf8') as file:
                metadata = json.load(file)
            query_img = cv2.imread(img_path)
            query_img = unidistort_cv2(query_img)
            query_r = self.query_sift(query_img, 'r')
            query_n = self.query_sift(query_img, 'n')
            query_centroid = None
            if (query_r and query_n) or metadata["height"] < 20.0:
                pass  # descartar, R y N o esta a menos de 20 m
            elif query_r:
                query_centroid = query_r
                lat, lon = coordenadas_RN(
                    width=self.horizontal_res, height=self.vertical_res,
                    vehicle_altitude=metadata['height'],
                    center_image_coordinates=(metadata['lat'], metadata['lon']),
                    target_xy_coordinates=query_centroid,
                    vehicle_heading=metadata['heading']
                )
                positions['r'].append((lat, lon))
            elif query_n:
                query_centroid = query_n
                lat, lon = coordenadas_RN(
                    width=self.horizontal_res, height=self.vertical_res,
                    vehicle_altitude=metadata['height'],
                    center_image_coordinates=(metadata['lat'], metadata['lon']),
                    target_xy_coordinates=query_centroid,
                    vehicle_heading=metadata['heading']
                )
                positions['n'].append((lat, lon))
        averages = {'n': (None, None), 'r': (None, None)}
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
        
        with open(os.join(self.dir, 'end_data.json'), 'w', encoding='utf8') as file:
            json.dump(self.heart_data, file)

        self.heart_up = False
        self.heart_thread.join()
        self.ser_samd21.close()
        print("END OF SAR")	

if __name__ == "__main__":
    sar = SarControl()
    sar.run_sar()