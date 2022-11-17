import json
from cv_functions import *
from Give_RN_coordinates import coordenadas_RN
from undistort_images import unidistort_cv2
from time import sleep
from sift.sift import SIFT
from datetime import datetime
from SAR_main import SarControl
import os

sar_control = SarControl()

sar_control.sift_r.robo_r = cv2.imread("./sift/letters/roboR/r0.png")
sar_control.sift_r.set_query_img(sar_control.sift_r.robo_r)
sar_control.sift_b.robo_b = cv2.imread("./sift/letters/black/b0.png")
sar_control.sift_b.set_query_img(sar_control.sift_b.robo_b)
sar_control.sift_n.robo_n = cv2.imread("./sift/letters/roboN/n0.png")
sar_control.sift_n.set_query_img(sar_control.sift_n.robo_n)

dir = "D:/3"

print("Processing images...")
positions = {'r': [], 'n': [], 'b': []}
for i in range(int(len(os.listdir(dir))/2)):
    metadata = None
    json_path = os.path.join(dir, f'{i}.json')
    img_path = os.path.join(dir, f'{i}.png')
    with open(json_path, 'r', encoding='utf8') as file:
        metadata = json.load(file)
    print(f'Processing image {i}...')
    print(metadata)
    query_img = cv2.imread(img_path)
    query_img = unidistort_cv2(query_img)
    query_r = sar_control.query_sift(query_img, 'r')
    query_n = sar_control.query_sift(query_img, 'n')
    query_b = sar_control.query_sift(query_img, 'b')
    print(f"Sift Query done! r: {query_r}; n: {query_n}; b: {query_b}")
    query_centroid = None
    if (query_r and query_n) or (metadata["height"] < 20.0 and not sar_control.debug):
        pass  # descartar, R y N o esta a menos de 20 m
    elif query_r:
        query_centroid = query_r
        lat, lon = coordenadas_RN(
            width=sar_control.horizontal_res, height=sar_control.vertical_res,
            vehicle_altitude=metadata['height'],
            center_image_coordinates=(metadata['lat'], metadata['lon']),
            target_xy_coordinates=query_centroid,
            vehicle_heading=metadata['heading']
        )
        positions['r'].append((lat, lon))
    elif query_n:
        query_centroid = query_n
        lat, lon = coordenadas_RN(
            width=sar_control.horizontal_res, height=sar_control.vertical_res,
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
    sar_control.heart_data['objectA'] = 'r'
    sar_control.heart_data['latA'] = averages['r'][0]
    sar_control.heart_data['nsA'] = 'S'
    sar_control.heart_data['lonA'] = averages['r'][1]
    sar_control.heart_data['ewA'] = 'E'
if averages['n']:
    sar_control.heart_data['objectB'] = 'n'
    sar_control.heart_data['latB'] = averages['n'][0]
    sar_control.heart_data['nsB'] = 'S'
    sar_control.heart_data['lonB'] = averages['n'][1]
    sar_control.heart_data['ewB'] = 'E'

with open(os.path.join(dir, 'end_data.json'), 'w', encoding='utf8') as file:
    json.dump(sar_control.heart_data, file)

print(positions)