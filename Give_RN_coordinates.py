import cv2 

import numpy as np

# import scipy as sp

import latlon as ll_converter

'''def add_offset(latitude, longitude):
    meters = 10
    earth_radius_in_km = 6378.137
    coeff = (1 / ((2 * math.pi / 360) * earth_radius_in_km)) / 1000
    blur_factor = meters * coeff
    new_lat = latitude + blur_factor
    new_long = longitude + blur_factor / math.cos(latitude * 0.018)
    return new_lat, new_long'''

def coordenadas_RN(width, height, vehicle_altitude, center_image_coordinates, target_xy_coordinates, vehicle_heading):

    cameraMatrix = np.array([[2339.0662877090776, 0, 1199.815925407665], [0, 2367.3154724881956, 607.0957703752879], [0, 0, 1]])
    distCoeffs   = np.array([-0.4643616561067709, 0.32342931446046447, -0.0036548702025194046, -0.015751779609361322, 0.07829950688584723])

    ##  width = 1920 -> float, pixels
    ##  height = 1080 -> float, pixels
    ##  instance_image = cv2.imread(r'path')
    ##  center_image_coordinates = np.asarray([lat, long])
    ##  target_xy_coordinates = np.asarray([[x, y]])
    ##  vehicle_heading = 0 -> float, degrees, being 0 NORTH (NED)

    ## Define the center of the image in pixels
    
    center_x_pixels = width/2
    center_y_pixels = height/2
    img_center_pixels = np.asarray([center_x_pixels, center_y_pixels])

    #We need to align with the north our corresponding points of the target

    # X_t = X_d * cos(theta) - Y_d * sin(theta)
    # Y_t = X_d * sin(theta) + Y_d * cos(theta)

    target_x_aligned = target_xy_coordinates[0] * np.cos(np.deg2rad(vehicle_heading)) - target_xy_coordinates[1] * np.sin(np.deg2rad(vehicle_heading))
    target_y_aligned = target_xy_coordinates[0] * np.sin(np.deg2rad(vehicle_heading)) + target_xy_coordinates[1] * np.cos(np.deg2rad(vehicle_heading))

    target_pixels = np.asarray([target_x_aligned, target_y_aligned])

    # Now with the orth aligned, we need to get the distance in pixels from the center

    offset_pixels = target_pixels - img_center_pixels

    offset_pixels = np.asarray([offset_pixels[0] *-1, offset_pixels[1]])

    # We will calculate relationship between a pixel and meters
    #This transforms pixels to meters (this is the scale factor)

    pixel_to_m_x = (vehicle_altitude/cameraMatrix[0][0])
    pixel_to_m_y = (vehicle_altitude/cameraMatrix[1][1])

    scale_factor = np.asarray([pixel_to_m_x, pixel_to_m_y])

    #Transform pixel distance to meters

    offset_meters = np.multiply(offset_pixels, scale_factor)

    # Now we can calculate the offset in coordinates

    center_in_meters = ll_converter.LLtoUTM(2, center_image_coordinates[1], center_image_coordinates[0]) #RETURNS (ZONE, LONG, LAT)

    latlon_meters = np.asarray([center_in_meters[2], center_in_meters[1]]) #Convenient (Lat,Lon) array

    coordinates_meters_with_offset = np.add(latlon_meters, offset_meters) #Add center in meters to offset in meters

    offset_in_coordinates = ll_converter.UTMtoLL(2, coordinates_meters_with_offset[0], coordinates_meters_with_offset[1], center_in_meters[0])
    # RETURNS (LAT,LONG)

    return offset_in_coordinates[1], offset_in_coordinates[0]
    

