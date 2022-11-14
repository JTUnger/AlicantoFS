import cv2 

import numpy as np 

import scipy as sp


def encontrar_mayores(lista, n):
    orden = lista.copy()
    orden.sort()
    mayores = orden[-n:]
    indice = [lista.index(i) for i in mayores]
    return indice 

def centroide(momentos):
    x = int(momentos["m10"] / momentos["m00"])
    y = int(momentos["m01"] / momentos["m00"])
    return [x,y]



#### Inputs


image = cv2.imread(r'C:\Users\Adminitrador\Desktop\mm\SARPhotos_BW_2\photo_rectificado_1.jpg')
lat_lon = np.asarray([[-36.0000000, -70.0000000]])
#############################################################


image = cv2.resize(image, (660, 340)) 
img_cntr = np.asarray([[660/2, 340/2]])
copia = image.copy()
print('Image Read')
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)



#HSV limits
lower_orange = np.array([0, 0, 0], dtype = "uint8") 

upper_orange= np.array([255, 255, 255], dtype = "uint8")


mask = cv2.inRange(image, 0, 255)

detected_output = cv2.bitwise_and(image, image, mask =  mask) 

escala_grises1 = cv2.cvtColor(detected_output, cv2.COLOR_BGR2GRAY)


###########################################
###Encontrar contornos####


thresh = cv2.threshold(escala_grises1,0,255,cv2.THRESH_OTSU + cv2.THRESH_BINARY)[1]
cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]
total = 0

size_contours = []
momentos = []
'''
for c in cnts:
    x,y,w,h = cv2.boundingRect(c)
    mask = np.zeros(hsv_image.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [c], [255,255,255])
    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    momento = cv2.moments(mask)
    momentos.append(momento)
    pixels = cv2.countNonZero(mask)
    total += pixels
    size_contours.append(pixels)
    cv2.putText(hsv_image, '{}'.format(pixels), (x,y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

#### Se buscan lo contornos mayores
conos = encontrar_mayores(size_contours, 4)

contours_conos = [cnts[i] for i in conos]

##Posicion conos en pixeles
posicion_conos = np.asarray([centroide(momentos[i]) for i in conos])


## Matriz de distancias
dist_matrix = sp.spatial.distance_matrix(posicion_conos,posicion_conos)
dist_matrix[dist_matrix == 0] = np.inf
ind = np.unravel_index(np.argmin(dist_matrix, axis=None), dist_matrix.shape)


m_por_pixel = 10/np.min(dist_matrix)

tasa_m_cord = 110000 

lat_lon = np.asarray([[-36.0000000, -70.0000000]])
'''

##### output
#posicion_conos_cord = (posicion_conos - img_cntr)*m_por_pixel/tasa_m_cord + lat_lon 


#print(posicion_conos_cord)


##############################################################################3

'''cv2.drawContours(image, contours_conos, -1, (255,0,0),10)



cv2.imshow('thresh', thresh)
cv2.imshow('image', image)
cv2.imshow('copia', copia)

cv2.imshow("red color detection", detected_output) 
cv2.waitKey(0)'''






print(total)
cv2.imshow('thresh', thresh)
cv2.imshow('image', image)
cv2.waitKey(0)

#print('Moments: ', M1)
#print('Output Detected')
cv2.imshow("red color detection", detected_output) 

cv2.waitKey(0)

