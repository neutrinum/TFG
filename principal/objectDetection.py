
from imutils.video import VideoStream
import cv2
import numpy as np



def objectThresh(img,l1,l2,m1,m2,m3,h1,distReal):
    

    hls =  cv2.cvtColor(img,cv2.COLOR_BGR2HLS)
    
    # rangos de colores del fondo en HLS
    low1 = np.array([0,0,0])            #([0,0,0])
    low2 = np.array([180,l1,l2])      #([180,125,120])

    mid1 = np.array([0,m2,0])          #([0,160,0])  
    mid2 = np.array([m1,h1,m3])       #([120,200,80])

    high1 = np.array([0,h1,0])         #([0,200,0]) 
    high2 = np.array([180,255,255])     #([180,255,255]) 
        
    # obtiene la imagen binaria para cada rango
    maskLow = cv2.inRange(hls, low1, low2)
    maskMid = cv2.inRange(hls, mid1, mid2)
    maskHigh = cv2.inRange(hls, high1, high2)

    # invierte imagen binaria
    maskLow = cv2.bitwise_not(maskLow)
    maskMid = cv2.bitwise_not(maskMid)
    maskHigh = cv2.bitwise_not(maskHigh)

    # unión de las tres imágenes binarias
    mask = cv2.bitwise_and(maskLow,maskHigh)
    mask = cv2.bitwise_and(mask,maskMid)
    
    # eliminación de ruido
    kernel_op = np.ones((3,3),np.uint8)
    kernel_cl = np.ones((3,3),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_op)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_cl)


    cv2.imshow('mask',mask)

    M = cv2.moments(mask)

    # calcula el centroide a partir de los momentos si se detecta un mínimo
    if M['m00']>5000:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        d = distReal(cy)
        #print('dist {: 7.2f} '.format(d))
    else:
        cx,cy = 0,0
        d = 0

    # Filtra objetos anormalmete pequeños (ruido).
    # Si aparecen areas muy pequeñas por debajo de la imagen,
    # es muy probable que no sea un objeto. Un objeto muy próximo
    # (por la zona baja de la imagen) debería mostrar bastante área.
    # "M['m00']*int(d)" en teoría sería constante para un objeto dado,
    # se calibra para el minimo valor que se considera objeto,
    # en este caso 300000
    if M['m00']*int(d) > 300000:
        #print('area ','{:.3f}'.format(float(M['m00']/1000)))
        #print('dist {: 7.2f} '.format(d))
        #print('obj')

        return (True,d)
    else:
        return (False,100)

    #maskColor = np.dstack((mask,mask,mask))

    #cv2.circle(maskColor,(cx,cy), 2, (0,255,0))

    #cv2.imshow('maskColor',maskColor)
    
    

