# MAIN PROGRAM

#from car_control import video_dir, car_dir, motor
import video_dir
import car_dir
import motor

from imutils.video import VideoStream
import imutils
import cv2
import numpy as np
import threading
import time
from math import tan, cos, sin, exp, isnan, asin

from sonarPair import Sonar
import RPi.GPIO as GPIO

from curveAngle import angleImg, angleReal
from objectDetection import objectColor, objectThresh



debug_motion=False
debug_sonar=False

#----------------------- Parámetros Óptica
focal=555 # en pixels
alpha= 34 / 360 * (2*np.pi) # o en radianes
L = 15
y0 = -11.5

#---------------------
resolution=(112,80)
scale=resolution[0]/640
focal = int(focal*scale)
fps = 40
rotation=180

printedShape=False  

cx=cy=cx_=cy_=None
inited_pid = False
rst_pid = False
G=128


spdPrev = spdMin = 30
spdChoice = 0

skipObj=False
newObj = False

M = None # moments


detectSigns = False
crossStop = False
crossWait = False

#-----------------------
def nothing(x):
    pass

cv2.namedWindow('main')
cv2.namedWindow('ctrl')
cv2.namedWindow('canny')
cv2.namedWindow('objParams')

cv2.createTrackbar('th','main',180,255,nothing)
cv2.createTrackbar('go','main',0,1,nothing)

cv2.createTrackbar('spd','ctrl',38,100,nothing)
cv2.createTrackbar('spdUp','ctrl',12,100,nothing)
cv2.createTrackbar('spdMin','ctrl',36,100,nothing)
cv2.createTrackbar('wght','ctrl',65,100,nothing)
cv2.createTrackbar('Kp','ctrl',100,300,nothing)
cv2.createTrackbar('Td','ctrl',10,100,nothing)
cv2.createTrackbar('Ti','ctrl',50,500,nothing)



cv2.createTrackbar('high','canny',195,255,nothing)
cv2.createTrackbar('low','canny',120,255,nothing)
cv2.createTrackbar('blur','canny',5,31,nothing)

cv2.createTrackbar('l hLs','objParams',125,255,nothing)
cv2.createTrackbar('l hlS','objParams',90,255,nothing)
cv2.createTrackbar('m Hls','objParams',180,180,nothing)
cv2.createTrackbar('m hLs','objParams',150,255,nothing)
cv2.createTrackbar('m hlS','objParams',175,255,nothing)
cv2.createTrackbar('h hLs','objParams',225,255,nothing)


spdBase = cv2.getTrackbarPos('spd','ctrl')

#--------------------------------------------------------------------
if not debug_sonar:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    sn = Sonar(pair=True)
    sn.start()
    
    #sn_dist_prev=0
else:
    sn_dist1 = sn_dist0 = 42 # anything greater than 20
        
alerta0 = alerta1 = False


if not debug_motion:
    busnum = 1 

    video_dir.setup(busnum=busnum)
    car_dir.setup(busnum=busnum)
    motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor. 
    video_dir.home_x_y()
    car_dir.home()




#------------------------------------------------------------------------------------------------
# Función para detectar señales
#-----------------------------------------------------------------------------------------------------
signalRd = False

def detectSignals():
    global vs, resolution, fps, root
    global rows, cols, focal, alpha, scale

    
    frame = vs.read()
    edged = getCanny(frame)
    
    cross,angle = isCross(edged[16:95,:],real=False)
    #angReal = angleReal(centroid=(cx_,cy_+int(rows/4)),angle=angle,
                                    #dim=(rows,cols),focal=focal,alpha=alpha,scale=scale)
    angle = 0
    print('angCruce ',angle)                
    if not debug_motion:
        video_dir.move_y(85)
        video_dir.move_x(int(2*angle/np.pi*250))    # 260 ~= 90º
        
    
    vs.stop()
    vs = VideoStream(usePiCamera=True, resolution=(320,240), framerate=fps, rotation=180).start()
    time.sleep(1.0)

    moveX = 0
    maxMatch = 0
    while maxMatch < 1:
        
        img = vs.read()
        img = img[int(0.45*240):240-1, :]

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        # edges, detection
        edged = cv2.Canny(gray, 110, 190) # source img

        detectThresh=3200000
        matches=[]
        nTemplates=4
        for i in range(nTemplates):
            tmpl=cv2.imread('templates/tmplCanny1'+str(i)+'.png',0)
            tH,tW = tmpl.shape[:2]
            found = None
            for scale in np.logspace(-0.5,0.2,6,base=2)[::-1]: # step 0.1 (9->17: step 0.05)
                # resize, ratio
                tmpl_ = imutils.resize(tmpl, width = int(tW * scale))
                r = float(tmpl_.shape[1]) / tW 
             
                result = cv2.matchTemplate(edged, tmpl_, cv2.TM_CCOEFF)
                (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
             
                # update
                if found is None or maxVal/r**2 > found[0]:
                    found = (maxVal, maxLoc, r)
             
            # box
            (maxVal, maxLoc, r) = found
            (startX, startY) = (maxLoc[0], maxLoc[1])
            (endX, endY) = ( int(maxLoc[0] + tW*r), int(maxLoc[1] + tH*r) )

            # check and add info
            if i == 0:
                maxVal *= 0.8
            matches.append( maxVal / detectThresh / r**2 )

            # DEBUG
            # draw a bounding box around the detected result and display the image
            print(maxVal/detectThresh/r**2)

            '''imgSave = np.copy(img)
            cv2.rectangle(imgSave, (startX, startY), (endX, endY), (0, 0, 255), 2)
            if i==0:
                info = 'stop'
            elif i==1:
                info = 'recto'
            elif i==2:
                info = 'dcha'
            else:
                info = 'izda'
            cv2.putText(imgSave, info,(endX+10, startY+25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)
            cv2.putText(imgSave, '{: 7.4f} '.format( maxVal / detectThresh / r**2 ),(endX, startY + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)
            name = root+info+'.png'
            cv2.imwrite(name,imgSave)'''

            #cv2.rectangle(img, (startX, startY), (endX, endY), (0, 0, 255), 2)
            cv2.imshow('Image', img)
            '''k = cv2.waitKey(0)
            if k == ord('q'):
                action = 1
                maxMatch = 2
                break'''

        action = np.argmax(np.array(matches))
        maxMatch = matches[action]
        
        if maxMatch<0.9:    # no hay señales
            action = 1
            if not debug_motion:
                if fc.ackL:
                    video_dir.move_x(-int(2*(-15)/180*250))
                    moveX -= 15
                else:
                    video_dir.move_x(-int(2*(+15)/180*250))
                    moveX += 15
                    
            if abs(moveX)>=60:  # no la ha encontrado
                maxMatch = 2
                action = 1 
        print('matches ',matches)
        print('maxMatch ',maxMatch)
        time.sleep(0.25)
        

    print('action ',action)
        

    if not debug_motion:
        video_dir.move_y(-85)
        #video_dir.move_x(-int(2*angle/np.pi*250))
        time.sleep(0.01)
        video_dir.move_x(-int(2*(-moveX)/180*250))
        time.sleep(0.1)
    
    vs.stop()
    vs = VideoStream(usePiCamera=True, resolution=resolution, framerate=fps, rotation=180).start()
    time.sleep(1.0)

    cv2.destroyWindow('Image')


    
    
    return action



#------------------------------------------------------------------------------------------------
# Función para determinar si es cruce
#------------------------------------------------------------------------------------------------
def isCross(edg,real=False):

    global rows,cols

    M0 = cv2.moments(edg)
    #print('area ',M0['m00'])
    if M0['m00'] > 50000:
    
    
        if real:
            kernel = np.ones((3,3),np.uint8)
            edg = cv2.dilate(edg, kernel, iterations = 1)

            srcBorder = np.float32([[[0,0],[0,96-1],[128-1,96-1],[128-1,0]]])
            dstBorder = np.float32([[[0,0],[48,96-1],[84,96-1],[128-1,0]]])
            transM = cv2.getPerspectiveTransform(srcBorder,dstBorder)
            edg = cv2.warpPerspective(edg, transM, (128,96))
            cv2.imshow('transform',edg)
        
        edg = edg[2:-2, 2:-2]
        
        gx = cv2.Sobel(edg, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(edg, cv2.CV_32F, 0, 1, ksize=3)

        dx = gx[(gx!=0) | (gy!=0)]
        dy = gy[(gx!=0) | (gy!=0)]

        dx=dx.ravel()
        dy=dy.ravel()

        dx = dx - np.mean(dx)
        dy = dy - np.mean(dy)

        coords = np.vstack([dx,dy])

        cov = np.cov(coords)
        #print('cov', cov)
        #print('M ',np.linalg.det(cov)-0.5*(np.trace(cov))**2)
        evals,evecs = np.linalg.eig(cov)
        #print('eig', evals)
        #print('evecs ', evecs)
        
        sort_indices = np.argsort(evals)[::-1]
        eval1, eval2 = evals[sort_indices]

        
        # REPRESENTACIÓN DEL VECTOR, TOMA ÁNGULOS PRINCIPALES
        
        #print('evecs sorted', evecs[:, sort_indices])
        evec1, evec2 = evecs[sort_indices, :]
        x_v1, y_v1 = evec1  # Eigenvector with largest eigenvalue
        x_v2, y_v2 = evec2

        mag0,ang0  = cv2.cartToPolar(np.array([x_v1,x_v2]), np.array([y_v1,y_v2]))
        #print('ang0 ', ang0*180/np.pi)
        ang0 %= np.pi
        ang0 = np.where(ang0 > np.pi/2, np.pi - ang0, -ang0)
        #print('ang0\' ', ang0*180/np.pi)
        ang1,ang2 = ang0
        #print(ang1*180/np.pi)'''
        
        #print('eigen\n',ang0*180/np.pi,'\n',evals,'\n')
		
		
        
        return ( (eval2/eval1 > 0.5) and (eval1+eval2>700000), -ang1)
    
    else:

        return (False, 0) # no es un cruce



#------------------------------------------------------------------------------------------------
# Función para obtener  el centroide de una banda de la imagen para efectuar el control de dirección
#------------------------------------------------------------------------------------------------
def bandControl(edged,cx,cy,up,down):
    global M,rows
    
    band= edged[int(up*rows):int(down*rows), :]
   
    # Contornos
    ret,cnts,hier = cv2.findContours(band,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    new=False
    if cnts:
        '''for cnt_ in cnts_:
        area = cv2.contourArea(cnt_)
        print('area ',area)
        if area>0:
        Mm = cv2.moments(cnt_)
            for el in Mm:
                print(el,Mm[el])
            print('\n\n\n')'''
        M = cv2.moments(band)
        #print('area bottom ',M['m00'])
        if M['m00']>0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            new=True
            
    return (new,cx,cy)


    
#------------------------------------------------------------------------------------------------
# Función para calcular aproximadamente el centro del cruce en la imagen
#------------------------------------------------------------------------------------------------
def distCentroid(edged):
    global cols
    #vertSum = np.sum(edged, axis=1)# - 2040  # max ~8*255 no siendo cruce

    #return  np.argmax(vertSum) # probar con centroide


    nzero0 = np.nonzero(edged[:,0])
    nzero1 = np.nonzero(edged[:,cols-1])

    if np.any(nzero1) and np.any(nzero0):
        y_centr0 = np.sum(nzero0)/np.shape(nzero0)[1]
        y_centr1 = np.sum(nzero1)/np.shape(nzero1)[1]

        y_centr = (y_centr0+y_centr1)/2
        return y_centr
    
    else:
        vertSum = np.sum(edged, axis=1)# - 2040  # max ~8*255 no siendo cruce
        return  np.argmax(vertSum) # probar con centroide


        
#------------------------------------------------------------------------------------------------
# Función para calcular una distancia real dada una posición del objeto en la imagen y dados los parámetros ópticos
#------------------------------------------------------------------------------------------------
def distReal(cy):
    global rows,alpha,focal,y0
    
    y_ = rows/2 - cy
    m = tan(alpha)

    Z = y0/(y_/focal-m)
    Y = y_/focal*Z

    dist = cos(alpha)*(Z+Y/m)
    return dist


    
#------------------------------------------------------------------------------------------------
# Función para obtener los bordes por Canny de una imagen 
#------------------------------------------------------------------------------------------------
def getCanny(frame):
    # BGR to GRAY
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = (cv2.getTrackbarPos('blur','canny'))*2-1
    gray = cv2.GaussianBlur(gray, (blur, blur), 0)


    # Bordes
    high = cv2.getTrackbarPos('high','canny')
    low = cv2.getTrackbarPos('low','canny')
    edged = cv2.Canny(gray, low, high)
    
    return edged
    

    
#------------------------------------------------------------------------------------------------
# Clase PID (no implementada)
#------------------------------------------------------------------------------------------------
class PID:
    pass



#------------------------------------------------------------------------------------------------
# Clase para tratar la tolerancia a fallos por redundancia hardware funcional
#------------------------------------------------------------------------------------------------
class FaultCtrl:
    def __init__(self):

        self.snMax = 45
        self.snMin = 15

        #self.spdReduction
        self.velReduc = 1
        
        # FLAGS
        self.camObj = False
        self.camCrs = False
        
        self.snLeft = False
        self.snRght = False
        self.snFlag = False

        #self.snLnew = False
        #self.snRnew = False
        #self.snNew = False

        #
        self.snL_ = 500
        self.snR_ = 500

        self.ack = self.ackL = self.ackR = False

        self.warning = False
        

    def cameraUpdate(self,obj,cross,dObj=None,dCross=None):
        self.camObj = obj
        self.camCrs = cross

    def sonarUpdate(self,dSonar0,dSonar1):
        self.snLeft = dSonar1<self.snMax
        self.snRght = dSonar0<self.snMax
        self.snFlag = self.snLeft or self.snRght
		
		self.snL_ = dSonar1
		self.snR_ = dSonar0


    def decide(self,obj,cross,snL,snR):

        snNewL = snL != self.snL_ and snL<self.snMin
        snNewR = snR != self.snR_ and snR<self.snMin

        snNew = snNewR or snNewR              # Nuevo objeto por debajo del umbral inferior

        
        if not self.camObj and not self.camCrs and not obj and not cross and snNew and self.snFlag:
            self.warning = True
        
        if self.warning:
            fault,vel = (True, 0)
            
        elif snL<self.snMax or snR<self.snMax:
            fault = False
            if obj:
                
                if (snL<self.snMin or snR<self.snMin) and snNew:
                    #stop normal
                    vel = 0
                else:
                    #reducir velocidad dentro de margen de control
                    vel = 0.75

            elif cross:

                vel = 1

                if snL<self.snMax and snR<self.snMax:
                    vel = self.velReduc                          # Reducción preventiva, posible objeto
                    if (snL<self.snMin or snR<self.snMin) and self.snFlag:  # snFlag obliga a tomar dos medidas < snMax
                        fault,vel = (True, 0)
						self.warning = True
                        
                    self.ack = False    # No dejamos que cruce si está detectando con los dos ultrasonidos

                elif snL<self.snMax:
                    if self.ackR:
                        vel = self.velReduc
                        if self.snLeft and snL<self.snMin:  # snLeft obliga a tomar dos medidas < snMax     // comentar 'and snL<..' para parar independientemente de la distancia
                            fault,vel = (True, 0)
							self.warning = True
						'''if self.snLeft:                            # alternativa. si se hacen dos reconocimientos de la señal a este lado, pasa considerarse que realmente se encuentra a 
						    self.ack = False                      # a este lado solo si la segunda medida es mayor que el rango mínimo
							if snL<self.snMin:                     # y mientras tanto se desactiva el reconocimiento de señal .ack    
							    fault,vel = (True, 0)
							else:
							    self.ackL = True
								self.ack = True
								self.ackR = False'''
                    elif not self.ackL or not self.ack:
                        self.ackL = True
                        self.ack = True

                else: # snR<self.snMax
                    if self.ackL:
                        vel = self.velReduc
                        if self.snRght and snR<self.snMin:  # snLeft obliga a tomar dos medidas < snMax	// comentar and snL<
                            fault,vel = (True, 0)
							self.warning = True
						'''if self.snRight:                            # alternativa. si se hacen dos reconocimientos de la señal a este lado, pasa considerarse que realmente se encuentra a 
						    self.ack = False                      # a este lado solo si la segunda medida es mayor que el rango mínimo
							if snR<self.snMin:                     # y mientras tanto se desactiva el reconocimiento de señal .ack    
							    fault,vel = (True, 0)
							else:
							    self.ackR = True
								self.ack = True
								self.ackL = False'''
                    elif not self.ackR or not self.ack:
                        self.ackR = True
                        self.ack = True
            
                
            else:
                #reducir velocidad prevetivamente
                vel = self.velReduc
                
                self.ack = self.ackL = self.ackR = False
                
        else: # Sólo dentro del rango de la cámara
            if obj:
                pass
            elif cross:
                pass
            else:
                self.ack = self.ackL = self.ackR = False
                    
            fault,vel = (False,1)


        self.cameraUpdate(obj,cross)
        self.sonarUpdate(snL,snR)

        #np.zeros((10,400,3), np.uint8)

        return (fault, vel)

# Inicialización del objeto de la clase de tolerancia a fallos
fc = FaultCtrl()
        
        
        
#---------------------------------------------
# Parámetros para escribir datos en un directorio
#---------------------------------------------
root='/home/pi/project/auto/MachineLearning/PID43/'
file = open(root+'steer_angles.txt','w')
count_=0
count_img=0
count_obj=0


#-------------------------------------------------------------------------------------
# Datos de giro en cruces para el modo de funcionamiento sin detección de señales
#-------------------------------------------------------------------------------------
mode = 1
if mode == 0:
    crossCount = 0
    with open('cruces.txt') as f:
        cruces = f.readlines()
    crossActs = []
    for cruce in cruces:
        crossActs.append(int(cruce))
    

    
#-----------------------------------------------------------------------
# Inicialización de la cámara
#-----------------------------------------------------------------------
vs = VideoStream(usePiCamera=True, resolution=resolution, framerate=fps, rotation=180).start()
time.sleep(1.0)
frame = vs.read()
#frame = cv2.resize(frame,None,fx=scale, fy=scale, interpolation = cv2.INTER_AREA)

rows,cols=frame.shape[:2]



#-----------------------------------------------
# Ventanas "sin imagen" para poner solo deslizadores
#-----------------------------------------------
canny = np.zeros((1,400,3), np.uint8)
objParams = np.zeros((1,400,3), np.uint8)






# ==========================================================================
# BUCLE PRINCIPAL
# ==========================================================================


while True:

            
    # Banda que servirá para mostrar la representación gráfica de la acción de control
    ctrl = np.zeros((10,400,3), np.uint8)
    
    
    
    # Rutina de detección de señales de tráfico. 
    # El bucle while no continua hasta que ésta no termine
    #-----------------------------------------------
    if detectSigns:
        crossAction = detectSignals()
        detectSigns = False
        signalRd = True
        crossStop = False

        inited_pid = inited_pid_ = False
        t1= cv2.getTickCount()
        
        if crossAction == 0:
            crossWait = True
        else:
            crossWait = False
     #-----------------------------------------------

     
    # Se obtiene la última imagen guardada en el hilo de la cámara
    frame = vs.read()


    # Información del tamaño de la imagen
    if not printedShape:
        print(threading.active_count())
        print(frame.shape)
        printedShape=True


    # OBJETO
    # ========================================

    l1=cv2.getTrackbarPos('l hLs','objParams')
    l2=cv2.getTrackbarPos('l hlS','objParams')
    m1=cv2.getTrackbarPos('m Hls','objParams')
    m2=cv2.getTrackbarPos('m hLs','objParams')
    m3=cv2.getTrackbarPos('m hlS','objParams')
    h1=cv2.getTrackbarPos('h hLs','objParams')
    
    # Boolean, Float
    newObj,distObj = objectThresh(frame,l1,l2,m1,m2,m3,h1,distReal)
    
    
        
    # ULTRASONIDO
    # ========================================
    
    if not debug_sonar:
        sn_dist0,sn_dist1=sn.read()
        #print('L ',sn_dist1)
        #print('R ',sn_dist0)
            

            
            
    # PROCESAMIENTO DE IMAGEN
    # =============================================================================
   
    

    # Obtención de bordes por técninca de Canny
    edged = getCanny(frame)

    

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # LÓGICA DE OPERACIÓN
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    go = cv2.getTrackbarPos('go','main')

    newMain = newAux = False
                                                                              #  _______
                                                                              # |       |
    if np.any(edged) and go:                                                  # |   |_  |
                                                                              # |___|___|
        # Detecta si hay cruce y su ángulo
        cross,angle = isCross(edged)

		# Teniendo ya datos de Cruce, Objeto y Ultrasonidos, decide si hay fallo
        fault, spdCtrl = fc.decide(newObj,cross,sn_dist1,sn_dist0) 
                                                                
        if cross and not fault:                                               # + cruce +
			
			# Debug para ver cuando los ultrasonidos detectan las señales
            #print('L ',sn_dist1)
            #print('R ',sn_dist0)
            #print('ack ',fc.ack)
            
            print('cruce ')
			# Modo girar sin mirar señales
			# - - - - - - - - - - - - - - - - - - - - - - -
            if mode == 0:
                spdChoice = 1

                crossDist = distCentroid(edged) 
                if crossDist < 40:# or newObj:    
                    (newMain,cx,cy) = bandControl(edged,cx,cy,0.8,1.0)
                    
                elif not signalRd:
                    signalRd = True
                    crossAction = crossActs[crossCount]
                    print('cruce n ',crossCount)
                    crossCount += 1
                    crossCount %= 6     # 6 acciones consecutivas en fichero 
                
            else:
                spdChoice = 0
            
            # Modo detectar señal
			# - - - - - - - - - - - - - - - - - - - - - - -
            if not signalRd and mode != 0:
                
                crossDist = distCentroid(edged) 
                print('dist ',crossDist)
                
                if crossDist < 15 or not fc.ack:# or newObj:       # el cruce está por arriba de la pantalla (lejos)
                                        
                    (newMain,cx,cy) = bandControl(edged,cx,cy,0.8,1.0)
                    
                else:
                    crossStop = True
                    detectSigns = True
                    
            # Acciones en los cruces cuando se ha leido la señal (en mode=1) o cuando se ha detectado cruce (en mode=0)
			# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            if signalRd:   # reincorporación después de leer señal
                
                if crossAction == 1: # no hay señal o señal de continúa recto
                    (newMain,cx,cy) = bandControl(edged,cx,cy,0.25,0.5) # control con
                    if not newMain:                                     # mitad superior
                        newMain = True      # obligamos a que vaya recto
                        cx = cols/2
                    #print('recto')
                        
                elif crossAction == 0: # señal de parar (hasta recibir orden)
                    if not crossWait:
                        crossStop = False
                        (newMain,cx,cy) = bandControl(edged,cx,cy,0.25,0.5)
                        if not newMain:
                            newMain = True 
                            cx = cols/2
                    elif not crossStop:
                        crossStop = True
                        print('parando')
                        
                elif crossAction == 2: # señal de gira dcha
                    newMain = True
                    cx = cols
                    spdChoice = 1

                    if not debug_motion:
                        motor.forwardWithSpeed(spdBase)
                        car_dir.turn(255)
                        time.sleep(1.25)
                        t1=t1-1.25
                    else:
                        print('giro dcha')
                    
                elif crossAction == 3: # señal de gira izda
                    newMain = True
                    cx = 0
                    spdChoice = 1

                    if not debug_motion:
                        motor.forwardWithSpeed(spdBase)
                        car_dir.turn(0)
                        time.sleep(1.25)
                        t1=t1-1.25
                    else:
                        print('giro izda')

		# No se ha detectado cruce
		#  - - - - - - - - - - - - - - - - - - - - - - -
        elif not fault:                                                        # | 1 carril | 
            
            if signalRd:
                signalRd = False    # reset variables de cruce por si 
            if crossStop:           # se interrumpe la secuencia a mitad
                crossStop = False

            (newMain,cx,cy) = bandControl(edged,cx,cy,0.8,1.0)
            (newAux,cx_,cy_) = bandControl(edged,cx_,cy_,0.25,0.5)
      
            if not newMain:             # no hay dato de la banda de control principal (newMain = False)
                if not inited_pid:         # si no se ha iniciado el PID
                    spdChoice = 0
                    if newAux:              # entrada en perpendicular a la pista. NO FUNCIONA BIEN EN PRESENCIA DE REFLEJOS
                        angReal = angleReal(centroid=(cx_,cy_+int(rows/4)),angle=angle,
                                        dim=(rows,cols),focal=focal,alpha=alpha,scale=scale)
                        if abs(angReal)>np.pi/6:
                            
                            nzero = np.nonzero(edged[:,int(cols/2)])
                            if np.any(nzero):
                                y_centr = np.sum(nzero)/np.shape(nzero)[1]
                            else:
                                y_centr = 0
                            dist = distReal(y_centr)
                            
                            theta2 = (np.pi/2-abs(angReal))/2
                            print('{:.2f}'.format(float(Z)), '{:.2f}'.format(float(Y)), '{:.3f}'.format(float(theta2/np.pi*180)))

                            try:
                                delta = (np.sign(-angReal))*asin( L/( dist*tan(np.pi/4+theta2) ) )
                            except ValueError:
                                delta = (np.sign(-angReal))*np.pi/4
                                
                            delta = -np.pi/4 if delta<-np.pi/4 else np.pi/4 if delta >np.pi/4 else delta    # crop
                            delta *= 0.9    # coger curva un poco abierta para sobrepasar linea'''

                            print ('delta ',delta/np.pi*180)

                            G = 127 + delta *4/np.pi *127

                        #else: ángulo anterior
                    else: # ir recto
                        G = 127
                        newAux = True
                        spdChoice = 0
                    
                else:  # como si no hubiera nada en la imagen, ya que el Aux puede tener ruidoy no sirve para el control
                    cx = cols+int(cols/12) if cx>cols/2 else -int(cols/12)
                    newMain = True
                    spdChoice = 1

                    #print('aquí')

            else:    # nuevo dato de la banda de control principal (newMain = True)
                if newAux: # si hay dato en la banda superior
                    spdChoice = 2
                    # SE USARAN LOS SIGUIENTES DATOS PARA EL CONTROL DE LA VELOCIDAD
                    band_ = edged[int(0.25*rows):int(0.5*rows), :]
                    ang,stdvAng = angleImg(band_,thresholded=True)

                else:
                    spdChoice = 1


                                                                                #  _______
                                                                                # |       |
    elif go:                                                                    # |       |
                                                                                # |_______|
          
        # Teniendo ya datos de Cruce(no hay en este caso), Objeto y Ultrasonidos, decide si hay fallo
        fault, spdCtrl = fc.decide(newObj,False,sn_dist1,sn_dist0) 

        if signalRd:
            signalRd = False
        if crossStop:
            crossStop = False
                
        if inited_pid:
            cx = cols+int(cols/12) if cx>cols/2 else -int(cols/12)
            
            newMain = True
            spdChoice = 1
        else:               # CUANDO NO HEMOS INICIADO PID VAMOS RECTO
            G = 127
            newAux = True
            spdChoice = 0


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
 
        
        
        
        
    # CONTROL
    # ==========================================================================


    # Tomar parámetros de interfaz gráfica
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    spdBase = cv2.getTrackbarPos('spd','ctrl')
    spdUp = cv2.getTrackbarPos('spdUp','ctrl')
    spdMin = cv2.getTrackbarPos('spdMin','ctrl')

    Kp = cv2.getTrackbarPos('Kp','ctrl')
    Td = cv2.getTrackbarPos('Td','ctrl')
    Ti = (cv2.getTrackbarPos('Ti','ctrl') + 0.001)/1000

   
    
    
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Se detiene el guiado
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if not go or spdCtrl==0 or fault or crossStop:   
        #print('stop')
		
		if fault and go:
		    print('fault ', fault)
		
        # Para motores
        if not debug_motion:
            motor.stop()
        
        t1= cv2.getTickCount()
        
        # dentro de este if se puede aprovechar para resetear ciertas variables
        if not go:
            e_i=0 # reset pid integral
            newMain = False
        
        
        
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # Se efectúa el guiado
    #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    else:   
    
    
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # VELOCIDAD
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        if spdChoice == 0:
            spd = spdMin
            
        elif spdChoice == 1:
            spd = spdBase
            
        elif spdChoice == 2:
            #band_ = edged[int(0.25*rows):int(0.5*rows), :]
            #ang,stdvAng = angleImg(band_,thresholded=True)    # GETS ANGLE OF UPPER SECTION
            if not isnan(ang):# and ang != 0:
                angReal = angleReal(centroid=(cx_,cy_+int(rows/4)),angle=ang,
                                    dim=(rows,cols),focal=focal,alpha=alpha,scale=scale)
                #print('angleR {:.2f}'.format(angReal/np.pi*180))
                discrep = abs( (G-128)/256 - (-angReal)/np.pi )
                        # 0~255 >> -128~127 >> -0.5~0.5 // -pi/2~pi/2 >> -0.5~0.5    
                spd = ((1-discrep)*exp(-(stdvAng*2)**2))*spdUp+spdBase
                

        #print(int((0.7*spd + 0.3*spdPrev)*((spdChoice!=2) + spdCtrl*(spdChoice==2))))
        spd = int((0.7*spd + 0.3*spdPrev)*spdCtrl)
        if spd > 0 and spd <spdMin:
            spd = spdMin
        spdPrev = spd 
        #print('spd ', spd)
        
        
        # Orden de velocidad a motores de cc
        if not debug_motion:
            motor.forwardWithSpeed(spd)
        

                
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # PID (Giro de ruedas delanteras)
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        
		
		if not inited_pid: # caso en que el programa se inicie directamente en 'go'
            t1= cv2.getTickCount() + 0.01
		
		
        t2= cv2.getTickCount()
        t=(t2-t1)/cv2.getTickFrequency()+0.0001 # 0.0001 -> tiempo de calculo del PID


         
        if newMain:
            e=(cx-(cols/2))/(cols/2)
            
            if not inited_pid:
                e_prev= e
                e_i=0
                Gi=0
                inited_pid= True
                
            if rst_pid:
                e_prev= e
                rst_pid = False
                
            Gp= e*(Kp)
            
            Gd= (e - e_prev)*(Td)/t
            
            e_i += e*t
            Gi= (e_i)*(1/Ti)
            #print('ganancia', Gi)
            
            G=int(127.5 + Gp + Gd + Gi)
                      
            e_prev=e
			
			 # AntiWindup
			 if G>255:
                G=255
                if e_i>0:           # acción integral en el sentido de la saturación
                    e_i -= e*t
            elif G<0:
                G=0
                if e_i<0:
                    e_i -= e*t
			
            
        elif inited_pid and not rst_pid:
            rst_pid = True
                      

        t1= cv2.getTickCount()
        

        if newMain or newAux:
            G=int(G)
            # Orden de control al servo de las ruedas
            if not debug_motion:
                car_dir.turn(G)


        # ^ aquí acaba el control PID
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        
        
        
        # FILE - guarda imágenes y otros datos en carpeta de destino (root)
        if go and False:
            count_+=1
            if count_==1 and count_img<1000:
                name=root+'sample'+'{:03d}'.format(count_img)+'.png'
                cv2.imwrite(name,frame)
                count_img+=1
                file.write('cx '+'{: 7.2f} '.format(cx)+
                           'G  '+'{: 7.2f} '.format(G)+
                           'Gp '+'{: 7.2f} '.format(Gp)+
                           'ei '+'{: 7.2f} '.format(e_i)+
                           'Gi '+'{: 7.2f} '.format(Gi)+
                           'Gd '+'{: 7.2f} '.format(Gd)+
                           't  '+'{: 7.2f} '.format(t)+'\n')
                #file.write('G '+ '{: 7.2f}'.format(G)+'  P '+
                           #'{: 7.2f}'.format(Gp*(1-weight)+Gp_*weight)+'  D '+
                           #'{: 7.2f}'.format(Gd*(1-weight)+Gd_*weight)+'  I '+
                            #'{: 7.4f}'.format(e_i)+'  O '+
                            #'{: 7.4f}'.format(-angReal/np.pi*64)+'\n')
                count_=0


        # TIME
        #cv2.putText(frame, 'time '+'{:.2f}'.format(float(time)),(10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        print('time {:.3f}'.format(float(t)))

        
        # Representación gráfica de la acción ce control
        if inited_pid:
            cv2.line(ctrl, (int(400/2),int(2)),(int(400/2+(G-127.5)/255*(400/2)),int(2)), (64,255,0), 2)
            cv2.line(ctrl, (int(400/2),int(5)),(int(400/2+(Gp)/255*(400/2)),int(5)), (255,128,64), 2)
            cv2.line(ctrl, (int(400/2),int(8)),(int(400/2+(Gd)/255*(400/2)),int(8)), (0,64,255), 2)
        

        
        
        
    # Mostrar Imágenes
    # ==========================================================================

    bandColor = np.dstack((edged,edged,edged))
    cv2.line(bandColor,(0,int(rows*0.25)),(cols,int(rows*0.25)),(0,255,0))
    cv2.line(bandColor,(0,int(rows*0.5)),(cols,int(rows*0.5)),(0,255,0))
    cv2.line(bandColor,(0,int(rows*0.8)),(cols,int(rows*0.8)),(0,0,255))
    cv2.imshow("main", bandColor)
    #if pred != None:
    #    cv2.circle(frame, (int(pred[0][0]),int(0.9*rows)), 2, (0,255,0))
    cv2.imshow("frame", frame)

    cv2.imshow("ctrl", ctrl)

    cv2.imshow('canny',canny)

    cv2.imshow('objParams',objParams)

    
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        if not debug_sonar:
            sn.stop()
        break
    if k == ord('s'):
        crossWait = not crossWait

    if k == ord('w'):
        fc.warning = False
 

 
# cerrar hilos abiertos y OpenCV
cv2.destroyAllWindows()
vs.stop()
file.close()
time.sleep(1.0)
if not debug_sonar:
    GPIO.cleanup()
