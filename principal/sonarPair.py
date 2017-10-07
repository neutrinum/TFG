from threading import Thread
import RPi.GPIO as GPIO                    #Import GPIO library
import time                                #Import time library


class Sonar:
    def __init__(self, trig0=23, echo0=24, trig1=5, echo1=6, pair=True):
        
        #GPIO.setmode(GPIO.BCM)                     

        self._TRIG0 = trig0                                
        self._ECHO0 = echo0

        GPIO.setup(self._TRIG0,GPIO.OUT)                  #Set pin as GPIO out
        GPIO.setup(self._ECHO0,GPIO.IN)

        self.distance0 = self.distPrev0 = self.distError0 = -1

        self.pair = pair
        if self.pair:
            self._TRIG1 = trig1                                  
            self._ECHO1 = echo1
            
            GPIO.setup(self._TRIG1,GPIO.OUT)                  #Set pin as GPIO out
            GPIO.setup(self._ECHO1,GPIO.IN)

            self.distance1 = self.distPrev1 = self.distError1 = -1
        else:
            self.distance1 = self.distPrev1 = self.distError1 = None

        self.warning0 = False
        self.warning1 = False
        
        self.stopped = False
        self.period = 0.2 if not self.pair else 0.1


    def start(self):
        Thread(target=self.update, args=()).start()
        
 
    def update(self):
        while True:
            #print('izq ', self.distance1, 'der ',self.distance0)
        
            if self.stopped:
                #GPIO.cleanup()
                return
                
            # SENSOR DE LA DERECHA
            # - - - - - - - - - - - - - - - - - - - - - - - - -
            if not self.stopped:
                GPIO.output(self._TRIG0, False)                 #Set TRIG as LOW
             
            time.sleep(self.period)                            #Delay 

            if not self.stopped:
                GPIO.output(self._TRIG0, True)                  #Set TRIG as HIGH
                time.sleep(0.00001)                      #Delay of 0.00001 seconds
                GPIO.output(self._TRIG0, False)                 #Set TRIG as LOW


                t = time.time()
                pulse_start = t + 0.0001
                while GPIO.input(self._ECHO0)==0 and pulse_start-t <0.0005:               #Check whether the ECHO is LOW
                    pulse_start = time.time()              #Saves the last known time of LOW pulse

                #pulse_end=pulse_start+0.01
                while GPIO.input(self._ECHO0)==1:               #Check whether the ECHO is HIGH
                    pulse_end = time.time()                #Saves the last known time of HIGH pulse 

                pulse_duration = pulse_end - pulse_start #Get pulse duration to a variable

                distance0 = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance
                distance0 = round(distance0, 2)            #Round to two decimal points
                #print('raw DER ', distance0)

                if self.warning0:
                    if self.distError0 == -1:
                        self.distError0 = distance0
                    
                    if distance0 == self.distError0:
                        distance0 = self.distPrev0
                    else:
                        self.warning0 = False
                        self.distError0 = -1

                if distance0 < 2:      #Check whether the distance is within range
                    #distance0 = -1  #Print distance with 0.5 cm calibration
                    distance0 = self.distPrev0
                    self.warning0 = True
                    #print('\nwarnDerecha\n')
                    
                elif distance0 > 400:      #Check whether the distance is within range
                    #distance0 = -2  #Print distance with 0.5 cm calibration
                    distance0 = self.distPrev0
                    self.warning0 = True
                    #print('\nwarnDerecha\n')

                self.distPrev0 = self.distance0 = distance0

                

            # SENSOR DE LA IZQUIERDA
            # - - - - - - - - - - - - - - - - - - - - - - - - -
            if self.pair:
                
                if not self.stopped:
                    GPIO.output(self._TRIG1, False)                 
                 
                time.sleep(self.period)                           

                if not self.stopped:
                    GPIO.output(self._TRIG1, True)                
                    time.sleep(0.00001)                     
                    GPIO.output(self._TRIG1, False)              


                    t = time.time()
                    pulse_start = t + 0.0001
                    while GPIO.input(self._ECHO1)==0 and pulse_start-t <0.0005:       
                        pulse_start = time.time()        

                    #pulse_end=pulse_start+0.01
                    while GPIO.input(self._ECHO1)==1:              
                        pulse_end = time.time()                 

                    pulse_duration = pulse_end - pulse_start

                    distance1 = pulse_duration * 17150  
                    distance1 = round(distance1, 2)
                    #print('raw IZQ ', distance1)

                    if self.warning1:
                        if self.distError1 == -1:
                            self.distError1 = distance1
                        
                        if distance1 == self.distError1:
                            distance1 = self.distPrev1
                        else:
                            self.warning1 = False
                            self.distError1 = -1

                    if distance1 < 2:      
                        #distance1 = -1
                        distance1 = self.distPrev1
                        self.warning1 = True
                        #print('\nwarnIzquierda\n')
                    elif distance1 > 400:      
                        #distance1 = -2
                        distance1 = self.distPrev1
                        self.warning1 = True
                        #rint('\nwarnIzquierda\n')

                    self.distPrev1 = self.distance1 = distance1

 
    def read(self):
        # return the frame most recently read
        return (self.distance0,self.distance1)
 
 
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

        
                




