import numpy as np
import cv2
from math import tan, cos, sin, exp, atan


def angleImg(section, thresholded=False):

    if not thresholded:
        
        section = cv2.GaussianBlur(section,(7,7),0)
        ret,section = cv2.threshold(blur,185,255,cv2.THRESH_BINARY)


    gx = cv2.Sobel(section, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(section, cv2.CV_32F, 0, 1, ksize=3)

    mag, ang = cv2.cartToPolar(gx, gy)
    mag = mag[3:-3,3:-3]
    ang = ang[3:-3,3:-3]

    ang = ang[mag != 0]

    ang %= np.pi
    ang = np.where(ang > np.pi/2-0.01, np.pi - ang, -ang)
    
    mean=np.mean(ang)
    #mean= np.pi-mean if mean>np.pi/2 else -mean

    return mean, np.var(ang)
            #cv2.meanStdDev(ang)  # probar (return: -> mean, stdev)

# scale = cols/640
def angleReal(centroid,angle,dim,focal,alpha,scale=None):

    x,y = centroid
    rows,cols = dim
    
    if angle!=0:
        slope = 1/tan(angle)
        #print('slope ', slope)
        
        if scale == None:
            scale = cols/640

        y_inf= focal * tan(alpha) * scale
        y__inf= -y_inf + rows/2
        #print('yinf',y__inf)
        
        lefty = int((-x*slope) + y)     # line cut at x=0
        #print('lefty', lefty)
                
        # PROCESS PARAMETERS
        x__inf = (y__inf - lefty)/(slope)
        x_inf= x__inf - cols/2   # offset in x from center

        #print('xinf', x_inf)
    else:
        x_inf=x-cols

    # THETA 
    theta = -atan(x_inf/(focal*scale)*cos(alpha))
    return theta

