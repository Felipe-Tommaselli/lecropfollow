import numpy as np
import math
from trajectory_utils2 import *

def isSignChange(a, b):
   if((a > 0 and b < 0) or (a < 0 and b > 0)):
       return True
   return False
   
def runningMeanFast(p, N):
    return np.convolve(p, np.ones((N,))/N, mode='valid')

def dlatlon2dxy(lat1, lon1, lat2, lon2):
    R = 6371000
    rlat1 = lat1*math.pi/180
    rlat2 = lat2*math.pi/180
    rlon1 = lon1*math.pi/180
    rlon2 = lon2*math.pi/180
    
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    
    dx = R*dlon*math.cos((rlat1+rlat2)/2)
    dy = R*dlat
    
    return (dx, dy)
    
def dxy2dlatlon(startLat, startLon, dx, dy):
    R = 6371000    
    dlat = (dy/R) * (180 / math.pi)
    rlat1 = startLat * math.pi / 180
    rlat2 = (startLat + dlat)*math.pi/180
    refangle = (rlat1+rlat2)/2
    # print 'refange'
    # print refangle
    dlon = (dx/(R*math.cos(refangle)))*(180/math.pi)    
    return (dlat, dlon)    
    
def dxy2dlatlonEst(dx, dy):
    R = 6371000
    dlat_tmp = dy/(R*(math.pi/180))
    return (dlat_tmp, dx / (R*math.cos(dlat_tmp/180)*(math.pi/180)))
    
def dxy2dlatlonWRONG(startLat, startLon, dx, dy):
    R = 6371000
    (tdlat, tdlon) = dxy2dlatlonEst(dx, dy)
    lat1 = startLat
    lon1 = startLon
    lat2 = startLat + tdlat
    lon2 = startLon + tdlon
    
    rlat1 = lat1*(math.pi/180)
    rlat2 = lat2*(math.pi/180)
    rlon1 = lon1*(math.pi/180)
    rlon2 = lon2*(math.pi/180)
    
    tdlat = rlat2 - rlat1;
    tdlon = rlon2 - rlon1;
    
    return (dy/(R*(180/math.pi)), dx/(R*math.cos((rlat1+rlat2)/2)*(180/math.pi)))
 
def interpolateTrajectory(x, y, spacing):

    count = len(x)
    
    dx = x[count - 1] - x[1]
    dy = y[count - 1] - y[1]

    space = spacing

    distance = (dx*dx + dy*dy)**(1/2.0)
    
    num = distance / space + 1

    if(abs(dx) > abs(dy)):
        xx = np.linspace(0, x[-1], num)
        yy = np.interp(xx, x, y)
    else:
        yy = np.linspace(0, y[-1], num)
        xx = np.interp(yy, y, x)

    return (xx, yy)
    
def interpolateTrajectory2(x, y, spacing):

    count = len(x)

    dispX = x[count-1] - x[0]
    dispY = y[count-1] - y[0]

    distX=x[0]
    distY=y[0]

    duplicate = []
    filteredX = runningMeanFast(x, 16)
    filteredY = runningMeanFast(y, 16)
    breakPoints = [0]
    breakPointCount = 1
    prevdx = filteredX[1] - filteredX[0]
    prevdy = filteredY[1] - filteredY[0]

    x_priority = 0
    y_priority = 1
    interp_priority = -1
    previous_interp_priority = -1
    for i in range(1, filteredX.size - 1):
        dx = filteredX[i] - filteredX[i-1]
        dy = filteredY[i] - filteredY[i-1]
        
        if(abs(dy) > abs(dx)):
            interp_priority = y_priority
        elif(abs(dx) > abs(dy)):
            interp_priority = x_priority
        elif(interp_priority == -1):
            interp_priority = x_priority
        if(interp_priority != previous_interp_priority):            
            breakPointCount = breakPointCount + 1
            breakPoints.append(i)
            
        previous_interp_priority = interp_priority    
        
        #if(isSignChange(prevdx, dx) == True and abs(filteredX[i] - filteredX[breakPoints[breakPointCount-1]]) > abs(filteredY[i] - filteredY[breakPoints[breakPointCount-1]])):
        #    breakPointCount = breakPointCount + 1
        #    breakPoints.append(i)
        #elif(isSignChange(prevdy, dy) == True and abs(filteredX[i] - filteredX[breakPoints[breakPointCount-1]]) > abs(filteredY[i] - filteredY[breakPoints[breakPointCount-1]])):
        #    breakPointCount = breakPointCount + 1
        #    breakPoints.append(i)   
        #prevdx = dx
        #prevdy = dy     
        
    if(breakPoints[breakPointCount-1] != count-1):
        breakPointCount = breakPointCount + 1
        breakPoints.append(count-1)

    # print breakPoints

    xout = []
    yout = []
    space = spacing
    for i in range(0, breakPointCount-1):
        start = breakPoints[i]
        end = breakPoints[i+1]

        dispX = x[end] - x[start]
        dispY = y[end] - y[start]
        
        distance = (dispX*dispX + dispY*dispY)**(1/2.0)    
        
        num = distance / space + 1    
        
        if(abs(dispX) > abs(dispY)):
            if(dispX < 0):
                xin = x[start:end+1][::-1]
                yin = y[start:end+1][::-1]
            else:
                xin = x[start:end+1]
                yin = y[start:end+1]
            xx = np.linspace(xin[0], xin[end-start], num)
            yy = np.interp(xx, xin[0:], yin[0:])
            if(dispX < 0):
                xx = xx[::-1]            
                yy = yy[::-1]
        else:
            if(dispY < 0):
                xin = x[start:end+1][::-1]
                yin = y[start:end+1][::-1]
            else:
                xin = x[start:end+1]
                yin = y[start:end+1]
            yy = np.linspace(yin[0], yin[end-start], num)
            xx = np.interp(yy, yin[0:], xin[0:])
            if(dispX < 0):
                xx = xx[::-1]            
                yy = yy[::-1]
        xout.extend(xx[1:])
        yout.extend(yy[1:])

    return (xx, yy)
    
def interpolateOriginedTrajectory(x, y, spacing, originLat, originLon):
    (xx, yy) = interpolateTrajectory(x, y, spacing)

    leftout = []
    rightout = []
    leftout.append(originLat)
    rightout.append(originLon)

    leftout.extend(xx)
    rightout.extend(yy)
        
    return (leftout,rightout)    
    
def removeTrajectoryOutliers(x, y, maxHeadingChangeDegrees):
    origin = [0,0]
    angle = 0
    maxHeadingChangeRadians = maxHeadingChangeDegrees*math.pi/180
    p0 = [0,0]
    p1 = [0,0]
    p2 = [0,0]
    for i in range(1, x.size-2):
        p0 = [x[i-1],y[i-1]]
        p1 = [x[i],y[i]]
        p2 = [x[i+1],y[i+1]]
        angle1 = getAngleBetweenPoints(origin, p0, p1)
        angle2 = getAngleBetweenPoints(origin, p1, p2)
        angle = angle2 - angle1
        if(abs(angle) > abs(maxHeadingChangeRadians)):
            pM = getMidPoint2D(p0, p1)
            x[i]= pM[0]
            y[i] = pM[1]
            
    return  (x, y)        
        
def smoothTrajectory(x, y, samplesAverage):
    x = runningMeanFast(x, samplesAverage)
    y = runningMeanFast(y, samplesAverage)
    return (x, y)

def smoothOriginedTrajectory(x, y, samplesAverage, originLat, originLon):
    (xx, yy) = smoothTrajectory(x, y, samplesAverage)
    leftout = []
    rightout = []
    leftout.append(originLat)
    rightout.append(originLon)

    leftout.extend(xx)
    rightout.extend(yy)
        
    return (leftout,rightout)  