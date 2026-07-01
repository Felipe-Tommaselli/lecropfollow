from trajectory_utils import *
import math
import numpy as np

def allocateList(x, typeitem, size):
    for i in range(1,size):
        x.append(typeitem)

    return x    
    
def getMidPoint2D(p1, p2):
    pM = [0,0]
    pM[0] = (p1[0] + p2[0]) / 2
    pM[1] = (p1[1] + p2[1]) / 2
    
    return pM
    
def getAngleBetweenPoints(origin, p2):
    return math.atan2(p2[1] - origin[1], p2[0] - origin[0])    


def getRadialPoint(origin, angle, radius):
    point = [0,0]
    point[0] = origin[0] + radius*math.cos(angle)
    point[1] = origin[1] + radius*math.sin(angle)
    
    return point

def getPrecisionDistance2D(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    
    return math.sqrt(dx*dx + dy*dy)
    
def getPrecisionDistance(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def populateArcConnection(pointSpacing, maxArcRadius, initialHeadingRadians, p1, p2,x,y,startindex, includeEndPoints):
    # print 'arc connection'
    count = 0
    i = 0
    index = 0
    angleRadians = 0
    radialPoint = [0,0,0]
    entranceJoint = [0,0,0]
    exitJoint = [0,0,0]
    entranceArcEnd = [0,0,0]
    exitArcEnd = [0,0,0]
    quarter_arc_length = 0
    straightAwayLength = 0
    traightAwayPoints = 0
    arcPoints = 0
    offset = 1
    headingRadians = initialHeadingRadians
    rotationDirection = 1# 1 is increasing angle -1 is decreasing angle

    dist = getPrecisionDistance2D(p1, p2)
    
    # print dist
    # print maxArcRadius
    
    if(dist < 2* maxArcRadius):
        return -1    

    straightAwayLength = dist - 2*maxArcRadius
    straightAwayPoints = int(straightAwayLength / pointSpacing)
    quarter_arc_length = 2*math.pi/4*maxArcRadius
    arcPoints = int(quarter_arc_length / pointSpacing)
    count = 2*arcPoints + straightAwayPoints - 6
    
    if(includeEndPoints):
        offset = 0
        count = count + 2
    
    arcSpacing = math.pi/2/arcPoints
    angleRadians = getAngleBetweenPoints(p1, p2)
    # print angleRadians

    if(headingRadians - angleRadians > 0):
        rotationDirection = -1    
    else:
        rotationDirection = 1    

    exitJoint = getRadialPoint(p1, angleRadians, maxArcRadius)

    for i in range (offset, arcPoints):
        radialPoint = getRadialPoint(exitJoint, -angleRadians + rotationDirection*i*arcSpacing, maxArcRadius)    
        x[startindex+i-offset] = radialPoint[0]
        y[startindex+i-offset] = radialPoint[1]
    
    exitArcEnd[0] = radialPoint[0]
    exitArcEnd[1] = radialPoint[1]

    #rotationDirection = -rotationDirection
    
    entranceJoint = getRadialPoint(p2, -angleRadians, maxArcRadius)

    index = count-1
    
    for i in range (offset, arcPoints):
        radialPoint = getRadialPoint(entranceJoint, angleRadians - rotationDirection*i*arcSpacing, maxArcRadius)
        x[startindex+index-i-offset] = radialPoint[0]
        y[startindex+index-i-offset] = radialPoint[1]    

    entranceArcEnd[0] = radialPoint[0]
    entranceArcEnd[1] = radialPoint[1]

    index = startindex + arcPoints - offset
    
    n = populateLineConnection(pointSpacing, exitArcEnd, entranceArcEnd, x, y, index, False)

    # print 'exit arc connection'
    
    return count

def generateLineTrajectory(length, angleDegrees, spacing, offsetX = 0, offsetY = 0):

    numPoints = int(length / spacing)

    dx = spacing*math.cos(angleDegrees*math.pi/180)
    dy = spacing*math.sin(angleDegrees*math.pi/180)
    x = []
    y = []

    for i in range (1, numPoints):
        nx = 0 + i * dx
        ny = 0 + i * dy
        x.append(nx+offsetX)
        y.append(ny+offsetY)
        
    X = np.transpose((x,y))    
    return X
    
def populateLineConnection(pointSpacing, p1, p2,x,y,startindex,includeEndPoints):
    index = 1
    # print p1
    # print p2
    dist = getPrecisionDistance2D(p1, p2)

    num_segments = int(dist/pointSpacing)
    # print dist
    # print pointSpacing
    count = num_segments - 1
    
    if(includeEndPoints):
        index = 0
        count = num_segments+1
    
    dx = (p2[0] - p1[0])/num_segments
    dy = (p2[1] - p1[1])/num_segments

    for i in range (index, count+index):
        x[startindex+i-index] = p1[0] + dx*i
        y[startindex+i-index] = p1[1] + dy*i    

    return count

    #row is an array of (lat,lng)
def traversalLength(row, numPoints):
    dx = 0
    dy = 0
    len = 0    
    i = 0    
    p1 = [0,0]    
    p2 = [0,0]    
    for i in range(0,numPoints-2):
        p1 = row[i]   
        p2 = row[i+1]    
        (dx, dy) = dlatlon2dxy(p1[0],p1[1],    p2[0],p2[1])    

        len = len + (dx*dx + dy*dy)**.5        

    return len    
    
    
def getRowFirstPoint(row):
    return row[0]
    
def getRowLastPoint(row):
    return row[-1]
        
    #row and rownext are arrays of (lat, lng)
def startToStartDistanceFromRowToRow(row, rownext):
    p1 = getRowFirstPoint(row)
    p2 = getRowFirstPoint(rownext)
    (dx, dy) = dlatlon2dxy(p1[0], p1[1], p2[0], p2[1])    

    return (dx*dx + dy*dy)**.5    
    
    
def endToEndDistanceFromRowToRow(row, rownext):
    p1 = getRowLastPoint(row)
    p2 = getRowLastPoint(rownext)
    (dx, dy) = dlatlon2dxy(p1[0], p1[1], p2[0], p2[1])    

    return (dx*dx + dy*dy)**.5    
    
    
def startToEndDistanceFromRowToRow(row, rownext):
    p1 = getRowFirstPoint(row)
    p2 = getRowLastPoint(rownext)
    (dx, dy) = dlatlon2dxy(p1[0], p1[1], p2[0], p2[1])    

    return (dx*dx + dy*dy)**.5    
    
    
def endToStartDistanceFromRowToRow(row, rownext):
    p1 = getRowLastPoint(row)
    p2 = getRowFirstPoint(rownext)
    (dx, dy) = dlatlon2dxy(p1[0], p1[1], p2[0], p2[1])    

    return (dx*dx + dy*dy)**.5    

# def generateTrajectoryFromRow():
#     print 'here'

# def generateRowTrajectories():
#     print 'here'
    
def generateRows(startlat, startlon, angle, numrows, spacing, rowspacing, rowlength):
    
    startlats = []
    startlons = []
    endlats = []
    endlons = []
    rows=[]

    angle = angle * math.pi / 180
    #cos(angle) = x / rowspacing
    #sin(angle) = y / rowspacing

    #rowspacing = sqrt(dx^2 + dy^2) = 

    dx = math.cos(angle+math.pi/2)*rowspacing
    dy = math.sin(angle+math.pi/2)*rowspacing
    rx = math.cos(angle)*rowlength
    ry = math.sin(angle)*rowlength

    (dlat, dlon) = dxy2dlatlon(startlat, startlon, rx, ry)
    endlat = startlat + dlat
    endlon = startlon + dlon

    startlats.append(startlat)
    startlons.append(startlon)
    endlats.append(endlat)
    endlons.append(endlon)

    for i in range(1, numrows):
        (dlat, dlon) = dxy2dlatlon(startlat, startlon, dx*i, dy*i)
        # print dlat 
        # print dlon
        startlats.append(startlat+dlat)
        startlons.append(startlon+dlon)
        (dlat, dlon) = dxy2dlatlon(endlat, endlon, dx*i, dy*i)
        # print dlat 
        # print dlon
        endlats.append(endlat+dlat)
        endlons.append(endlon+dlon)
        
    points_per_row = int(rowlength / spacing)    
        
    for i in range(0, numrows-1):
        ddlat = (endlats[i] - startlats[i]) / points_per_row
        ddlon = (endlons[i] - startlons[i]) / points_per_row
        rows.append([])
        rows[i].append([])
        rows[i].append([])
        # print endlons[i]
        # print startlons[i]
        # print points_per_row
        # print ddlat
        # print ddlon
        for j in range(1, points_per_row):
            rows[i][0].append(startlats[i] + ddlat*j)
            rows[i][1].append(startlons[i] + ddlon*j)
        
        row = np.transpose((rows[i][0],rows[i][1]))

    return rows    