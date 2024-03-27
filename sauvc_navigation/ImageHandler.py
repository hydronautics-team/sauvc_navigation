
import math
import Mesh

PIXEL_RESOLUTION_X = 640
PIXEL_RESOLUTION_Y = 480

ANGLE_RESOLUTION_X = 90
ANGLE_RESOLUTION_Y = ANGLE_RESOLUTION_X * (PIXEL_RESOLUTION_Y / PIXEL_RESOLUTION_X)

MIN_P_VALUE = 0.75

MAX_RATIO_DIFFERENCE = 1.2

class ObjectDataSegment :

    def __init__(self, width, height, maxRatio, badR, goodR, p, color):
        self.color = color
        self.goodR = goodR
        self.badR = badR
        self.maxRatio = maxRatio
        self.height = height
        self.width = width
        self.p = p

# горизонтальный размер в метрах
# вертикальный размер в метрах
# радиус в метрах
# цвет на карте
objectData = {
    'gate' : ObjectDataSegment(2.58, 2.64, 1.4, 2, 1.5, 0.85, 'blue'),
    'yellow_flare' : ObjectDataSegment(0.2 * 5, 3.9, 1.8, 1, 1, 0.75, 'yellow'),
    'red_flare' : ObjectDataSegment(0.2 * 4, 3.9 /2,1.8, 1,  1, 0.75, 'red'),
    'blue_bowl' : ObjectDataSegment(0.7, 0.35,1.4, 4,  0.8, 0.85, 'blue'),
    'red_bowl' : ObjectDataSegment(0.7, 0.35,1.4, 4,  0.8, 0.85, 'red')
}

def tg(x) :
    return math.tan(x/360*2*math.pi)

def calcDistance(angleSize, realSize) :
    #return realSize / Mesh.sin(angleSize)
    return realSize / (2 * tg(angleSize / 2))
    #return realSize / (1 * tg(angleSize / 1))
    #return realSize / Mesh.sin(angleSize)
    #angleSize = 45
    #return realSize / (angleSize/360*2*math.pi)
    #return realSize / (2 * tg(angleSize / 1))

class Object :

    def __init__(self, name, p=0.0, x1=0, y1=0, x2=0, y2=0):
        self.name = name
        self.p = p
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __str__(self):
        return self.name + '->' + str(round(self.p, 2))

    def __repr__(self):
        return str(self)

def parseData(data) :
    #print(data)
    #input()

    videoData = []
    frameData = []

    for i in range(len(data)) :
        i = data[i]

        if i == 'new img' :
            videoData.append(frameData)
            frameData = []
            continue

        q = i.split()

        name = q[0]
        p = float(q[1])
        x1 = int(q[2])
        y1 = int(q[3])
        x2 = int(q[4])
        y2 = int(q[5])

        frameData.append(Object(name, p, x1, y1, x2, y2))

    print(videoData)
    return videoData

def calcDistanceAndAngle(frameData, image) :

    objectPosition = []

    for object in frameData :

        if object.name == 'gate_qualification' :
            object.name = 'gate'

        if objectData.get(object.name, None) == None : 
            objectPosition.append([-1, 0, name])
            continue

        #rint('adasd')
        #print(frameData)
        #input()

        if object.p < objectData[object.name].p :
            objectPosition.append([-1, 0, name])
            continue

        name = object.name
        realSizeX, realSizeY = objectData[object.name].width, objectData[object.name].height

        pixelSizeX = object.x2 - object.x1
        pixelSizeY = object.y2 - object.y1
        sizeX = pixelSizeX / PIXEL_RESOLUTION_X
        sizeY = pixelSizeY / PIXEL_RESOLUTION_Y
        angleSizeX = sizeX * ANGLE_RESOLUTION_X
        angleSizeY = sizeY * ANGLE_RESOLUTION_Y

        calcRatio = sizeX / sizeY
        realRatio = realSizeX / realSizeY

        DX = calcDistance(angleSizeX, realSizeX)
        DY = calcDistance(angleSizeY, realSizeY)

        print('Ratio:', calcRatio, realRatio, max(calcRatio, realRatio) / min(calcRatio, realRatio),
              objectData[object.name].maxRatio)
        print('Size:', sizeX, sizeY, realSizeX, realSizeY)
        print('Distance:', DX, DY)

        if (max(calcRatio, realRatio) / min(calcRatio, realRatio)) > objectData[object.name].maxRatio :
            print('NO')
            objectPosition.append([-1, 0, name])
            continue

        D = DX
        print(object, object.x1, object.y1, object.x2, object.y2)
        if (object.y1 == 0 or object.y2 == PIXEL_RESOLUTION_Y) and (object.x1 == 0 or object.x2 == PIXEL_RESOLUTION_X):
            print('NO')
            objectPosition.append([-1, 0, name])
            continue
        if (object.x1 == 0 or object.x2 == PIXEL_RESOLUTION_X):
            print('NO')
            objectPosition.append([-1, 0, name])
            continue
        if (object.x1 != 0 and object.x2 != PIXEL_RESOLUTION_X) :
            D = DX
            #print('DX')
        if (object.y1 != 0 and object.y2 != PIXEL_RESOLUTION_Y) :
            D = DY
            #print('DY')
        if (object.x1 != 0 and object.x2 != PIXEL_RESOLUTION_X) and (object.y1 != 0 and object.y2 != PIXEL_RESOLUTION_Y):
            D = (DX + DY) / 2
            #print('DXY')

        #if (sizeY != 0 and sizeY != 1): D = DY
        #if (sizeX != 0 and sizeX != 1): D = DX
        #if (sizeX != 0 and sizeX != 1 and sizeY != 0 and sizeY != 1) : D = (DX + DY) / 2

        positionX = object.x1 + (object.x2 - object.x1) / 2
        if object.x1 == 0 : positionX = object.x1 + (object.x2 - object.x2/2) / 2
        if object.x2 == PIXEL_RESOLUTION_X: positionX = object.x1 + (object.x2 - object.x1)*2 / 2
        angleX = (positionX / PIXEL_RESOLUTION_X) * ANGLE_RESOLUTION_X
        angle = angleX - ANGLE_RESOLUTION_X / 2

        if object.x2 == PIXEL_RESOLUTION_X or object.x1 == 0 : angle = None


        objectPosition.append([D, angle, name])

    # расстояние до объекта (в метрах)
    #distance = 15
    # угол до объекта (относительно "линни прямо" в правую сторону в градусах)
    #relativeAngle = -20 # влево на 20 градусов
    # название объекта
    #name = 'Врата'

    #return [[distance, relativeAngle, name]]
    #print(objectPosition)

    return objectPosition

