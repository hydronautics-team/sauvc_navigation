
import time
import AStar
import ImageHandler
import Mesh

#from test import *


# robotR - собственный радиус
# fc_in - кадров для инакцивации объектов
# rd - растояние в приделах которого, объекты остаются дольше активными
# fc_c - коэффициент для rd
# sc - коэффициент скорости
# cd - дистанция на который происходит полное преоблаание новых данных над старами
class Map :

    def __init__(self, robotR=0, fc_in=50, rd=2, fc_c=20, sc=1/40, cd=1) :
        self.objects = dict()
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0

        self.robotR = robotR
        self.fc_in = fc_in
        self.rd = rd
        self.fc_c = fc_c
        self.sc = sc
        self.cd = cd

        self.frameCount = 0
        self.totalPath = 0
        self.target = None

        self.lastPosUpdateTime = 0
        self.lastVector = [0, 0, 0]

        self.lastAngle = 0
        self.lastTargetName = ''
        self.lastActiveCount = 0

    # this function update object's name, witch we must go
    def updateTarget(self, targetName):
        if targetName not in ImageHandler.objectData or targetName is None : return
        self.target = targetName

    # this function return: state, position, angle
    # where:
    # state -> current state of finding object, one of:
    #          Mesh.STATE_ERROR     = 0
    #          Mesh.STATE_OK        = 1
    #          Mesh.STATE_COMPLETE  = 2
    #          Mesh.STATE_FIND      = 3
    #          Mesh.STATE_NO_PATH   = 4
    #          Mesh.STATE_FIND_FAIL = 5
    # position -> [surge, pitch, always zero]
    # angle -> yaw
    def calcPath(self):
        needToDraw = True

        targetName = self.target

        circles = self.__getCircles(targetName)

        startPoint = [self.x, self.y, self.z]

        if targetName not in self.objects or not self.objects[targetName].isActive:
            #endPoint=  [0, 0, 0]
            #state, vector, image = Mesh.calcMove(circles, startPoint, [0, 0, self.angle], endPoint, needToDraw, 1, 1)
            #return Mesh.STATE_FIND, vector, image
            return Mesh.STATE_FIND, [0, 0, 0], 0

        r = ImageHandler.objectData[targetName].goodR + self.robotR
        endPoint = [self.objects[targetName].x, self.objects[targetName].y, r]

        state, vector, image = Mesh.calcMove(circles, startPoint, [0, 0, self.angle], endPoint, needToDraw, 1, 1)

        # new danger (but necessary) code start
        angle = vector[4][2] % 360
        angleDistance = min(abs(angle - self.lastAngle), 360 - abs(angle - self.lastAngle))
        activeCount = self.__getActiveObjCount()

        if (angleDistance > 7 and state == Mesh.STATE_OK and self.lastTargetName == targetName
                and self.lastActiveCount == activeCount):
            angleAAA = [0, 0, self.lastAngle]
            # about 5 degrees cost must be < one significant step
            degrees = 5
            step = 0.5
            oneDegCost = step / degrees
            state, vector, image = Mesh.calcMove(circles, startPoint, angleAAA, endPoint, needToDraw,
                                                 1, 1, oneDegCost)
            angle = vector[4][2] % 360

        self.lastAngle = angle
        self.lastTargetName = targetName
        self.lastActiveCount = self.__getActiveObjCount()

        # new danger (but necessary) code end
        self.__intPos(vector)

        speed = vector[5][:]
        angle = vector[4][2]

        return state, speed, angle
        #return state, vector, image

    # this function update current yaw angle on the map
    def updateAngle(self, angle: float):
        self.angle = angle

    # update objects from Bbox array
    def updateObjects(self, bbox_array: BboxArray):
        self.frameCount += 1

        for bbox in bbox_array.bboxes:
            #distance, relativeAngle, name
            if bbox.distance is None: continue
            if bbox.distance < 0 : continue
            self.__updateObject(bbox.distance, bbox.angle, bbox.name)

        for obj in self.objects :
            obj = self.objects[obj]

            distance = Mesh.distance(self.x, self.y, obj.x, obj.y)
            objFC = obj.frameCount + self.fc_in
            objFC_RD = obj.frameCount + self.fc_in * self.fc_c

            if ((objFC < self.frameCount and distance > self.rd)
                    or objFC_RD  < self.frameCount ) :
                obj.isActive = False

    def __updateObject(self, distance, relativeAngle, name) :

        if name not in self.objects : self.objects[name] = Object(name)

        if relativeAngle is None :
            polarAngle = AStar.AStar.getPolarAngle(self.x, self.y, self.objects[name].x, self.objects[name].y)
        else : polarAngle = self.angle - relativeAngle

        value = 1
        if self.x != 0 and self.y != 0 :
            value = 0.3 + (self.frameCount-1 - self.objects[name].frameCount)/5
            if self.totalPath != 0 :
                value = 0.12 + (self.totalPath - self.objects[name].totalPath) / self.cd

        self.objects[name].x = (self.__weightValue(self.objects[name].x, self.x + distance * Mesh.cos(polarAngle), value))
        self.objects[name].y = (self.__weightValue(self.objects[name].y, self.y + distance * Mesh.sin(polarAngle), value))

        self.objects[name].frameCount = self.frameCount
        self.objects[name].totalPath = self.totalPath
        self.objects[name].isActive = True

    def __intPos(self, vector):
        dt = time.time() - self.lastPosUpdateTime
        self.x += self.lastVector[0] * dt * self.sc
        self.y += self.lastVector[1] * dt * self.sc
        self.lastPosUpdateTime = time.time()
        self.lastVector = vector[0][:]

    def __getActiveObjCount(self):
        q = 0
        for obj in self.objects:
            if self.objects[obj].isActive : q += 1
        return q

    def __calcObjectRadius(self, obj) :
        return ImageHandler.objectData.get(obj.name, 1).badR + self.robotR

    def __getCircles(self, targetName):
        circles = []
        for obj in self.objects:
            obj = self.objects[obj]
            if obj.isActive and obj.name != targetName:
                circles.append([obj.x, obj.y, self.__calcObjectRadius(obj)])

        return circles

    def __weightValue(self, a, b, value):
        if value < -1: value = -1
        if value > 1: value = 1

        if b < a:
            a, b = b, a
            value = 1 - value

        return a + (b - a) * value

class Object :

    def __init__(self, name) :
        self.name = name
        self.x = 0
        self.y = 0
        self.z = 1
        self.isActive = False
        self.frameCount = 0
        self.totalPath = 0

    def __str__(self):
        return '[' + str(self.x) + ', ' + str(self.y) + '] (' + str(self.isActive) + ')'

    def __repr__(self):
        return str(self)