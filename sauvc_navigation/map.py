import random
import time

import AStar
import ImageHandler
import Mesh
from PIL import Image, ImageDraw
from stingray_interfaces.msg import BboxArray, Bbox

# растояние в приделах которого, объекты остаются дольше активными
REMEMBER_DISTANCE = 2

# множитель для радиуса в случае конечной точки
COMPLETE_DISTANT_COEF = 1

# целевая глубина
POOL_TARGET_DEEP = 25
# кадров для инакцивации объектов
FRAME_COUNT_TO_INACTIVE = 50
# коэффициент для REMEMBER_DISTANCE
FRAME_COUNT_COEF = 20

# логи
LOG_SAVE = False
LOG_SAVE_FILENAME = 'NTB_log.txt'

# наш радиус
SELF_RADIUS = 0.1

# если не знаем радиус объекта
DEFAULT_OBJECT_RADIUS = 2

# дистанция на который происходит полное преоблаание новых данных над старами
FILL_CHANGE_DISTANCE = 1

# растояние до центра при поиске
FIND_CENTER_DISTANCE = 3

# for new dander code
MAX_VIBE_ANGLE = 7
VIBE_CHECK_DISTANCE = 2

class Vector :

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __abs__(self):
        return (self.x**2 + self.y**2 + self.x**2)**0.5

    def __str__(self):
        return '(' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.z) + ')'


class Object :

    def __init__(self, name) :
        self.name = name
        #self.pos = Vector(0, 0, 0)
        self.x = 0
        self.y = 0
        self.z = POOL_TARGET_DEEP
        self.isActive = False
        self.frameCount = 0
        self.totalPath = 0

    def __str__(self):
        return '[' + str(self.x) + ', ' + str(self.y) + '] (' + str(self.isActive) + ')'

    def __repr__(self):
        return str(self)


def weightValue(a, b, value) :
    if value < -1: value = -1
    if value > 1 : value = 1

    if b < a :
        a, b = b, a
        value = 1 - value

    return a + (b - a) * value

FS_START_FIND = 0
FS_GO_TO_OBJECT = 1
FS_FIRST_CIRCLE_HALF = 2
FS_FIRST_CIRCLE_FULL = 3
FS_TO_CENTER = 4
FS_SECOND_CIRCLE_HALF = 5
FS_SECOND_CIRCLE_FULL = 6

class Map :

    def __init__(self, position, angle) :
        self.objects = dict()
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]
        self.angle = angle
        self.frameCount = 0
        self.totalPath = 0
        self.target = None

        self.findFlag_1 = False
        self.findVector = 0
        self.findState = FS_START_FIND
        self.lastRolAngle = 0

        self.lastAngle = 0
        self.lastTurn = 0
        self.lastSpeedAbs = 0
        self.lastTargetName = ''


        if LOG_SAVE:
            self.file_imageData = open(LOG_SAVE_FILENAME, 'a')
            self.file_posData = open(LOG_SAVE_FILENAME + '2', 'a')
            self.file_imageData.write('\nSTART on ' + str(round(time.time(), 2)))
            self.file_posData.write('\nSTART on ' + str(round(time.time(), 2)))


    @staticmethod
    def sendVector(vector):
        speed = vector[4][:]
        angle = vector[3][:]
        pass

    def update(self, bbox_array: BboxArray):
        self.frameCount += 1

        for bbox in bbox_array.bboxes:
            #distance, relativeAngle, name
            self.__updateObject(bbox.distance, bbox.angle, bbox.name)

        for obj in self.objects :
            obj = self.objects[obj]

            distance = Mesh.distance(self.x, self.y, obj.x, obj.y)
            objFC = obj.frameCount + FRAME_COUNT_TO_INACTIVE
            objFC_RD = obj.frameCount + FRAME_COUNT_TO_INACTIVE * FRAME_COUNT_COEF

            if ((objFC < self.frameCount and distance > REMEMBER_DISTANCE)
                    or objFC_RD  < self.frameCount ) :
                obj.isActive = False

    @staticmethod
    def updateMap(map, data, image):
        #print(data)
        #input()
        objectData = ImageHandler.calcDistanceAndAngle(ImageHandler.parseData(data), image)

        if LOG_SAVE:
            map.file_imageData.write('\n' + '\n'.join(['new img'] + data))
            #file = open(LOG_SAVE_FILENAME, 'a')
            #file.write('\n'.join(data))
            #file.close()

        map.updateObject(objectData)

    @staticmethod
    def updatePositionMap(map, pos, angle3D):

        if LOG_SAVE:
            #file = open(LOG_SAVE_FILENAME + '_2', 'a')
            s = str(pos) + '; ' + str(angle3D) + '\n'
            #map.file_posData.write(','.join(list(map(lambda x : str(x), pos))) + ' ' + str(map.frameCount) + '\n')
            map.file_posData.write(s)
            #file.close()

        map.updatePosition(pos, angle3D)

    def updatePosition(self, pos, angle3D):

        self.totalPath += ((pos[0] - self.x) ** 2 + (pos[1] - self.y) ** 2 + (pos[2] - self.z) ** 2) ** 0.5

        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.angle = angle3D

    def updateObject(self, objectList) :

        #print(self.objects)

        self.frameCount += 1

        for obj in objectList :
            #distance, relativeAngle, name
            self.__updateObject(obj[0], obj[1], obj[2])

        for obj in self.objects :
            obj = self.objects[obj]

            distance = Mesh.distance(self.x, self.y, obj.x, obj.y)
            objFC = obj.frameCount + FRAME_COUNT_TO_INACTIVE
            objFC_RD = obj.frameCount + FRAME_COUNT_TO_INACTIVE * FRAME_COUNT_COEF

            if ((objFC < self.frameCount and distance > REMEMBER_DISTANCE)
                    or objFC_RD  < self.frameCount ) :
                obj.isActive = False

    def __updateObject(self, distance, relativeAngle, name) :

        if name not in self.objects : self.objects[name] = Object(name)

        if relativeAngle is None :
            polarAngle = AStar.AStar.getPolarAngle(self.x, self.y, self.objects[name].x, self.objects[name].y)
        else : polarAngle = self.angle[2] - relativeAngle
        #self.objects[name].x = self.x + distance * Mesh.cos(polarAngle)
        #self.objects[name].y = self.y + distance * Mesh.sin(polarAngle)

        value = 1
        if self.x != 0 and self.y != 0 :
            value = 0.3 + (self.frameCount-1 - self.objects[name].frameCount)/5
            if self.totalPath != 0 :
                value = 0.12 + (self.totalPath - self.objects[name].totalPath) / FILL_CHANGE_DISTANCE

        print('value:', value)
        self.objects[name].x = (weightValue(self.objects[name].x, self.x + distance * Mesh.cos(polarAngle), value))
        self.objects[name].y = (weightValue(self.objects[name].y, self.y + distance * Mesh.sin(polarAngle), value))

        self.objects[name].frameCount = self.frameCount
        self.objects[name].totalPath = self.totalPath
        self.objects[name].isActive = True

    @staticmethod
    def calcObjectRadius(obj) :
        #print(ImageHandler.objectData.get(obj.name, DEFAULT_OBJECT_RADIUS)[2] + SELF_RADIUS)
        return ImageHandler.objectData.get(obj.name, DEFAULT_OBJECT_RADIUS).badR + SELF_RADIUS

    def initRotating(self):
        self.lastRolAngle = self.angle[2]
        self.lastTurn = Mesh.RIGHT

    # def rotate(self):
    #     x = AStar.POOL_SIZE_X / 2
    #     y = AStar.POOL_SIZE_Y / 2
    #     r = FIND_CENTER_DISTANCE
    #     endPoint = [x, y, r]
    #
    #     _, _, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw,
    #                                 POOL_TARGET_DEEP, speedLongRatio)
    #     state = Mesh.STATE_FIND
    #     angularSpeed = Mesh.MAX_ANGULAR_SPEED_L
    #     vector = [[0, 0, 0], [0, 0, -angularSpeed], [0, 0, 0], [0, 0, 0], [0, 0, self.angle[2] - angularSpeed],
    #               [0, 0, 0]]
    #
    #     angle, turn = Mesh.calcAngleBeetwen(self.lastRolAngle, self.angle[2] + 1)
    #     print('FIRST')
    #     if turn == Mesh.RIGHT:
    #         # self.lastRolAngle = self.angle[2]
    #         self.findState = FS_FIRST_CIRCLE_FULL
    #
    #     return state, vector, image, endPoint

    def find(self, targetName, circles, startPoint, needToDraw, speedLongRatio):


        # for staying
        if self.x == 0 and self.y == 0 and self.totalPath == 0 :
            if targetName in self.objects:
                r = ImageHandler.objectData[targetName].goodR * COMPLETE_DISTANT_COEF + SELF_RADIUS
                endPoint = [self.objects[targetName].x, self.objects[targetName].y, r]

                state, vector, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw,
                                                     POOL_TARGET_DEEP, speedLongRatio)

                return Mesh.STATE_ERROR, Mesh.defaultVector(self.angle, POOL_TARGET_DEEP), image, endPoint
            else :
                x = AStar.POOL_SIZE_X / 2
                y = AStar.POOL_SIZE_Y / 2
                r = FIND_CENTER_DISTANCE
                endPoint = [x, y, r]

                state, vector, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw,
                                                     POOL_TARGET_DEEP, speedLongRatio)

                return Mesh.STATE_ERROR, Mesh.defaultVector(self.angle, POOL_TARGET_DEEP), image, endPoint

        if self.findState == FS_START_FIND :
            if targetName not in self.objects :
                self.lastRolAngle = self.angle[2]
                self.findState = FS_FIRST_CIRCLE_HALF
            else : self.findState = FS_GO_TO_OBJECT

        if self.findState == FS_GO_TO_OBJECT :
            x = self.objects[targetName].x
            y = self.objects[targetName].y
            badObjectIndex = AStar.AStar.checkLine2(circles, self.x, self.y, x, y, [], None)

            if badObjectIndex != -1 :

                distanceTargetToBed = ((x - circles[badObjectIndex][0]) ** 2 + (y - circles[badObjectIndex][1]) ** 2) ** 0.5
                distanceToTarget = ((self.x - x)**2 + (self.y - y)**2)**0.5
                distanceToBad = ((self.x - circles[badObjectIndex][0])**2 + (self.y - circles[badObjectIndex][1])**2)**0.5

                if distanceToTarget < distanceToBad:
                    angle = AStar.AStar.getPolarAngle(self.x, self.y, circles[badObjectIndex][0], circles[badObjectIndex][1])
                    self.objects[targetName].x += distanceTargetToBed*2 * Mesh.cos(angle)
                    self.objects[targetName].y += distanceTargetToBed*2 * Mesh.sin(angle)

            r = ImageHandler.objectData[targetName].goodR * COMPLETE_DISTANT_COEF + SELF_RADIUS
            endPoint = [self.objects[targetName].x, self.objects[targetName].y, r]

            state, vector, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw,
                                                 POOL_TARGET_DEEP, speedLongRatio)

            if state != Mesh.STATE_OK :
                self.lastRolAngle = self.angle[2]
                self.findState = FS_FIRST_CIRCLE_HALF
            state = Mesh.STATE_FIND

            return state, vector, image, endPoint

        if self.findState == FS_FIRST_CIRCLE_HALF:
            x = AStar.POOL_SIZE_X / 2
            y = AStar.POOL_SIZE_Y / 2
            r = FIND_CENTER_DISTANCE
            endPoint = [x, y, r]

            _, _, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw,
                                                 POOL_TARGET_DEEP, speedLongRatio)
            state = Mesh.STATE_FIND
            angularSpeed = Mesh.MAX_ANGULAR_SPEED_L
            vector = [[0, 0, 0], [0, 0, -angularSpeed], [0, 0, 0], [0, 0, 0], [0, 0, self.angle[2]+angularSpeed], [0, 0, 0]]

            angle, turn = Mesh.calcAngleBeetwen(self.lastRolAngle, self.angle[2]+1)
            #print('FIRST')
            if turn == Mesh.RIGHT :
                #self.lastRolAngle = self.angle[2]
                self.findState = FS_FIRST_CIRCLE_FULL

            return state, vector, image, endPoint

        if self.findState == FS_FIRST_CIRCLE_FULL:
            x = AStar.POOL_SIZE_X / 2
            y = AStar.POOL_SIZE_Y / 2
            r = FIND_CENTER_DISTANCE
            endPoint = [x, y, r]

            _, _, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw,
                                                 POOL_TARGET_DEEP, speedLongRatio)
            state = Mesh.STATE_FIND
            angularSpeed = Mesh.MAX_ANGULAR_SPEED_L
            vector = [[0, 0, 0], [0, 0, -angularSpeed], [0, 0, 0], [0, 0, 0], [0, 0, self.angle[2]+angularSpeed], [0, 0, 0]]

            angle, turn = Mesh.calcAngleBeetwen(self.lastRolAngle, self.angle[2]+1)
            #print('SECOND')
            if turn == Mesh.LEFT : self.findState = FS_TO_CENTER
            else :
                return state, vector, image, endPoint

        if self.findState == FS_TO_CENTER:

            x = AStar.POOL_SIZE_X / 2
            y = AStar.POOL_SIZE_Y / 2
            r = FIND_CENTER_DISTANCE

            if Mesh.distance(self.x, self.y, x, y) > FIND_CENTER_DISTANCE :
                endPoint = [x, y, r]

                state, vector, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw,
                                                     POOL_TARGET_DEEP, speedLongRatio)

                return state, vector, image, endPoint

            else :
                self.lastRolAngle = self.angle[2]
                self.findState = FS_SECOND_CIRCLE_HALF

        if self.findState == FS_SECOND_CIRCLE_HALF:
            x = AStar.POOL_SIZE_X / 2
            y = AStar.POOL_SIZE_Y / 2
            r = FIND_CENTER_DISTANCE
            endPoint = [x, y, r]

            _, _, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw,
                                                 POOL_TARGET_DEEP, speedLongRatio)
            state = Mesh.STATE_FIND
            angularSpeed = Mesh.MAX_ANGULAR_SPEED_L
            vector = [[0, 0, 0], [0, 0, -angularSpeed], [0, 0, 0], [0, 0, 0], [0, 0, self.angle[2]+angularSpeed], [0, 0, 0]]

            angle, turn = Mesh.calcAngleBeetwen(self.lastRolAngle, self.angle[2]+1)
            if turn == Mesh.RIGHT :
                #self.lastRolAngle = self.angle[2]
                self.findState = FS_SECOND_CIRCLE_FULL

            return state, vector, image, endPoint

        if self.findState == FS_SECOND_CIRCLE_FULL:
            x = AStar.POOL_SIZE_X / 2
            y = AStar.POOL_SIZE_Y / 2
            r = FIND_CENTER_DISTANCE
            endPoint = [x, y, r]

            _, _, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw,
                                        POOL_TARGET_DEEP, speedLongRatio)
            state = Mesh.STATE_FIND
            angularSpeed = Mesh.MAX_ANGULAR_SPEED_L
            vector = [[0, 0, 0], [0, 0, -angularSpeed], [0, 0, 0], [0, 0, 0], [0, 0, self.angle[2] + angularSpeed],
                      [0, 0, 0]]

            angle, turn = Mesh.calcAngleBeetwen(self.lastRolAngle, self.angle[2]+1)
            if turn == Mesh.LEFT:
                self.findState = FS_START_FIND
                #print('NO FIND')
                #input()
                return Mesh.STATE_FIND_FAIL, Mesh.defaultVector(self.angle, POOL_TARGET_DEEP), image, endPoint
            else:
                return state, vector, image, endPoint

    def updateTarget(self, targetName):
        if targetName not in ImageHandler.objectData or targetName is None : return
        self.target = targetName

    def getCircles(self, targetName):
        circles = []
        for obj in self.objects:
            obj = self.objects[obj]
            if obj.isActive and obj.name != targetName:
                circles.append([obj.x, obj.y, Map.calcObjectRadius(obj)])

        return circles

    def calcPath(self, needToDraw=False, targetName=None, speedLongRatio=1) :

        if targetName is not None:
            self.target = targetName
        targetName = self.target

        circles = self.getCircles(targetName)

        startPoint = [self.x, self.y, self.z]

        if targetName not in self.objects or not self.objects[targetName].isActive:
            state, vector, image, endPoint = self.find(targetName, circles, startPoint, needToDraw, speedLongRatio)
            if state == Mesh.STATE_OK : state = Mesh.STATE_FIND
            #isFind = True
        else :
            r = ImageHandler.objectData[targetName].goodR * COMPLETE_DISTANT_COEF + SELF_RADIUS
            endPoint = [self.objects[targetName].x, self.objects[targetName].y, r]

            #isFind = False
            state, vector, image = Mesh.calcMove(circles, startPoint, self.angle, endPoint, needToDraw, POOL_TARGET_DEEP, speedLongRatio)
            #if targetName not in self.objects : isFind = True
            self.findState = FS_START_FIND

        # new danger (but necessary) code start
        angle = vector[4][2]
        #angle += vector[4][2]
        speed = vector[5]

        angle %= 360


        angleDistance = min(abs(angle - self.lastAngle), 360 - abs(angle - self.lastAngle))
        print('ANGLE VIVE:', angle, self.lastAngle, angleDistance)

        # draw = ImageDraw.Draw(image)
        # AStar.AStar.drawLine(draw, startPoint[0], startPoint[1],
        #                      startPoint[0] + Mesh.cos(angle) * 7,
        #                      startPoint[1] + Mesh.sin(angle) * 7, 'green', 3)
        # AStar.AStar.drawLine(draw, startPoint[0], startPoint[1],
        #                      startPoint[0] + Mesh.cos(self.lastAngle) * 7,
        #                      startPoint[1] + Mesh.sin(self.lastAngle) * 7, 'black', 3)

        if (angleDistance > MAX_VIBE_ANGLE and False and
                self.findState != FS_FIRST_CIRCLE_HALF and self.findState != FS_SECOND_CIRCLE_HALF and
                self.findState != FS_FIRST_CIRCLE_FULL and self.findState != FS_SECOND_CIRCLE_FULL
            and (state == Mesh.STATE_OK or state == Mesh.STATE_FIND) and
            self.lastTargetName == targetName) :

            print("__VIBE__" * 5)
            #print('angle:', angle)
            #print('self.lastAngle:', self.lastAngle)
            #input()
            x = self.x + VIBE_CHECK_DISTANCE * Mesh.cos(self.lastAngle)
            y = self.y + VIBE_CHECK_DISTANCE * Mesh.sin(self.lastAngle)
            #startPoint = [x, y, self.z]

            #AStar.AStar.drawCircle(draw, x, y, 0.3, 'pink', outline='black', width=1)

            angleAAA = [0, 0, self.lastAngle]
            state2, vector2, image2 = Mesh.calcMove(circles, startPoint, angleAAA, endPoint, needToDraw,
                                                 POOL_TARGET_DEEP, speedLongRatio, 360 * 1/360)
            angle2 = vector2[4][2]
            angle2 %= 360

            angleDistance2 = min(abs(angle2 - self.lastAngle), 360 - abs(angle2 - self.lastAngle))
            if (angleDistance2 < angleDistance and angleDistance2 < MAX_VIBE_ANGLE * 3.5) or True:
                state, vector, image = state2, vector2, image2
                draw = ImageDraw.Draw(image)
                AStar.AStar.drawCircle(draw, self.x, self.y, 0.3, 'pink', outline='black', width=1)
                angle = angle2

        self.lastAngle = angle
        self.lastTargetName = targetName

        # new danger (but necessary) code end

        return state, vector, image




def updateImage(map, data, image, targetName) :
    objectData = ImageHandler.calcDistanceAndAngle(data, image)

    if LOG_SAVE :
        file = open(LOG_SAVE_FILENAME, 'a')
        file.write('#imageData#' + str(data) + '\n')
        file.write('#objectData#' + str(objectData) + '\n')
        file.close()

    map.updateObject(objectData)

    state, vector = map.calcPath(targetName)

    if state != Mesh.STATE_ERROR and state != Mesh.STATE_COMPLETE : Map.sendVector(vector)

    return state

def updatePosition(map, newPosition, targetName) :
    map.updatePosition(newPosition[0], newPosition[1], newPosition[2])


if __name__ == '__main__':

    map2 = Map([0, 0, 0], [0, 0, 90])