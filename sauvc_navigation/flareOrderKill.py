import AStar
import ImageHandler
import Mesh
import sauvc_navigation.map as map

killOrder = ['yellow_flare', 'red_flare']

killCount = 0

SPEED_LONG_RATION = 5
END_POINT_RADIUS = 1
FRAME_CHECK_COUNT = 7
MAX_FAIL_COUNT = 4

STATUS_COMPLETE = 0
STATUS_FINDING = 1
STATUS_GOING = 2
STATUS_ERROR = 3
STATUS_KILLING = 4
STATUS_ROTATING = 5
STATUS_CHECKING = 6
STATUS_RESTART = 7
STATUS_KILL = 8
STATUS_MAX_FAIL_COUNT = 9
STATUS_CANT_FIND = 10

STATE_START = 0
STATE_REACHING = 1
STATE_KILLING = 2
STATE_ROTATING = 3
STATE_CHECKING = 4
stateM = STATE_START

toPoint = [0, 0, 0]
lastRolAngle = 0
lastViewedFrame = 0
failCount = 0

def statusToStr(status) :
    if status == STATUS_COMPLETE : return 'STATUS_COMPLETE'
    if status == STATUS_FINDING: return 'STATUS_FINDING'
    if status == STATUS_GOING : return 'STATUS_GOING'
    if status == STATUS_ERROR : return 'STATUS_ERROR'
    if status == STATUS_KILLING : return 'STATUS_KILLING'
    if status == STATUS_ROTATING : return 'STATUS_ROTATING'
    if status == STATUS_CHECKING : return 'STATUS_CHECKING'
    if status == STATUS_RESTART : return 'STATUS_RESTART'
    if status == STATUS_KILL : return 'STATUS_KILL'
    if status == STATUS_MAX_FAIL_COUNT: return 'STATUS_MAX_FAIL_COUNT'

    return 'STATUS_UNKNOWN'

def killHandler(state, vector, imageMap) :
    return state, vector, imageMap

def killFlares(map) :
    s, v = toKillNext(map, False, killHandler)

    return s, v[1], v[2]

def toKillNext(map, needToDraw, handler) :
    global stateM, toPoint, lastRolAngle, lastViewedFrame, killCount, killOrder, failCount

    if len(killOrder) == 0 : return STATUS_COMPLETE, handler(None, None, None)

    if stateM == STATE_START :
        toPoint = [0, 0, 0]
        lastRolAngle = 0
        lastViewedFrame = 0
        failCount = 0
        stateM = STATE_REACHING

    if stateM == STATE_REACHING :

        if failCount > MAX_FAIL_COUNT :
            stateM = STATE_START
            return STATUS_MAX_FAIL_COUNT, handler(None, None, None)

        state, vector, imageMap = map.calcPath(needToDraw, killOrder[0], SPEED_LONG_RATION)

        if state == Mesh.STATE_ERROR or state == Mesh.STATE_NO_PATH :
            stateM = STATE_START
            return STATUS_ERROR, handler(None, None, None)

        if state == Mesh.STATE_FIND_FAIL :
            stateM = STATE_START
            return STATUS_CANT_FIND, handler(None, None, None)

        status = STATUS_GOING
        if state == Mesh.STATE_FIND : status = STATUS_FINDING

        if state == Mesh.STATE_COMPLETE :
            r = ImageHandler.objectData[killOrder[0]].goodR * map.COMPLETE_DISTANT_COEF + map.SELF_RADIUS
            x = map.objects[killOrder[0]].x + Mesh.cos(map.angle[2]) * 2*r
            y = map.objects[killOrder[0]].y + Mesh.sin(map.angle[2]) * 2*r
            toPoint = [x, y, END_POINT_RADIUS]
            lastRolAngle = map.angle[2]
            stateM = STATE_KILLING
        else :
            return status, handler(state, vector, imageMap)

    if stateM == STATE_KILLING :
        circles = map.getCircles(killOrder[0])
        startPoint = [map.x, map.y, map.z]

        state, vector, image = Mesh.calcMove(circles, startPoint, map.angle, toPoint, needToDraw, map.POOL_TARGET_DEEP,
                                             SPEED_LONG_RATION)

        if state == Mesh.STATE_OK : return STATUS_KILLING, handler(state, vector, image)

        if state == Mesh.STATE_COMPLETE:
            lastRolAngle = map.angle[2]
            stateM = STATE_ROTATING
        else :
            stateM = STATE_START
            return STATUS_ERROR, handler(None, None, None)

    if stateM == STATE_ROTATING:

        x = AStar.POOL_SIZE_X / 2
        y = AStar.POOL_SIZE_Y / 2
        r = map.FIND_CENTER_DISTANCE
        endPoint = [x, y, r]
        circles = map.getCircles(killOrder[0])
        startPoint = [map.x, map.y, map.z]

        _, _, image = Mesh.calcMove(circles, startPoint, map.angle, endPoint, needToDraw,
                                    map.POOL_TARGET_DEEP, SPEED_LONG_RATION)
        state = Mesh.STATE_FIND
        angularSpeed = Mesh.MAX_ANGULAR_SPEED_L
        vector = [[0, 0, 0], [0, 0, -angularSpeed], [0, 0, 0], [0, 0, 0], [0, 0, map.angle[2] - angularSpeed],
                  [0, 0, 0]]

        angle, turn = Mesh.calcAngleBeetwen(lastRolAngle, map.angle[2] + 1)

        if turn == Mesh.RIGHT:
            lastViewedFrame = map.frameCount
            stateM = STATE_CHECKING
        else : return STATUS_ROTATING, handler(state, vector, image)

    if stateM == STATE_CHECKING:

        if map.objects[killOrder[0]].frameCount > lastViewedFrame :
            stateM = STATE_REACHING
            return STATUS_RESTART, handler(None, None, None)

        if map.frameCount > FRAME_CHECK_COUNT + lastViewedFrame :
            stateM = STATE_START
            killCount += 1
            killOrder.pop(0)
            return STATUS_KILL, handler(None, None, None)

        return STATUS_CHECKING, handler(None, None, None)

    print('ERROR:', stateM)
