import AStar
import ImageHandler
import Mesh
import sauvc_navigation.Map as map

gateNames = ['gate', 'gate_qualification']

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
STATE_ENTERING = 2
STATE_ROTATING = 3
STATE_CHECKING = 4
stateM = STATE_START

toPoint = [0, 0, 0]
lastRolAngle = 0
lastViewedFrame = 0
failCount = 0

def gateHandler(state, vector, imageMap) :
    return state, vector, imageMap

def enterGate(map) :
    s, v = enterGateNext(map, False, gateHandler)

    return s, v[1], v[2]

def enterGateNext(map, needToDraw, handler) :
    global stateM, toPoint, lastRolAngle, lastViewedFrame, failCount

    print('STATE:', stateM)

    if stateM == STATE_START :
        toPoint = [0, 0, 0]
        lastRolAngle = 0
        lastViewedFrame = 0
        failCount = 0
        stateM = STATE_REACHING

    if stateM == STATE_REACHING :

        state, vector, imageMap = map.calcPath(needToDraw, gateNames[0], SPEED_LONG_RATION)

        if state == Mesh.STATE_ERROR or state == Mesh.STATE_NO_PATH :
            stateM = STATE_START
            return STATUS_ERROR, handler(None, None, None)

        if state == Mesh.STATE_FIND_FAIL :
            stateM = STATE_START
            return STATUS_CANT_FIND, handler(None, None, None)

        status = STATUS_GOING
        if state == Mesh.STATE_FIND : status = STATUS_FINDING

        if state == Mesh.STATE_COMPLETE :
            r = 2 * ImageHandler.objectData[gateNames[0]].goodR * map.COMPLETE_DISTANT_COEF + map.SELF_RADIUS
            r += END_POINT_RADIUS
            x = map.objects[gateNames[0]].x + Mesh.cos(map.angle[2]) * 1*r
            y = map.objects[gateNames[0]].y + Mesh.sin(map.angle[2]) * 1*r
            toPoint = [x, y, END_POINT_RADIUS]
            lastRolAngle = map.angle[2]
            print('GatePos: ', map.objects['gate'].x, map.objects['gate'].y)
            stateM = STATE_ENTERING
        else :
            return status, handler(state, vector, imageMap)

    if stateM == STATE_ENTERING :
        circles = map.getCircles(gateNames[0])
        startPoint = [map.x, map.y, map.z]

        state, vector, image = Mesh.calcMove(circles, startPoint, map.angle, toPoint, needToDraw, map.POOL_TARGET_DEEP,
                                             SPEED_LONG_RATION)

        if state == Mesh.STATE_OK : return STATUS_KILLING, handler(state, vector, image)

        if state == Mesh.STATE_COMPLETE:
            stateM = STATE_START
            return STATUS_KILL, handler(None, None, None)
        else :
            stateM = STATE_START
            return STATUS_ERROR, handler(None, None, None)

    print('ERROR:', stateM)
