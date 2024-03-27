
import math

class ObjectDataSegment :
    def __init__(self, width, height, maxRatio, badR, goodR, p, color):
        self.color = color
        self.goodR = goodR
        self.badR = badR
        self.maxRatio = maxRatio
        self.height = height
        self.width = width
        self.p = p

class DistanceCalculator:
    # Y, X
    def __init__(self,
                 imgsz=(480, 640),
                 fov=60,
                 object_attrs=None,
                 ):
        self.imgsz = imgsz
        self.fov = (fov * (imgsz[0] / imgsz[1]), fov)
        self.object_attrs = object_attrs

    def _tg(self, x) :
        return math.tan(x/360*2*math.pi)

    def _calcDistance(self, angleSize, realSize) :
        return realSize / (2 * self._tg(angleSize / 2))

    def calcDistanceAndAngle(self, xyxy, cls) :

        if cls in self.object_attrs:

            if cls == 'gate_qualification': cls = 'gate'

            if cls.p < self.object_attrs[cls].p: return None, None

            realSizeX, realSizeY = self.object_attrs[cls].width, self.object_attrs[cls].height

            sizeX = (xyxy[2] - xyxy[0]) / self.imgsz[1]
            sizeY = (xyxy[3] - xyxy[1]) / self.imgsz[0]
            angleSizeX = sizeX * self.fov[1]
            angleSizeY = sizeY * self.fov[0]

            DX = self._calcDistance(angleSizeX, realSizeX)
            DY = self._calcDistance(angleSizeY, realSizeY)

            calcRatio = sizeX / sizeY
            realRatio = realSizeX / realSizeY
            if (max(calcRatio, realRatio) / min(calcRatio, realRatio)) > self.object_attrs[cls].maxRatio:
                return None, None

            D = DX
            if (xyxy[1] == 0 or xyxy[3] == self.imgsz[0]) and (xyxy[0] == 0 or xyxy[2] == self.imgsz[1]): return None, None
            if (xyxy[0] == 0 or xyxy[2] == self.imgsz[1]): return None, None
            if (xyxy[0] != 0 and xyxy[2] != self.imgsz[1]): D = DX
            if (xyxy[1] != 0 and xyxy[3] != self.imgsz[0]): D = DY
            if (xyxy[0] != 0 and xyxy[2] != self.imgsz[1]) and (xyxy[1] != 0 and xyxy[3] != self.imgsz[0]): D = (DX + DY) / 2

            positionX = xyxy[0] + (xyxy[2] - xyxy[0]) / 2
            angleX = (positionX / self.imgsz[1]) * self.fov[1]
            angle = angleX - self.fov[1] / 2

            return D, angle

        return None, None
