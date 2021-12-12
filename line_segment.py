from enum import Enum
import math
import cv2 as cv


class LineSegment(object):

    def __init__(self, x1, y1, x2, y2):
        pts = sorted([(x1, y1), (x2, y2)], key=lambda pt: pt[0])
        self.x1 = int(pts[0][0])
        self.y1 = int(pts[0][1])
        self.x2 = int(pts[1][0])
        self.y2 = int(pts[1][1])
        self.length = self.get_length(
            x1=self.x1, y1=self.y1, x2=self.x2, y2=self.y2)
        self.angle = self.__angle()
        self.position = None

    def __lt__(self, other_line_segment):
        return self.length < other_line_segment.length

    def distance_to_point(self, pt_x=0, pt_y=0):
        t = -((self.x1 - pt_x) * (self.x2 - self.x1) + (self.y1 - pt_y) * (self.y2 - self.y1)) / \
            (math.pow(self.x2 - self.x1, 2) + math.pow(self.y2 - self.y1, 2))

        if 0 <= t <= 1:
            distance = math.fabs((self.x2 - self.x1) * (self.y1 - pt_y) - (self.y2 - self.y1) * (self.x1 - pt_x)) / \
                math.sqrt(math.pow(self.x2 - self.x1, 2) +
                          math.pow(self.y2 - self.y1, 2))
        else:
            distance = min(math.sqrt(math.pow(self.x1 - pt_x, 2) + math.pow(self.y1 - pt_y, 2)),
                           math.sqrt(math.pow(self.x2 - pt_x, 2) + math.pow(self.y2 - pt_y, 2)))

        return distance

    def to_cv_line(self):
        return (self.to_pixel_value(self.x1), self.to_pixel_value(self.y1)), \
               (self.to_pixel_value(self.x2), self.to_pixel_value(self.y2))

    def get_angle_degree(self):
        return self.angle/math.pi*180
    def __angle(self):
        angle = math.atan2((self.y2 - self.y1), (self.x2 - self.x1))

        if self.x2 != self.x1 and (self.y2 - self.y1) / (self.x2 - self.x1) < 0:
            angle = -math.fabs(angle)

        return angle


    def drawLine(self, frame):
        frame = cv.line(frame, (self.x1,self.y1), (self.x2,self.y2), (255,0,255), 2)
    @staticmethod
    def distance(x1, y1, x2=0, y2=0):
        return math.sqrt(math.pow(math.fabs(x2 - x1), 2) + math.pow(math.fabs(y2 - y1), 2))

    @staticmethod
    def get_length(x1, y1, x2, y2):
        return LineSegment.distance(x1=x1, y1=y1, x2=x2, y2=y2)

    @staticmethod
    def to_pixel_value(pt):
        return max(0, int(round(pt)))


class Corner(object):

    CornerType = Enum('CornerType', 'l_shape, t_shape')

    def __init__(self, x, y, line1, line2=None):
        self.x = x
        self.y = y

        if line2 is not None:
            min_line = min(line1, line2, key=lambda line: line.length)
            max_line = line1 if min_line == line2 else line2
        else:
            min_line = line1
            max_line = None

        self.line1 = min_line
        self.line2 = max_line


class FieldLines(object):

    def __init__(self, lines, l_corners=0, t_corners=0):
        self.lines = lines
        self.l_corners = l_corners
        self.t_corners = t_corners

    def add_l_corner(self, line):
        self.lines += [line]
        self.l_corners += 1

    def add_t_corner(self):
        self.t_corners += 1

    def has_line(self, line):
        return line in self.lines
