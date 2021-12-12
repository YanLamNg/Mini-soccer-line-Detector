import cv2 as cv
import numpy as np
import random
import math
from line_segment import LineSegment, Corner, FieldLines
from comparingLines import ComparingLines
from bisect import insort

lines = None
drawn_img=None
LSD_img=None
start = True
frame = None
lines_s = []

ANGLE_ERROR = 20
MAX_JOIN_DIST = 3
MAX_LINE_WIDTH = 70


def merge_line_by_distance_and_angle(lines_s,min_dist_to_connect,angle_offset):
    distance_offset = 3
    i = 0
    while i<len(lines_s):
        j = i+1
        while j<len(lines_s):
            comparingLine = ComparingLines(lines_s[i],lines_s[j])
            pts_dist = comparingLine.end_point_dist()
            angleDifference = comparingLine.angleDifference
            if (angleDifference < angle_offset or angleDifference > 180 -angle_offset) and min(pts_dist) < min_dist_to_connect and max(pts_dist)+distance_offset>comparingLine.get_total_length():
                lines_s.append(comparingLine.get_line_from_two_farest_points())
                lines_s.pop(j)
                lines_s.pop(i)
                break
            else:
                j+=1
        else:
            i+=1
    return lines_s

def merge_line_by_distance_and_angle_and_parallel_line(lines_s,min_dist_to_connect,angle_offset):
    distance_offset = 2
    i = 0
    while i<len(lines_s):
        j = i+1
        while j<len(lines_s):
            comparingLine = ComparingLines(lines_s[i],lines_s[j])
            pts_dist = comparingLine.end_point_dist()
            angleDifference = comparingLine.angleDifference
            if (angleDifference < angle_offset or angleDifference > 180 -angle_offset) and min(pts_dist) < min_dist_to_connect and max(pts_dist)>comparingLine.get_total_length()*0.75:
                lines_s.append(comparingLine.get_line_from_two_farest_points())
                lines_s.pop(j)
                lines_s.pop(i)
                break
            else:
                j+=1
        else:
            i+=1
    return lines_s

def merge_parallel_line(lines_s,max_dist_to_connect,angle_offset):
    distance_offset = 10
    i = 0
    while i<len(lines_s):
        j = i+1
        while j<len(lines_s):
            comparingLine = ComparingLines(lines_s[i],lines_s[j])
            pts_dist = comparingLine.end_point_dist()
            angleDifference = comparingLine.angleDifference
            if (angleDifference < angle_offset or angleDifference > 180 -angle_offset) and min(pts_dist) < max_dist_to_connect and max(pts_dist)+distance_offset<comparingLine.get_total_length():
                lines_s.append(comparingLine.get_line_from_two_closest_end_points())
                lines_s.pop(j)
                lines_s.pop(i)
                break
            else:
                j+=1
        else:
            i+=1
    return lines_s

def draw_line(drawn_img, lines_s, color):

    for line in lines_s:
        cv_line = line.to_cv_line()
        cv.line(drawn_img, cv_line[0], cv_line[1], \
                color, 3)


def lsd(lines_s):
    lineSegmentDetect()

    if lines_s is not None:

        lines_s = merge_line_by_distance_and_angle(lines_s, MAX_JOIN_DIST, 10)
        lines_s = merge_line_by_distance_and_angle(lines_s, MAX_JOIN_DIST, 30)

        lines_s = __filter_by_length(lines_s, 15)

        lines_s = merge_line_by_distance_and_angle(lines_s, MAX_LINE_WIDTH, 5)
        lines_s = merge_line_by_distance_and_angle_and_parallel_line(lines_s, MAX_LINE_WIDTH, 7)

        lines_s = __filter_by_length(lines_s, 20)

        merge_parallel_line(lines_s, MAX_LINE_WIDTH, 15)

        lines_s = __filter_by_length(lines_s, 50)

        classify_lines(lines_s)

    return lines_s

def classify_corner_shape(line, point, max_distance_apart):
    # Determine whether corner is L shaped or T shaped
    distance_apart = min(LineSegment.distance(x1=line.x1, y1=line.y1, x2=point[0], y2=point[1]),
                         LineSegment.distance(x1=line.x2, y1=line.y2, x2=point[0], y2=point[1]))

    return Corner.CornerType.l_shape if distance_apart <= max_distance_apart else Corner.CornerType.t_shape


def classify_lines(lines_s):

    center_line = []
    center_circle_PK_circle = []

    curve_line=[]
    all_line = list(lines_s)


    max_dist_to_connect = 60
    i = 0
    while i < len(lines_s):
        j = 0

        while j < len(lines_s):
            line = all_line[i]
            reference_line = all_line[j]
            comparingLine1 = ComparingLines(line, reference_line)
            angleDifference1 = comparingLine1.angleDifference


            if angleDifference1 > 10 and angleDifference1 < 50 and \
                    min(comparingLine1.end_point_dist() )< max_dist_to_connect and comparingLine1.length_difference()<200:

                k = 0

                while k < len(lines_s):

                    comparingLine2 = ComparingLines(lines_s[k], lines_s[i])
                    angleDifference2 = comparingLine2.angleDifference
                    if angleDifference2<50 and angleDifference2 > 10 and\
                        min(comparingLine2.end_point_dist()) < max_dist_to_connect and comparingLine2.length_difference()<200:
                        if all_line[k] not in curve_line:
                            curve_line.append(all_line[k])
                        if all_line[j] not in curve_line:
                            curve_line.append(all_line[j])
                        if all_line[i] not in curve_line:
                            curve_line.append(all_line[i])

                    k += 1
            j += 1
        i += 1


    drawn_img = frame.copy()
    draw_line(drawn_img, all_line, (255, 255, 0))

    draw_line(drawn_img, curve_line, (255, 0, 0))


    draw_line(drawn_img, center_line, (0, 0, 255))

    cv.imshow('lsd', drawn_img)

    return center_circle_PK_circle





def callback(x):
    global lines_s






def contoursConvexHull(contours):
    pts = []
    for i in range(0, len(contours)):
        if cv.contourArea(contours[i])>500:
            for j in range(0, len(contours[i])):
                pts.append(contours[i][j])

    pts = np.array(pts)
    result = None
    if len(pts>2):
        result = cv.convexHull(pts)
    return result

def __filter_by_length(lines, min_length):
    filtered_lines = []

    if lines is not []:
        for index, line in enumerate(lines):
            if line.length > min_length:
                filtered_lines.append(line)

    return filtered_lines

def lineSegmentDetect():
    global lines, LSD_img, lines_s
    lines_s = []
    gray_lsd = cv.cvtColor(LSD_img, cv.COLOR_BGR2GRAY)
    LineSegmentDetector = cv.createLineSegmentDetector()
    lines = LineSegmentDetector.detect(gray_lsd)[0]

    if lines is not None:

        for line in lines:
            lines_s.append(LineSegment(line[0][0], line[0][1], line[0][2], line[0][3]))
        lines_s.sort(key=lambda x:x.angle)




if __name__ == '__main__':
    cap = cv.VideoCapture('testing.mp4')
    if (cap.isOpened() == False):
        print("Error opening video stream or file")
    _, frame = cap.read()
    # frame = cv.imread('IMG_20180911_122210.jpg')
    frame = cv.resize(frame, (480, 360))
    lines_img = frame

    cv.namedWindow('frame', cv.WINDOW_AUTOSIZE)
    cv.namedWindow("lines_img", cv.WINDOW_AUTOSIZE)
    cv.imshow('frame', frame)
    cv.imshow('lines_img', lines_img)
    drawn_img = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)

    ilowH = 62
    ihighH = 121
    ilowS = 115
    ihighS = 255
    ilowV = 51
    ihighV = 255

    # # create trackbars for color change
    cv.createTrackbar('lowH', 'frame', ilowH, 179, callback)
    cv.createTrackbar('highH', 'frame', ihighH, 179, callback)
    cv.createTrackbar('lowS', 'frame', ilowS, 255, callback)
    cv.createTrackbar('highS', 'frame', ihighS, 255, callback)
    cv.createTrackbar('lowV', 'frame', ilowV, 255, callback)
    cv.createTrackbar('highV', 'frame', ihighV, 255, callback)

    l_lowH = 0
    l_highH = 179
    l_lowS = 0
    l_highS = 255
    l_lowV = 160
    l_highV = 255

    # create trackbars for color change
    cv.createTrackbar('lowH', 'lines_img', l_lowH, 179, callback)
    cv.createTrackbar('highH', 'lines_img', l_highH, 179, callback)
    cv.createTrackbar('lowS', 'lines_img', l_lowS, 255, callback)
    cv.createTrackbar('highS', 'lines_img', l_highS, 255, callback)
    cv.createTrackbar('lowV', 'lines_img', l_lowV, 255, callback)
    cv.createTrackbar('highV', 'lines_img', l_highV, 255, callback)

    while(1):
        _, frame = cap.read()
        frame = cv.resize(frame, (480, 360))


        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        hul = cv.getTrackbarPos('lowH', 'frame')
        huh = cv.getTrackbarPos('highH', 'frame')
        sal = cv.getTrackbarPos('lowS', 'frame')
        sah = cv.getTrackbarPos('highS', 'frame')
        val = cv.getTrackbarPos('lowV', 'frame')
        vah = cv.getTrackbarPos('highV', 'frame')

        blur = cv.GaussianBlur(frame, sigmaX=0, ksize=(5, 5))
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, (hul, sal, val), (huh, sah, vah))


        element = cv.getStructuringElement(cv.MORPH_ELLIPSE, (10,10))
        mask = cv.morphologyEx(mask,cv.MORPH_OPEN, element)
        mask = cv.morphologyEx(mask,cv.MORPH_CLOSE, element)


        contours,_ = cv.findContours(mask, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        height, width = frame.shape[:2]


        contourImg = np.zeros([height,width, 3], np.uint8)
        maxContour = None
        approx = None
        maxArea = -1
        hull = contoursConvexHull(contours)
        for cnt in contours:
            if cv.contourArea(cnt)>maxArea:
                maxContour = cnt
                maxArea = cv.contourArea(cnt)
            epsilon = 0.1 * cv.arcLength(cnt, True)


        field = np.zeros((height, width, 3),np.uint8)
        if hull is not None:
            for i, h in enumerate(hull):
                cv.polylines(contourImg, [hull], True, (0,255,255), 2)
                contourImg = cv.fillPoly(contourImg, [hull],(255,255,255))
                field = cv.bitwise_and(frame,contourImg)

        hul = cv.getTrackbarPos('lowH', 'lines_img')
        huh = cv.getTrackbarPos('highH', 'lines_img')
        sal = cv.getTrackbarPos('lowS', 'lines_img')
        sah = cv.getTrackbarPos('highS', 'lines_img')
        val = cv.getTrackbarPos('lowV', 'lines_img')
        vah = cv.getTrackbarPos('highV', 'lines_img')

        field = cv.GaussianBlur(field, ksize=(9,9), sigmaX=0)
        field_hsv = cv.cvtColor(field, cv.COLOR_BGR2HSV)
        lines_mask = cv.inRange(field_hsv,(hul, sal, val), (huh, sah, vah))

        element = cv.getStructuringElement(cv.MORPH_ELLIPSE, ksize=(7, 7))
        lines_mask = cv.morphologyEx(lines_mask, cv.MORPH_OPEN, kernel=element)
        lines_mask = cv.morphologyEx(lines_mask, cv.MORPH_CLOSE, kernel=element)

        white_img = np.zeros((height, width, 3), np.uint8)
        white_img[:] = (255, 255, 255)
        lines_img = cv.bitwise_and(white_img, white_img, mask=lines_mask)
        LSD_img =lines_img

        lsd(lines_s)
        cv.imshow('frame', mask)
        # cv.imshow('field', field)
        # cv.imshow('contourImg', contourImg)

        # cv.imshow('mask', mask)

        cv.imshow('lines_img',lines_img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # cap.release()
    cv.destroyAllWindows()