from line_segment import LineSegment
import math


class ComparingLines(object):

    def __init__(self, line_s1, line_s2):
        self.line_s1 = line_s1
        self.line_s2 = line_s2
        self.angleDifference = math.fabs(line_s1.get_angle_degree() - line_s2.get_angle_degree())

    def end_point_dist(self):
        line1 = self.line_s1.to_cv_line()
        line2 = self.line_s2.to_cv_line()
        dist1 = LineSegment.distance(line1[0][0], line1[0][1], line2[0][0], line2[0][1])
        dist2 = LineSegment.distance(line1[0][0], line1[0][1], line2[1][0], line2[1][1])
        dist3 = LineSegment.distance(line1[1][0], line1[1][1], line2[0][0], line2[0][1])
        dist4 = LineSegment.distance(line1[1][0], line1[1][1], line2[1][0], line2[1][1])
        list = [dist1, dist2, dist3, dist4]
        list.sort()
        return list

    def closest_point_to_line_dist(self):
        line1 = self.line_s1.to_cv_line()
        line2 = self.line_s2.to_cv_line()
        dist1 = self.line_s1.distance_to_point(line2[0][0], line2[0][1])
        dist2 = self.line_s1.distance_to_point(line2[1][0], line2[1][1])
        dist3 = self.line_s2.distance_to_point(line1[0][0], line1[0][1])
        dist4 = self.line_s2.distance_to_point(line1[1][0], line1[1][1])
        list = [dist1, dist2, dist3, dist4]
        list.sort()
        return list

    def get_line_from_two_farest_points(self):
        line1 = self.line_s1.to_cv_line()
        line2 = self.line_s2.to_cv_line()
        dist1 = LineSegment.distance(line1[0][0], line1[0][1], line2[0][0], line2[0][1])
        dist2 = LineSegment.distance(line1[0][0], line1[0][1], line2[1][0], line2[1][1])
        dist3 = LineSegment.distance(line1[1][0], line1[1][1], line2[0][0], line2[0][1])
        dist4 = LineSegment.distance(line1[1][0], line1[1][1], line2[1][0], line2[1][1])

        if dist1 == max(dist1, dist2, dist3, dist4):
            return LineSegment(line1[0][0], line1[0][1], line2[0][0], line2[0][1])
        elif dist2 == max(dist2, dist3, dist4):
            return LineSegment(line1[0][0], line1[0][1], line2[1][0], line2[1][1])
        elif dist3 == max(dist3, dist4):
            return LineSegment(line1[1][0], line1[1][1], line2[0][0], line2[0][1])
        else:
            return LineSegment(line1[1][0], line1[1][1], line2[1][0], line2[1][1])

    def get_line_from_two_closest_end_points(self):
        to_return = None
        line1 = self.line_s1.to_cv_line()
        line2 = self.line_s2.to_cv_line()

        dist1 = LineSegment.distance(line1[0][0], line1[0][1], line2[0][0], line2[0][1])
        dist2 = LineSegment.distance(line1[0][0], line1[0][1], line2[1][0], line2[1][1])
        dist3 = LineSegment.distance(line1[1][0], line1[1][1], line2[0][0], line2[0][1])
        dist4 = LineSegment.distance(line1[1][0], line1[1][1], line2[1][0], line2[1][1])

        if dist1 == max(dist1, dist2, dist3, dist4):
            pt1_x = (line1[0][0] + line2[1][0]) / 2
            pt1_y = (line1[0][1] + line2[1][1]) / 2
            pt2_x = (line1[1][0] + line2[0][0]) / 2
            pt2_y = (line1[1][1] + line2[0][1]) / 2
            to_return = LineSegment(pt1_x, pt1_y, pt2_x, pt2_y)
        elif dist2 == max(dist2, dist3, dist4):
            pt1_x = (line1[0][0] + line2[0][0]) / 2
            pt1_y = (line1[0][1] + line2[0][1]) / 2
            pt2_x = (line1[1][0] + line2[1][0]) / 2
            pt2_y = (line1[1][1] + line2[1][1]) / 2
            to_return = LineSegment(pt1_x, pt1_y, pt2_x, pt2_y)
        elif dist3 == max(dist3, dist4):
            pt1_x = (line1[0][0] + line2[0][0]) / 2
            pt1_y = (line1[0][1] + line2[0][1]) / 2
            pt2_x = (line1[1][0] + line2[1][0]) / 2
            pt2_y = (line1[1][1] + line2[1][1]) / 2
            to_return = LineSegment(pt1_x, pt1_y, pt2_x, pt2_y)
        else:
            pt1_x = (line1[0][0] + line2[1][0]) / 2
            pt1_y = (line1[0][1] + line2[1][1]) / 2
            pt2_x = (line1[1][0] + line2[0][0]) / 2
            pt2_y = (line1[1][1] + line2[0][1]) / 2
            to_return = LineSegment(pt1_x, pt1_y, pt2_x, pt2_y)

        return to_return

    def get_total_length(self):
        return self.line_s1.length+self.line_s2.length

    def length_difference(self):
        return math.fabs(self.line_s1.length-self.line_s2.length)


    #Using formula from: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    def findIntersection(self):
        line1 = self.line_s1.to_cv_line()
        line2 = self.line_s2.to_cv_line()
        x1,y1,x2,y2 = line1[0][0], line1[0][1], line1[1][0], line2[1][1]
        x3,y3,x4,y4 = line1[0][0], line1[0][1], line2[0][0], line2[0][1]
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / (
                    (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / (
                    (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
        return [px, py]

