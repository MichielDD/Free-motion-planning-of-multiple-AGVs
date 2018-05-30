# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

import numpy as np


def distance_between_points(point1, point2):
        # calculate distance between two points in kD-space
        dist = 0
        if len(point1) != len(point2):
            raise ValueError('Dimensions of point1 and point2 do not match: ', len(point1), ', ', len(point2))
        for k in range(len(point1)):
            dist += (point2[k]-point1[k])**2
        return np.sqrt(dist)

def distance_to_rectangle(point, rectangle):
    # returns the x- and y-direction distance from point to a rectangle
    # based on: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    vertices = rectangle.vertices
    vertices[0] +=  rectangle.signals['position'][:,-1][0]
    vertices[1] +=  rectangle.signals['position'][:,-1][1]

    x1 = min(vertices[0])
    y1 = max(vertices[1])
    x2 = max(vertices[0])
    y2 = min(vertices[1])

    # number vertices of rectangle
    # v1--v2
    # |   |
    # v4--v3
    v1 = [x1,y1]
    v2 = [x1,y2]
    v3 = [x2,y2]
    v4 = [x2,y1]

    # dist from v1-v2
    # Note the minus sign! A negative shift in x-direction is required to lower the distance
    dist12 = -distance_to_line(point, [v1,v2])
    # dist from v2-v3
    dist23 = distance_to_line(point, [v2,v3])
    # dist from v3-v4
    dist34 = distance_to_line(point, [v3,v4])
    # dist from v4-v1
    # Note the minus sign! A negative shift in y-direction is required to lower the distance
    dist41 = -distance_to_line(point, [v4,v1])

    distance = [0.,0.]
    distance[0] = dist12 if abs(dist12) < abs(dist34) else dist34  # x-direction: from point to side12 or side34
    distance[1] = dist23 if abs(dist23) < abs(dist41) else dist41  # y-direction: from point to side23 or side41
    return distance

def distance_to_line(point, line):
    vertex1 = line[0]  # endpoint 1 of line
    vertex2 = line[1]  # endpoint 2 of line
    dist = abs((vertex2[1]-vertex1[1])*point[0] - (vertex2[0]-vertex1[0])*point[1] + vertex2[0]*vertex1[1] - vertex2[1]*vertex1[0])/(np.sqrt((vertex2[1]-vertex1[1])**2+(vertex2[0]-vertex1[0])**2))
    return dist

def order_is_ccw(point1, point2, point3):
    # based on: http://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    # Determine if three points are listed in a counterclockwise order.
    # There are three points: point1, point2, point3. If the slope of the line between point1&point2 is
    # smaller than the slope of the line between point1&point3,
    # then the three points have a counterclockwise order.

    return (point3[1]-point1[1])*(point2[0]-point1[0]) > (point2[1]-point1[1])*(point3[0]-point1[0])

def intersect_line_segments(line1, line2):
    point1, point2 = line1
    point3, point4 = line2
    # based on: http://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    # There are two line segments, one between point1&point2, one between point2&point3.
    # The segments only intersect if point1 and point2 are separated by the line segment between
    # point2 & point3 and point3 & point4 are separated by the segment between point1 & point2.
    # If point1 and point 2 are separated by the segment between point3 & point4, then the connection of
    # point1-point3-point4 and point2-point3-point4 have an opposite order, this means that
    # one of both connections is ordered counterclockwise, but noth both.

    return order_is_ccw(point1,point3,point4) != order_is_ccw(point2,point3,point4) and order_is_ccw(point1,point2,point3) != order_is_ccw(point1,point2,point4)

def intersect_lines(line1, line2):
    # based on:  https://en.wikipedia.org/wiki/Line-line_intersection
    x1, y1= line1[0]
    x2, y2= line1[1]
    x3, y3= line2[0]
    x4, y4= line2[1]

    if (((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)) == 0.):
        # parallel lines
        intersection_point = None
    else:
        intersection_point = [0.,0.]
        intersection_point[0] = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))
        intersection_point[1] = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4))
    return intersection_point

def point_in_polyhedron(point, polyhedron_shape, polyhedron_position, margin=0):
    # is the point inside the polyhedron?

    # for each vertex couple (=side) check if point is at positive side of normal
    # if so return False, because the point can never be inside the polyhedron then
    hyperplanes = polyhedron_shape.get_hyperplanes(position=polyhedron_position)
    for idx, hyperplane in hyperplanes.items():
        a = hyperplane['a']
        b = hyperplane['b']
        if ((a[0]*point[0] + a[1]*point[1] - b - margin) > 0):
            return False
    return True

def circle_polyhedron_intersection(circle, polyhedron_shape, polyhedron_position):

    # Does one of the sides of the polyhedron have a point inside the circle?
    # Compute perpendicular from the circle center to the polyhedron side
    # Check if the intersection between the normal and the side is inside the circle

    # The checks below can give problems when using floating point numbers:
    # x1<=x4<=x2 can fail if x1~=x4~=x2 but e.g. just a little bigger for the n'th decimal
    # to avoid this, we use some epsilon
    eps = 1e-6

    # get polyhedron vertices
    vertices = polyhedron_shape.vertices.copy()
    # duplicate first vertex and place at the end,
    # to check all sides
    vertices = np.c_[vertices, vertices[:,0]]
    # add current position to vertices
    vertices[0] +=  polyhedron_position[0]
    vertices[1] +=  polyhedron_position[1]

    center = circle.signals['position'][:,-1]
    for i in range(len(vertices)):
        # first quickly check if any vertex is inside the circle
        dist = distance_between_points(center, [vertices[0][i], vertices[1][i]])
        if dist <= circle.shape.radius:
            # one of the vertices is inside the circle
            return True
    for i in range(len(vertices[0])-1):
        # compute perpendicular of circle center to line and its intersection point (x4,y4) with the line
        # based on: http://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point
        x1, y1, x2, y2 = vertices[0][i], vertices[1][i], vertices[0][i+1], vertices[1][i+1]
        k = ((y2-y1) * (center[0]-x1) - (x2-x1) * (center[1]-y1)) / ((y2-y1)**2 + (x2-x1)**2)
        x4 = center[0] - k * (y2-y1)
        y4 = center[1] + k * (x2-x1)

        # see if intersection point is within the two end points of line
        # and check distance between intersection point and circle center
        line1 = [[x1,y1],[x2,y2]]  # side of polyhedron
        if (((x1-eps<=x4<=x2+eps and y1-eps<=y4<=y2+eps) or (x1+eps>=x4>=x2-eps and y1+eps>=y4>=y2-eps)) and
           (distance_between_points(center, [x4,y4]) <= circle.shape.radius)):
            return True
    return False

def point_in_rectangle(rectangle_limits, point, horizon_time=None, velocity=None, distance=0, xy_check=False):
        # check if the provided point is inside the provided rectangle
        # both for stationary or moving points
        xmin, ymin, xmax, ymax = rectangle_limits
        # check stationary point
        if horizon_time is None:
            if xy_check:
                # see if point is inside rectangle for separate directions (x and y)
                result = []
                if(xmin+distance <= point[0] <= xmax-distance):
                    result.append(True)
                else:
                    result.append(False)
                if(ymin+distance <= point[1] <= ymax-distance):
                    result.append(True)
                else:
                    result.append(False)
                return result
            else:
                if (xmin+distance <= point[0] <= xmax-distance) and (ymin+distance <= point[1] <= ymax-distance):
                    return True
                else:
                    return False
        # check moving point
        elif isinstance(horizon_time, (float, int)):
            # check at each time_interval seconds
            time_interval = 0.5
            # amount of times to check
            N = int(round(horizon_time/time_interval)+1)
            # sample time of check
            Ts = float(horizon_time)/N
            x, y = point
            vx, vy = velocity
            for l in range(N+1):
                if xmin <= (x+l*Ts*vx) <= xmax and ymin <= (y+l*Ts*vy) <= ymax:
                    return True
            return False
        else:
            raise RuntimeError('Argument horizon_time was of the wrong type, not None, float or int')

def rectangles_overlap(shape1, pos1, shape2, pos2):

    if shape1.orientation == 0 and shape2.orientation == 0:
        [[xmin1,xmax1],[ymin1,ymax1]] = shape1.get_canvas_limits()
        xmin1 += pos1[0]
        xmax1 += pos1[0]
        ymin1 += pos1[1]
        ymax1 += pos1[1]
        [[xmin2,xmax2],[ymin2,ymax2]] = shape2.get_canvas_limits()
        xmin2 += pos2[0]
        xmax2 += pos2[0]
        ymin2 += pos2[1]
        ymax2 += pos2[1]
        if (xmin2 <= xmax1 and xmax2 >= xmin1 and
            ymin2 <= ymax1 and ymax2 >= ymin1):
            return True
        else:
            return False
    else:
        raise RuntimeError('Provided a rectangle with non-zero orientation, this function only supports' +
                           ' zero orientation')

def compute_rectangle_overlap_center(shape1, pos1, shape2, pos2):
    # Compute the center of the overlap region of two overlapping rectangles
    # Note: this function only works for non-rotated rectangles

    [[xmin1,xmax1],[ymin1,ymax1]] = shape1.get_canvas_limits()
    [[xmin2,xmax2],[ymin2,ymax2]] = shape2.get_canvas_limits()

    xmin1 += pos1[0]
    xmax1 += pos1[0]
    ymin1 += pos1[1]
    ymax1 += pos1[1]

    xmin2 += pos2[0]
    xmax2 += pos2[0]
    ymin2 += pos2[1]
    ymax2 += pos2[1]

    # Check if the rectangles overlap
    if (xmin2 <= xmax1 and xmax2 >= xmin1 and
        ymin2 <= ymax1 and ymax2 >= ymin1):

        xmin3 = max(xmin1, xmin2)
        xmax3 = min(xmax1, xmax2)
        ymin3 = max(ymin1, ymin2)
        ymax3 = min(ymax1, ymax2)

        center = [0, 0]
        center[0] = xmin3 + (xmax3-xmin3)*0.5
        center[1] = ymin3 + (ymax3-ymin3)*0.5
        return center
    else:
        raise RuntimeError('Trying to compute center of overlap region, but rectangles don\'t overlap')

