"""Funtions related to computing balance"""

import math
import logging
log = logging.getLogger(__name__)


Point = (float, float)


def area_triangle(a: Point, b: Point, c: Point) -> float:
    """Compute the area of a triangle given three points A, B and C

    >>> area_triangle((0., 0.), (0., 1.), (1., 0.))
    0.5

    >>> area_triangle((0., 0.), (0., 2.), (2., 0.))
    2.0
    """

    x1, y1 = a
    x2, y2 = b
    x3, y3 = c

    area = abs(x1 * (y2 - y3) +
               x2 * (y3 - y1) +
               x3 * (y1 - y2)) / 2.0
    return area


def centroid_triangle(a: Point, b: Point, c: Point) -> Point:
    """Compute the centroid of a triangle given three points A, B and C

    >>> centroid_triangle((0., 0.), (0., 1.), (1., 0.))
    (0.3333333333333333, 0.3333333333333333)
    """

    x1, y1 = a
    x2, y2 = b
    x3, y3 = c

    x_average = (x1 + x2 + x3) / 3.0
    y_average = (y1 + y2 + y3) / 3.0
    return x_average, y_average


def centroid_quadrilateral(a: Point, b: Point, c: Point, d: Point) -> Point:
    """quadrilateral

    >>> centroid_quadrilateral((0., 0.), (0., 1.), (1., 1.), (1., 0.))
    (0.5, 0.5)

    >>> centroid_quadrilateral((-2., 0.), (0., 2.), (2., 0.), (0., -2.))
    (0.0, 0.0)
    """

    """The method used here is as follows:
        1) Divide the quadrilateral into two triangles, 
        2) Find the centroid and area of each triangle
        3) compute the weighted average of each triangle, using area as the weight
    """
    t1_area = area_triangle(a, b, c)
    t1_cent_x, t1_cent_y = centroid_triangle(a, b, c)
    t2_area = area_triangle(c, d, a)
    t2_cent_x, t2_cent_y = centroid_triangle(c, d, a)

    q_area = t1_area + t2_area
    weighted_x = (t1_area * t1_cent_x) + (t2_area * t2_cent_x)
    weighted_y = (t1_area * t1_cent_y) + (t2_area * t2_cent_y)

    q_cent_x = weighted_x / q_area
    q_cent_y = weighted_y / q_area

    return q_cent_x, q_cent_y


def rotate_to_level(accel_x: float, accel_y: float, accel_z: float) -> (float, float):
    """convert an accelerometer output to angels (in radians) needed to level the body

    returns (roll, pitch) both floats in radians



    >>> rotate_to_level(0.0, 0.0, 10.0)
    (0.0, 0.0)

    >>> rotate_to_level(1.0, 0.0, 10.0)
    (0.0, -0.09966865249116202)

    >>> rotate_to_level(0.0, 1.0, 10.0)
    (-0.09966865249116202, 0.0)

    >>> rotate_to_level(-1.0, -1.0, 10.0)
    (0.09966865249116202, 0.09966865249116202)
    """

    # FIXME for now, we assume the accelerometer axis is aligned with the body axis

    # find the angles, assume z is vertical, y points left, x forward
    # Multiply be -1 to get corrective rotation to restore to zero.
    roll  = -1.0 * math.atan2(accel_y, accel_z)
    pitch = -1.0 * math.atan2(accel_x, accel_z)

    # don't bother correcting angles less than the servo motor resolution
    # and also get rid of those annoying -0.0 results.
    if abs(roll) < 0.001:
        roll = 0.0
    if abs(pitch) < 0.001:
        pitch = 0.0
    return roll, pitch


if __name__ == "__main__":
    import doctest

    print("Running doctests...")
    doctest.testmod(name='balance', verbose=True)
