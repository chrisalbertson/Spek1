import logging


class PiecewiseLinear:
    """class to allow definition and lookup of functions defined by a set of line segments
    """

    def __init__(self, points):
        """Constructor, Defines a function based of a list (or tuple) of points"""

        max_slope = 100.0
        self.pointlist = points

        # function must have at least one line segment, defined by two points
        if len(self.pointlist) < 2:
            logging.error('function must have at least one line segment, defined by two points')

        # Validate input.  X values must monotonically increase.
        (x1, y1) = self.pointlist[0]
        for (x2, y2) in self.pointlist[1:-1]:
            if x1 > x2:
                logging.error('x values out of sequence, must be monotonic')
            if x1 == x2:
                logging.error('infinite slope detected')
            m = (y2 - y1) / (x2 - x1)
            if abs(m) > max_slope:
                logging.error('Maximum allowed slop is ', max_slope)
            (x1, y1) = (x2, y2)

        # find max X value. end of our function's range.
        pt_last = self.pointlist[-1]
        self.max_x = pt_last[0]

        # find max Y value. end of funtion's domain
        (x, max_y) = self.pointlist[0]
        for (x, y) in self.pointlist[1:]:
            if y > max_y:
                max_y = y

        self.max_y = max_y

    def get_point_count(self):
        """Returns the number of points started in the function definition

        >>> f = PiecewiseLinear( ((0,0), (1,1)) )
        >>> f.get_point_count()
        2
        """

        return len(self.pointlist)

    def get_point(self, x):
        """Returns the Y-value of the function for a given X-value

        >>> g = PiecewiseLinear( ((0,0), (1,1)) )
        >>> g.get_point( 0.5)
        0.5
        >>> g.get_point( 0.0)
        0.0
        >>> g.get_point( 1.0)
        1.0
        """

        pl_iter = iter(self.pointlist)
        (x1, y1) = next(pl_iter)

        # loop from first to next-to-last point
        next_pt = self.pointlist[-1]  # make sure next_pt is defined.
        for i in range(len(self.pointlist) - 1):
            (x2, y2) = next(pl_iter)
            if x1 <= x <= x2:
                # found it
                break

            (x1, y1) = (x2, y2)

        # Do the linear interpolation
        # print('interplation: ', x1, y1, x2, y2)
        m = (y2 - y1) / (x2 - x1)
        x_incr = x - x1
        y = y1 + (m * x_incr)
        return y


if __name__ == "__main__":
    import doctest

    doctest.testmod()
