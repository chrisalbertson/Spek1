import math
import unittest
import sm_kinematics

sk = sm_kinematics.Kinematic()


def ik_write_csv(x: float, y: float, z: float, ) -> (float, float, float):

    (th1, th2, th3) = sk.legIK((x, y, z))
    th1d = math.degrees(th1)
    th2d = math.degrees(th2)
    th3d = math.degrees(th3)
    print("%7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f" % (x, y, z, th1d, th2d, th3d))


class MyTestCase(unittest.TestCase):
    def test_something(self):
        self.assertEqual(True, False)  # add assertion here




if __name__ == '__main__':

    ik_write_csv(-60, -240, 40)
    # unittest.main()
