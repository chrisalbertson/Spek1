import math
import logging
import numpy as np
from scipy.interpolate import interp1d
log = logging.getLogger(__name__)

class Footpath:
    def __init__(self, leg_x: float = 0.0, leg_y: float = 0.0) -> None:
        self.step_height = 0.030
        self.air_fraction_preferred = 0.1
        self.air_fraction = self.air_fraction_preferred
        self.contact_fraction = 1.0 - self.air_fraction

        # This define distance from leg origin to the robot body's center
        # of rotation.
        self.body_center_x = -leg_x     # meters
        self.body_center_y = -leg_y     # meters

        # These define the robot's neutral body center motion of the ground
        self.set_velocity(0.0, 0.0, 0.0)
        
        ## Build a loopup table for z_height(time)
        #
        # Function where time is time is input, z-height is output
        """points_t = (0.0,  0.20, 0.35, 0.50, 0.65, 0.80, 1.0)  # Function INPUT
        points_z = (0.0,  0.50, 0.95, 1.00, 0.95, 0.50, 0.0)  # Output values for z"""
        points_t = (0.0,  0.45, 0.55, 1.0)  # Function INPUT
        points_z = (0.0,  0.90, 0.90,0.0)  # Output values for z

        # create cubic spline function based on the above data points
        fz_of_t = interp1d(points_t, points_z, kind='cubic')

        # Load up a Numpy array with many points from the above cubic spline
        # this will be our look-up table, so we compute 100 or more values,
        # later we can use the nearest point lookup and not have much error.
        self.num_z_points = 100
        t_point_table = np.linspace(0, 1, num=self.num_z_points+1, endpoint=True)
        self.z_point_table = fz_of_t(t_point_table)

    def get_length_period(self, velocity: float) -> (float, float):
        """ find combination of step length and period given a desired velocity

        This is a placeholder function that returns a workable result.
        """
        assert velocity > 0.0

        min_hz = 0.2
        max_hz = 1.0

        preferred_step_length = 0.050  # FIXME constant
        max_length = 0.100

        step_hz = velocity / preferred_step_length

        if step_hz > max_hz:
            step_hz = max_hz
        elif step_hz < min_hz:
            step_hz - min_hz

        step_length = velocity / step_hz
        if step_length > max_length:
            print('step length clipped to max', step_length, max_length)
            step_length = max_length

        step_period = 1.0 / step_hz

        return step_length, step_period

    def set_velocity(self, x_vel: float, y_vel: float, z_rot: float) -> float:

        # Parameter, move to config.py
        sog_max = 0.4  # meters per second

        self.body_Rz = z_rot    #FIXME should do limit check

        sog = math.sqrt((x_vel**2) + (y_vel**2))

        if sog < 0.001:         #FIXME make this threshold configuable
            self.body_Vx = 0.0
            self.body_Vy = 0.0
            self.step_length = 0.0
            self.step_period = 1.0
        else:

            if sog > sog_max:
                self.body_Vx = x_vel * (sog_max / sog)
                self.body_Vy = y_vel * (sog_max / sog)
                print('WARNING SOG over limit,', sog, 'reduced to ', sog_max)
                print('  Vx Vy = ', self.body_Vx, self.body_Vy)
                sog = sog_max
            else:
                self.body_Vx = x_vel
                self.body_Vy = y_vel


            # Try the preferred step rate.   Use a faster rate if needed up to the
            # maximum rate.
            self.step_length, self.step_period = self.get_length_period(sog)

            print('sog, period length', sog, self.step_period, self.step_length)

            # Look at the length of time the feet are in the air.  With no abilty (yet)
            # to do active balance we must limit "air time".
            max_air_time = 0.1
            air_time = self.air_fraction * self.step_period
            if air_time > max_air_time:
                air_time = max_air_time
                self.air_fraction = air_time / self.step_period
                print('air fraction reset ', self.air_fraction, air_time)
            else:
                self.air_fraction = self.air_fraction_preferred

        return self.step_period

    def xyz(self, step_phase: float) -> (float, float, float):

        assert 0.0 <= step_phase <= 1.0

        # How much of the current segment is completed?
        # A step has two segments, an "air segment" and a "ground contact segment".
        # Typically, the air segment uses much less time than the ground contact segment
        # but both move the foot and equal distance.  During the air segment the foot
        # is in the air, moving forward, while in the ground contact segment the foot is
        # moving at -1.0 times the robot's neutral body center.
        if step_phase <= self.air_fraction:
            # foot is in the air (moving forward)
            segment_completed = step_phase / self.air_fraction
            
            # foot location in (x,y) before considering rotation
            f_x = segment_completed * self.body_Vx * self.step_period
            f_y = segment_completed * self.body_Vy * self.step_period

            # Check for special case of exactly zero x,y velocity
            if self.step_length == 0.0:
                f_z = 0.0
            else:
                f_z = self.z(segment_completed)
 
        else:
            # foot is in ground contact (moving backward)

            segment_completed = (step_phase - self.air_fraction) / (1.0 - self.air_fraction)
            
            # foot location in (x,y) before considering rotation
            f_x = (1.0 - segment_completed) * self.body_Vx * self.step_period
            f_y = (1.0 - segment_completed) * self.body_Vy * self.step_period
            f_z = 0.0

        # We can ignore very slow rotations, below some threshold
        if not math.isclose(self.body_Vy, 0.0, abs_tol=0.001):
            f_pnt = np.array([f_x, f_y, 1.0])
            f_pnt = self.rotation_mat(step_phase) @ f_pnt
            f_x, f_y = f_pnt[:2]

        return f_x, f_y, f_z

    def z(self, air_completed: float) -> float:
    
        assert 0.0 <= air_completed <= 1.0
	
        z = self.step_height * \
    	    self.z_point_table[round(air_completed * self.num_z_points)]
        return z

    def rotation_mat(self, step_phase: float):

        assert 0.0 <= step_phase <= 1.0
	
        # Numer of radians to rotate
        d_theta = step_phase * self.step_period * self.body_Rz

        # rotation is trivial if the rotation velocity is zero
        if not math.isclose(d_theta, 0.0, abs_tol=0.00001):
            # apply the effects of body rotation
            # ref: https://math.stackexchange.com/questions/
            #      2093314/rotation-matrix-of-rotation-around-a-point-other-than-the-origin
            cos_dt = math.cos(d_theta)
            sin_dt = math.sin(d_theta)
            xp = self.body_center_x
            yp = self.body_center_y
            """rot_mat = np.array([[cos_dt, -sin_dt, -xp * cos_dt + yp * sin_dt + xp],
                                   [sin_dt,  cos_dt, -xp * sin_dt + yp * cos_dt + yp],
                                   [0.0,     0.0,     1.0, ]])"""
            tr      = np.array([[1.0, 0.0,  xp],
                                [0.0, 1.0,  yp],
                                [0.0, 0.0,  1.0]])
            tr_inv  = np.array([[1.0, 0.0, -xp],
                                [0.0, 1.0, -yp],
                                [0.0, 0.0,  1.0]])               
            rot     = np.array([[cos_dt, -sin_dt, 0.0],
                                [sin_dt,  cos_dt, 0.0],
                                [0.0,     0.0,    1.0]])
                                
            # Translate to body center, rotate, then transfer back
            rot_mat = tr @ rot @ tr_inv
        else:
            # use identify matrix
            rot_mat = np.eye(3)
        return rot_mat


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    logging.basicConfig(filename='footpath.log',
                        filemode='w',
                        level=logging.DEBUG, )
    ## Perform simple test

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    xs = []
    ys = []
    zs = []

    fp = Footpath(0.130, 0.040)
    rot_rate = math.pi / 8.0
    fp.set_velocity(0.5, 0.25, rot_rate)
    n_steps = 1000

    for i in range(n_steps):
        phase: float = i/n_steps
        foot = fp.xyz(phase)
        print(phase, foot)
        xs.append(foot[0])
        ys.append(foot[1])
        zs.append(foot[2])

    ax.scatter(xs, ys, zs, marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()
