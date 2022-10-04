import math
import numpy as np
from scipy.interpolate import interp1d
import logging
log = logging.getLogger(__name__)

class Footpath:
    def __init__(self, leg_x: float = 0.0, leg_y: float = 0.0):
        self.step_height = 0.040
        self.air_fraction = 0.1
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
        points_t = (0.0, 0.20, 0.40, 0.60, 0.80, 1.0)  # Function INPUT
        points_z = (0.0, 0.30, 0.60, 0.90, 1.00, 0.0)  # Output values for z

        # create cubic spline function based on the above data points
        fz_of_t = interp1d(points_t, points_z, kind='cubic')

        # Load up a Numpy array with many points from the above cubic spline
        # this will be our look-up table, so we compute 100 or more values,
        # later we can use the nearest point lookup and not have much error.
        self.num_z_points = 100
        t_point_table = np.linspace(0, 1, num=self.num_z_points+1, endpoint=True)
        self.z_point_table = fz_of_t(t_point_table)



    def set_velocity(self, x_vel: float, y_vel: float, z_rot: float):

        # Some parameters, it should be possible to set these, but use constants for now
        preferred_step_rate = 1.0    # Hz
        max_step_rate = 4.0          # Hz
        max_step_length = 0.100      # Meters
        
        # From above limits, find max speed over ground
        sog_max = max_step_length * max_step_rate        # meters per second

        # find Speed Over Ground or the length of the velocity vector
        sog = math.sqrt(x_vel**2 + y_vel**2)
        
        # If the requested sog is too high we will clip it to maximum
        if sog > sog_max: 
            self.body_Vx = x_vel * (sog_max / sog)
            self.body_Vy = y_vel * (sog_max / sog)
            print('WARNING SOG over limit,', sog, 'reduced to ', sog_max)
            print('  Vx Vy = ', self.body_Vx, self.body_Vy)
            sog = sog_max
        else:
            self.body_Vx = x_vel
            self.body_Vy = y_vel
            
        self.body_Rz = z_rot   #FIXME should do limit check

        # Try the preferred step rate.   Use a faster rate if needed up to the
        # maximum rate.
        step_rate = preferred_step_rate
        step_length = sog / step_rate
        if step_length > max_step_length:
            step_rate = min(sog / max_step_length, max_step_rate)
            
        self.step_length = min(step_length, max_step_length)

        self.step_period = 1.0 / step_rate
        
        
        print('step period = ', self.step_period)
        print('step length = ', self.step_length)


    def xyz(self, phase: float) -> (float, float, float):

        assert 0.0 <= phase <= 1.0

        # How much of the current segment is completed?
        # A step has two segments, an "air segment" and a "ground contact segment".
        # Typically, the air segment uses much less time than the ground contact segment
        # but both move the foot and equal distance.  During the air segment the foot
        # is in the air, moving forward, while in the ground contact segment the foot is
        # moving at -1.0 times the robot's neutral body center.
        if phase <= self.air_fraction:
            # foot is in the air (moving forward)
            segment_completed = phase / self.air_fraction 
            
            # foot location in (x,y) before considering rotation
            f_x = segment_completed * self.body_Vx * self.step_period
            f_y = segment_completed * self.body_Vy * self.step_period
            f_z = self.z(segment_completed)
 
        else:
            # foot is in ground contact (moving backward)

            segment_completed = (phase - self.air_fraction) / (1.0 - self.air_fraction)
            
            # foot location in (x,y) before considering rotation
            f_x = (1.0 - segment_completed) * self.body_Vx * self.step_period
            f_y = (1.0 - segment_completed) * self.body_Vy * self.step_period
            f_z = 0.0

        # We can ignore very slow rotations, below some threshold
        if not math.isclose(self.body_Vy, 0.0, abs_tol=0.001):
            f_pnt = np.array([f_x, f_y, 1.0])
            f_pnt = self.rotation_mat(phase) @ f_pnt
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
    n_steps = 100

    for i in range(n_steps):
        phase = i/n_steps
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
