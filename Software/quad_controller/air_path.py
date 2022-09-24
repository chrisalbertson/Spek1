""" Compute the path a foot should take while it is raised off the ground
"""
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

class AirPath:

    def __init__(self):

        # for testing, we can use a triangle ramp footpath.  Later we can use
        # more realistic curves
        #
        # These are parametric functions where time is the parameter.
        points_t = (0.0, 0.20, 0.40, 0.60, 0.80, 1.0)  # Function INPUT
        points_x = (0.0, 0.20, 0.37, 0.64, 0.90, 1.0)  # Output values for x
        points_z = (0.0, 0.30, 0.60, 0.90, 1.00, 0.0)  # Output values for z

        # create cubic spline functions based on the above data points
        fx_of_t = interp1d(points_t, points_x, kind='cubic')
        fz_of_t = interp1d(points_t, points_z, kind='cubic')

        # Load up a Numpy array with many points from the above cubic spline
        # this will be our look-up table, so we compute 100 or more values,
        # later we can use the nearest point lookup and not have much error.
        self.num_data_points = 100
        self.t_point_table = np.linspace(0, 1, num=self.num_data_points+1, endpoint=True)
        self.x_point_table = fx_of_t(self.t_point_table)
        self.z_point_table = fz_of_t(self.t_point_table)

    def get_xz(self, air_fraction: float,
                     step_length: float,
                     step_height: float) -> (float, float):
        """Compute the position of a foot in the plane of motion for a given phase

        Parameters:
            air_fraction:   A value in the range 0..1 that gives the fraction of the
                            time a foot raise has been completed.  (See discussion below)

            step_length:    The length in meters of one step

            step_height:    The maximum height the foot should rais above the ground

        Returns:    A tuple containing (x, z) in meters

        Discussion:  This function basically does a look-up and a scaling of the data
            As much of the computation is done in the __init__ function as possible
            to make this lookup faster.  In this software, a step is completed by one
            leg in a period of time called "step_period".   In this period the foot
            leaves the ground for a fraction of the period we call "step_air_fraction".
            It then comes back in contact with the ground and remains there for a period
            of time step_period * ( 1.0 - step_air_fraction).

            The input parameters that define the steps, length, period, heiht and so on
            can change in real time.  THis funtion must react to this but we do not do
            planning here.  If the caller has done something silly such as shortening
            the step while a foot is in the air the robot may stumble can fall.

        TBD:  (1) Future research may show that using the same shape air path for all
            gaits and speeds is non-optimal.  So later  this function might be modified
            to accept a parameter perhaps defining "acceleration" or "stability target"
            and that would be ued to modify the path, perhaps by using a different lookup
            table.

            (2) This function uses linear interpolation.  It may be better to use
            something like cubic spline interpolation or perhaps Beiser curves.
        """

        x = step_length * self.x_point_table[round(air_fraction * self.num_data_points)]
        z = step_height * self.z_point_table[round(air_fraction * self.num_data_points)]
        foot_position = (x, z)

        return foot_position

    def plot_paths(self):
        """ Plt the air path from the table."""

        plt.plot(self.t_point_table, self.x_point_table, '-',
                 self.t_point_table, self.z_point_table, '--')
        plt.legend(['x', 'z'], loc='best')
        plt.show()


if __name__ == "__main__":

    for t in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        print(AirPath().get_xz(t, 0.100, 0.010))

    # Make a simple plot to verify everything works as it should
    AirPath().plot_paths()
