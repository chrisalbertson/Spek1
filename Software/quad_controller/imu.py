import time
import logging
log = logging.getLogger(__name__)

# Need to check of the hardware is installed
try:
    import board
except:
    # Likely there is no IMU hardware
    have_imu_hardware = False
else:
    have_imu_hardware = True

if have_imu_hardware:
    import adafruit_mpu6050


class IMU:
    """encapsulate the IMU and allow transparent use of differnt kinds of IMU boards"""

    def __init__(self):
        if not have_imu_hardware:
            return
        else:
            self.i2c = board.I2C()  # uses board.SCL and board.SDA
            self.mpu = adafruit_mpu6050.MPU6050(self.i2c)

            # set some properties
            self.mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
            self.mpu.filter_bandwidth = adafruit_mpu6050.Bandwidth.BAND_10_HZ
            self.mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS

            self.gyro_x_bias = 0.0
            self.gyro_y_bias = 0.0
            self.gyro_z_bias = 0.0
        return

    def check_imu_present(self):
        return have_imu_hardware

    def compute_gyro_bias(self, num_samples=100):
        if not have_imu_hardware:
            return 0., 0., 0.
        else:
            x_total = 0.0
            y_total = 0.0
            z_total = 0.0

            for i in range(num_samples):
                gyro_x, gyro_y, gyro_z = self.mpu.gyro
                x_total += gyro_x
                y_total += gyro_y
                z_total += gyro_z
                time.sleep(0.02)  # 50 Hz

            self.gyro_x_bias = x_total / num_samples
            self.gyro_y_bias = y_total / num_samples
            self.gyro_z_bias = z_total / num_samples

        return self.gyro_x_bias, self.gyro_y_bias, self.gyro_z_bias

    def get_acceleration(self):
        if not have_imu_hardware:
            return 0.0, 0.0, 9.81
        else:
            return self.mpu.acceleration

    def get_gyro(self):
        if not have_imu_hardware:
            return 0.0, 0.0, 0.0
        else:
            gyro_x, gyro_y, gyro_z = self.mpu.gyro
            gyro_x = gyro_x - self.gyro_x_bias
            gyro_y = gyro_y - self.gyro_y_bias
            gyro_z = gyro_z - self.gyro_z_bias
            return gyro_x, gyro_y, gyro_z

    def get_temperature(self):
        if not have_imu_hardware:
            return 0.0
        else:
            return self.mpu.temperature


def test_imu():
    imu = IMU()
    while True:
        a = imu.get_acceleration()
        g = imu.get_gyro()
        t = imu.get_temperature()

        b = imu.compute_gyro_bias(100)

        print("IMU present: " + str(imu.check_imu_present()))
        print("IMU zero bias:: X:%.3f, Y: %.3f, Z: %.3f rad/s" % b)
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % a)
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % g)
        print("Temperature: %.2f C" % t)
        print("")
        time.sleep(1)
    return


if __name__ == "__main__":

    test_imu()
