import numpy as np
import smbus2
import time
import serial
import adafruit_gps
import ctypes
import os
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3, TwistStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
import tf_transformations as tft

##  g++ -shared -fPIC -o build/libekfNavINS.so src/ekfNavINS_wrapper.cpp src/ekfNavINS.cpp -Iinc -I/usr/include/eigen3 -std=c++17
# Load the shared library
lib_path = os.path.abspath("./ekf_nav_ins/build/libekfNavINS.so")
ekf_lib = ctypes.CDLL(lib_path)

# --- Define Function Signatures ---

# Instance management
ekf_lib.ekf_create.restype = ctypes.c_void_p
ekf_lib.ekf_destroy.argtypes = [ctypes.c_void_p]

# EKF Update
ekf_lib.ekf_update.argtypes = [
    ctypes.c_void_p, ctypes.c_uint64,
    ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float
]

# Initialization status
ekf_lib.ekf_initialized.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_initialized.restype = ctypes.c_bool

# Getters for angles
ekf_lib.ekf_get_pitch.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_pitch.restype = ctypes.c_float

ekf_lib.ekf_get_roll.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_roll.restype = ctypes.c_float

ekf_lib.ekf_get_heading.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_heading.restype = ctypes.c_float

ekf_lib.ekf_get_heading_constrained.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_heading_constrained.restype = ctypes.c_float

# GPS Information
ekf_lib.ekf_get_latitude.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_latitude.restype = ctypes.c_double

ekf_lib.ekf_get_longitude.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_longitude.restype = ctypes.c_double

ekf_lib.ekf_get_altitude.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_altitude.restype = ctypes.c_double

# Velocities
ekf_lib.ekf_get_vel_north.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_vel_north.restype = ctypes.c_double

ekf_lib.ekf_get_vel_east.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_vel_east.restype = ctypes.c_double

ekf_lib.ekf_get_vel_down.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_vel_down.restype = ctypes.c_double

ekf_lib.ekf_get_ground_track.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_ground_track.restype = ctypes.c_float

# Gyro Bias
ekf_lib.ekf_get_gyro_bias_x.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_gyro_bias_x.restype = ctypes.c_float

ekf_lib.ekf_get_gyro_bias_y.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_gyro_bias_y.restype = ctypes.c_float

ekf_lib.ekf_get_gyro_bias_z.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_gyro_bias_z.restype = ctypes.c_float

# Accelerometer Bias
ekf_lib.ekf_get_accel_bias_x.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_accel_bias_x.restype = ctypes.c_float

ekf_lib.ekf_get_accel_bias_y.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_accel_bias_y.restype = ctypes.c_float

ekf_lib.ekf_get_accel_bias_z.argtypes = [ctypes.c_void_p]
ekf_lib.ekf_get_accel_bias_z.restype = ctypes.c_float

# Pitch, Roll, Yaw
ekf_lib.ekf_get_pitch_roll_yaw.argtypes = [
    ctypes.c_void_p,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float)
]

# IMU, GPS Coordinate, and GPS Velocity Updates
ekf_lib.ekf_imu_update.argtypes = [
    ctypes.c_void_p, ctypes.c_uint64,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float
]

ekf_lib.ekf_gps_coordinate_update.argtypes = [
    ctypes.c_void_p,
    ctypes.c_double, ctypes.c_double, ctypes.c_double
]

ekf_lib.ekf_gps_velocity_update.argtypes = [
    ctypes.c_void_p,
    ctypes.c_double, ctypes.c_double, ctypes.c_double
]

# --- Class Wrapper in Python ---
i2c_bus=1
class EKFNavINS:
    def __init__(self):
        self.obj = ekf_lib.ekf_create()

    def __del__(self):
        ekf_lib.ekf_destroy(self.obj)
    
    def update(self, time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz):
        ekf_lib.ekf_update(self.obj, time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz)

    def is_initialized(self):
        return ekf_lib.ekf_initialized(self.obj)

    # Angles
    def get_pitch(self):
        return ekf_lib.ekf_get_pitch(self.obj)

    def get_roll(self):
        return ekf_lib.ekf_get_roll(self.obj)

    def get_heading(self):
        return ekf_lib.ekf_get_heading(self.obj)

    def get_heading_constrained(self):
        return ekf_lib.ekf_get_heading_constrained(self.obj)

    # GPS
    def get_latitude(self):
        return ekf_lib.ekf_get_latitude(self.obj)

    def get_longitude(self):
        return ekf_lib.ekf_get_longitude(self.obj)

    def get_altitude(self):
        return ekf_lib.ekf_get_altitude(self.obj)

    # Velocities
    def get_vel_north(self):
        return ekf_lib.ekf_get_vel_north(self.obj)

    def get_vel_east(self):
        return ekf_lib.ekf_get_vel_east(self.obj)

    def get_vel_down(self):
        return ekf_lib.ekf_get_vel_down(self.obj)

    def get_ground_track(self):
        return ekf_lib.ekf_get_ground_track(self.obj)

    # Gyro Bias
    def get_gyro_bias_x(self):
        return ekf_lib.ekf_get_gyro_bias_x(self.obj)

    def get_gyro_bias_y(self):
        return ekf_lib.ekf_get_gyro_bias_y(self.obj)

    def get_gyro_bias_z(self):
        return ekf_lib.ekf_get_gyro_bias_z(self.obj)

    # Accelerometer Bias
    def get_accel_bias_x(self):
        return ekf_lib.ekf_get_accel_bias_x(self.obj)

    def get_accel_bias_y(self):
        return ekf_lib.ekf_get_accel_bias_y(self.obj)

    def get_accel_bias_z(self):
        return ekf_lib.ekf_get_accel_bias_z(self.obj)

    # Pitch, Roll, Yaw
    def get_pitch_roll_yaw(self, ax, ay, az, hx, hy, hz):
        pitch, roll, yaw = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
        ekf_lib.ekf_get_pitch_roll_yaw(self.obj, ax, ay, az, hx, hy, hz,
                                       ctypes.byref(pitch),
                                       ctypes.byref(roll),
                                       ctypes.byref(yaw))
        return pitch.value, roll.value, yaw.value

    # IMU, GPS Coordinate, GPS Velocity Updates
    def imu_update(self, time, gx, gy, gz, ax, ay, az, hx, hy, hz):
        ekf_lib.ekf_imu_update(self.obj, time, gx, gy, gz, ax, ay, az, hx, hy, hz)

    def gps_coordinate_update(self, lat, lon, alt):
        ekf_lib.ekf_gps_coordinate_update(self.obj, lat, lon, alt)

    def gps_velocity_update(self, vN, vE, vD):
        ekf_lib.ekf_gps_velocity_update(self.obj, vN, vE, vD)

# --- Class: HMC5883L ---
class HMC5883L:
    def __init__(self, i2c_bus=1, address=0x1E):
        self.bus = smbus2.SMBus(1)
        self.address = address

    def read_data(self):
        # Read 6 bytes from DATA_OUT register (0x03)
        data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
        x = self.convert_to_signed(data[0] << 8 | data[1])
        z = self.convert_to_signed(data[2] << 8 | data[3])
        y = self.convert_to_signed(data[4] << 8 | data[5])
        return x, y, z

    @staticmethod
    def convert_to_signed(value):
        return value - 0x10000 if value >= 0x8000 else value


# --- Class: MPU6050 ---
class MPU6050:
    def __init__(self, i2c_bus=1, address=0x68):
        self.bus = smbus2.SMBus(1)
        self.address = address

    def read_data(self):
        # Read 14 bytes from ACCEL_XOUT_H (0x3B)
        data = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
        accel_x = self.convert_to_signed(data[0] << 8 | data[1])
        accel_y = self.convert_to_signed(data[2] << 8 | data[3])
        accel_z = self.convert_to_signed(data[4] << 8 | data[5])
        gyro_x = self.convert_to_signed(data[8] << 8 | data[9])
        gyro_y = self.convert_to_signed(data[10] << 8 | data[11])
        gyro_z = self.convert_to_signed(data[12] << 8 | data[13])
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    @staticmethod
    def convert_to_signed(value):
        return value - 0x10000 if value >= 0x8000 else value


class EKFPublisherNode(Node):
    def __init__(self):
        super().__init__('ekf_publisher')

        # Publishers
        self.fix_publisher = self.create_publisher(NavSatFix, '/filtered/fix', 10)
        self.pry_publisher = self.create_publisher(Vector3, '/filtered/pry', 10)
        self.vel_publisher = self.create_publisher(TwistStamped, '/filtered/vel', 10)
        self.quaternion_publisher = self.create_publisher(Quaternion, '/filtered/quaternion', 10)
        
        # Initialize EKF, GPS, IMU, and Magnetometer
        self.ekf = EKFNavINS()
        self.gps = self.initialize_gps()
        self.mpu = MPU6050()
        self.mag = HMC5883L()

        # Timer to update data at 1Hz
        self.timer = self.create_timer(0.001, self.publish_data)

    def initialize_gps(self):
        gps_uart = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=2)
        gps = adafruit_gps.GPS(gps_uart)
        return gps

    def publish_data(self):
        # Read data from sensors
        ax, ay, az, gx, gy, gz = self.mpu.read_data()
        hx, hy, hz = self.mag.read_data()

        if self.gps.update() and self.gps.has_fix:
            lat = self.gps.latitude
            lon = self.gps.longitude
            alt = 0.0
            speed_knots = self.gps.speed_knots if self.gps.speed_knots else 0.0    # Velocity fields (set to 0 since no velocity info is available in this example)ts if self.gps.speed_knots else 0.0
            course_deg = self.gps.track_angle_deg if self.gps.track_angle_deg else 0.0

            speed_mps = speed_knots * 0.514444
            vn = speed_mps * math.cos(math.radians(course_deg))
            ve = speed_mps * math.sin(math.radians(course_deg))
            vd = 0.0  # Downward velocity is set to zero

            if self.gps.timestamp_utc:
                gps_time_of_week = (
                    self.gps.timestamp_utc.tm_hour * 3600
                    + self.gps.timestamp_utc.tm_min * 60
                    + self.gps.timestamp_utc.tm_sec
                )
            else:
                self.get_logger().warn("GPS time not available, falling back to system time.")
                gps_time_of_week = time.time()
            self.ekf.update(np.uint64(gps_time_of_week), vn, ve, vd, lat, lon, alt, gx, gy, gz, ax, ay, az, hx, hy, hz)

            # Get EKF outputs
            pitch = self.ekf.get_pitch()
            roll = self.ekf.get_roll()
            heading = self.ekf.get_heading()
            vel_north = self.ekf.get_vel_north()
            vel_east = self.ekf.get_vel_east()
            vel_down = self.ekf.get_vel_down()

            # Publish /filtered/fix
            fix_msg = NavSatFix()            # Convert PRY (pitch, roll, heading) to a quaternion
            self.quaternion_publisher = self.create_publisher(Quaternion, '/filtered/quaternion', 10)
            quaternion = tft.quaternion_from_euler(roll, pitch, heading)

            # Create a quaternion message
            quaternion_msg = Quaternion()
            quaternion_msg.x = quaternion[0]
            quaternion_msg.y = quaternion[1]
            quaternion_msg.z = quaternion[2]
            quaternion_msg.w = quaternion[3]

            # Publish the quaternion
            self.quaternion_publisher.publish(quaternion_msg)
            fix_msg.header.stamp = self.get_clock().now().to_msg()
            fix_msg.latitude = self.ekf.get_latitude()
            fix_msg.longitude = self.ekf.get_longitude()
            fix_msg.altitude = 0.0
            self.fix_publisher.publish(fix_msg)

            # Publish /filtered/vel
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.twist.linear.x = vel_north
            vel_msg.twist.linear.y = vel_east
            vel_msg.twist.linear.z = 0.0
            self.vel_publisher.publish(vel_msg)

            self.get_logger().info(f"Published GPS Fix, PRY, and Velocity data.")
        #else:
            #self.get_logger().warn("Waiting for GPS fix...")

def main(args=None):
    rclpy.init(args=args)
    node = EKFPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

