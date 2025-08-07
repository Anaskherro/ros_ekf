import ctypes
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, MagneticField, TimeReference
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from datetime import datetime
from std_msgs.msg import Float32


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

class EKFNavINS:
    def __init__(self):
        self.obj = ekf_lib.ekf_create()

    def __del__(self):
        ekf_lib.ekf_destroy(self.obj)
    #                      /vel_topic- /fix_topic--  /imu/data_raw-------  /imu/mag--     
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
# --- Example Usage ---

class DataGetter(Node):
    def __init__(self, ekf):
        super().__init__('data_getter')

        # Variables to store raw data
        self.time_sec = None
        self.time_ref_sec = None
        self.time_ref_nanosec = None

        self.lon = None
        self.lat = None
        self.alt = None
        self.vn = None
        self.ve = None
        self.vd = None
        self.mx = None
        self.my = None
        self.mz = None
        self.p = None
        self.q = None
        self.r = None
        self.ax = None
        self.ay = None
        self.az = None

        # EKF instance
        self.ekf = ekf

        # Publishers for filtered data
        self.filtered_fix_publisher = self.create_publisher(NavSatFix, '/filtered/fix', 10)
        self.filtered_velocity_publisher = self.create_publisher(TwistStamped, '/filtered/velocity', 10)
        self.filtered_pitch_publisher = self.create_publisher(Float32, '/filtered/pitch', 10)
        self.filtered_roll_publisher = self.create_publisher(Float32, '/filtered/roll', 10)
        self.filtered_heading_publisher = self.create_publisher(Float32, '/filtered/heading', 10)

        # Subscriptions
        self.create_subscription(NavSatFix, '/fix', self.navsatfix_callback, 1)
        self.create_subscription(TwistStamped, '/vel', self.twiststamped_callback, 1)
        self.create_subscription(MagneticField, '/imu/mag', self.mag_callback, 1)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 1)
        self.subscription = self.create_subscription(TimeReference,'/time_reference',self.time_callback,1)

    def time_callback(self, msg: TimeReference):
	# Extract the header stamp
        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec

        # Extract the time_ref
        self.time_ref_sec = msg.time_ref.sec + msg.time_ref.nanosec * 1e-9
        self.time_ref_nanosec = msg.time_ref.nanosec
        # Extract the source
        source = msg.source
    def navsatfix_callback(self, msg: NavSatFix):
        self.lon = msg.longitude
        self.lat = msg.latitude
        self.alt = msg.altitude
        self.time_sec = self.get_current_time(msg.header)
        self.update_ekf()

    def twiststamped_callback(self, msg: TwistStamped):
        self.vn = msg.twist.linear.x
        self.ve = msg.twist.linear.y
        self.vd = msg.twist.linear.z
        self.time_sec = self.get_current_time(msg.header)
        self.update_ekf()

    def mag_callback(self, msg: Imu):
        self.mx = msg.magnetic_field.x
        self.my = msg.magnetic_field.y
        self.mz = msg.magnetic_field.z
        self.update_ekf()

    def imu_callback(self, msg: Imu):
        self.p = msg.angular_velocity.x
        self.q = msg.angular_velocity.y
        self.r = msg.angular_velocity.z
        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.az = msg.linear_acceleration.z
        self.update_ekf()

    def get_current_time(self, header: Header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def update_ekf(self):
        # Ensure all data is available before updating EKF
        if None not in (self.lon, self.lat, self.vn, self.ve, self.vd,
                        self.mx, self.my, self.mz, self.p, self.q, self.r, self.ax, self.ay, self.az):
            import numpy as np
            import time
            self.ekf.update(
                np.uint64(time.time()), self.vn, self.ve, self.vd,
                self.lat, self.lon, 0.0,
                self.p, self.q, self.r,
                self.ax, self.ay, self.az,
                self.mx, self.my, self.mz
            )

            # Publish all filtered data
            self.publish_filtered_data()

    def publish_filtered_data(self):
        # Publish filtered GPS data
        filtered_fix_msg = NavSatFix()
        filtered_fix_msg.header.stamp = self.get_clock().now().to_msg()
        filtered_fix_msg.header.frame_id = "gps"
        filtered_fix_msg.latitude = self.ekf.get_latitude()
        filtered_fix_msg.longitude = self.ekf.get_longitude()
        filtered_fix_msg.altitude = 0.0
        self.filtered_fix_publisher.publish(filtered_fix_msg)

        # Publish filtered velocity
        filtered_velocity_msg = TwistStamped()
        filtered_velocity_msg.header.stamp = self.get_clock().now().to_msg()
        filtered_velocity_msg.header.frame_id = "gps"
        filtered_velocity_msg.twist.linear.x = self.ekf.get_vel_north()
        filtered_velocity_msg.twist.linear.y = self.ekf.get_vel_east()
        filtered_velocity_msg.twist.linear.z = 0.0
        self.filtered_velocity_publisher.publish(filtered_velocity_msg)

        # Log all filtered data
        self.get_logger().info(
            f"Filtered Data Published:\n"
            f"  GPS -> Latitude: {filtered_fix_msg.latitude}, Longitude: {filtered_fix_msg.longitude}, Altitude: {filtered_fix_msg.altitude}\n"
            f"  Velocity -> North: {filtered_velocity_msg.twist.linear.x}, East: {filtered_velocity_msg.twist.linear.y}, Down: {filtered_velocity_msg.twist.linear.z}\n"
        )

if __name__ == "__main__":
    rclpy.init(args=None)  # Moved this line before creating any nodes

    ekf = EKFNavINS()

    # Create an instance of the DataGetter node
    data_getter = DataGetter(ekf)  # Removed duplicate creation of DataGetter

    # Spin the node to keep it running
    try:
        rclpy.spin(data_getter)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        # Destroy the node explicitly and shut down ROS 2
        data_getter.destroy_node()
        rclpy.shutdown()
