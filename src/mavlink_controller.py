# A small MAVLink helper to send body-frame velocity setpoints to a vehicle (works with PX4 / SITL). 

# src/mavlink_controller.py
import time
from pymavlink import mavutil

class MAVLinkController:
    def __init__(self, connection_str="udp:127.0.0.1:14540", target_system=1, target_component=1):
        """
        connection_str examples:
          - SITL PX4: "udp:127.0.0.1:14540"
          - Serial: "/dev/ttyUSB0"
        """
        self.conn = mavutil.mavlink_connection(connection_str)
        # wait for heartbeat
        print("Waiting for heartbeat from vehicle...")
        self.conn.wait_heartbeat(timeout=30)
        print("Heartbeat received.")
        self.target_system = target_system
        self.target_component = target_component

    def send_velocity_body(self, vx, vy, vz, duration=1.0):
        """
        Send velocity in body frame (m/s). This uses SET_POSITION_TARGET_LOCAL_NED message
        with type_mask to ignore position and only use velocity.
        """
        # Frame: MAV_FRAME_BODY_NED (8)
        # type_mask: set bits to ignore pos and accel/force and yaw -> we pass velocities
        # type_mask bits: https...
        TYPE_MASK = 0b0000111111000111  # ignore position, accel, yaw, only velocity enabled
        # timestamp, target system/component
        self.conn.mav.set_position_target_local_ned_send(
            int(time.time()*1e3),
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            TYPE_MASK,
            0,0,0,     # x,y,z positions (ignored)
            float(vx), float(vy), float(vz),  # vx, vy, vz (m/s)
            0,0,0,     # afx, afy, afz
            0,0        # yaw, yaw_rate
        )

    def arm(self):
        self.conn.arducopter_arm()
    def disarm(self):
        self.conn.arducopter_disarm()
