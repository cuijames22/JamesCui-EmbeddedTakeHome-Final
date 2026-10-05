"""
Odometry Node — your implementation goes here.

Run via:
    python sim/launch.py
    python sim/launch.py --visualize      # with live plot
    python sim/launch.py --no-faults      # clean feeds while getting started

This file uses rclpy_lite, a lightweight simulator shim that mirrors the real
rclpy (ROS 2 Python) API. The class names, method signatures and message field
names match real ROS 2, and the import paths differ:

    Real ROS 2:     import rclpy / from rclpy.node import Node
    This shim:      import rclpy_lite as rclpy / from rclpy_lite.node import Node

(The README lists the few small differences, e.g. header stamps are plain floats.)

Two things that make life easier here:
  - Callbacks never run at the same time as each other (real rclpy does the same
    with its default executor), so you do not need locks around your state.
  - If a callback raises an exception you will see a traceback in the terminal.
    The run keeps going, so read the terminal, don't just look for a crash.
"""

import sys

if __name__ == "__main__":
    # This file is loaded BY the simulator; running it directly can't work.
    # (Delete this guard if you ever port the node to a real ROS 2 install.)
    sys.exit("Don't run this file directly. Start everything with:\n\n    python sim/launch.py\n")

import math
import time

import rclpy_lite as rclpy
from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Message types provided by the simulator
from sim.messages import WheelTicks, GPSEstimate

# Standard ROS message types (mirrored by the shim)
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

#importing NumPy for polynomial regression
import numpy as np

# --------------------------------------------------------------------------
# Constants  (do not change — these match the real rover's encoder setup)
# --------------------------------------------------------------------------
WHEEL_RADIUS_M = 0.075          # metres
TICKS_PER_REVOLUTION = 360
DIST_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS_M) / TICKS_PER_REVOLUTION  # ~0.00131 m

# Subscribes to /wheel_ticks and /gps_estimate
# Publishes /odom
# Fuses encoder and gps readings to determine an estimate of robot position
class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")

        #Variables first since I got error if they were after
        self.x: float = 0.0                  # fused position, metres (east)
        self.y: float = 0.0                  # fused position, metres (north)
        self.heading: float = 0.0            # radians counterclockwise starting EAST
        self.angularSpeed : float = 0.0      # angular speed
        self.velocity : float =0.0           # linear speed (no direction)
        self.distance : float= 0.0           # distance traveled for wheel ticks

        #All the last variables
        self.last_tick_count: int | None = None
        self.last_tick_time: float | None = None
        self.last_gps_time: float | None = None
        self.last_gps_x : float | None = None
        self.last_gps_y : float | None = None
        self.last_heading: float = 0.0
        self.last_wheel_time: float | None = None

        #All the delta variables
        self.delta_tick_count : int | None = None
        self.delta_tick_time : float | None = None
        self.delta_gps_time : float = 1.0
        self.delta_gps_heading : float = 0.0

        # Miscellaneous variables
        self.w_gps : float = 1.0                    #Weighting
        self.wheel_msg_count: int = 0               #Wheel msg count
        self.gps_msg_count : int = 0                #GPS msg count
        self.start_time: float = time.monotonic()   #Start time

        # Variables used for regressions
        self.last10GPS_Times : np.array = np.zeros(5)
        self.last10GPSx : np.array = np.zeros(5)
        self.last10GPSy : np.array = np.zeros(5)
        self.coefficientsX : np.ndarray = [0, 0]
        self.coefficientsY : np.ndarray = [0, 0]
# ------------------------------------------------------------------
        # Subscribers

        #qos that I am using for wheel ticks and odometry
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Subscriber, publisher and timer definitions
        self.tick_sub = self.create_subscription(WheelTicks, "/wheel_ticks", self.wheel_tick_callback, qos)
        self.gps_sub = self.create_subscription(GPSEstimate, "/gps_estimate", self.gps_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos)
        self.monitor_timer = self.create_timer(1.0, self.monitoring_callback)

    # Called each time a wheel tick message arrives (~50 Hz).
    #Uses heading calculated by gps to figure out direction
    #Checks for duplicate readings
    def wheel_tick_callback(self, msg: WheelTicks) -> None:

        #Just checking if I am actually getting around 50 Hz (Test)
        """
        self.wheel_msg_count += 1
        hz = self.wheel_msg_count/ (time.monotonic() - self.start_time)
        print("Wheel tick rate:", hz, "Hz")
        """

        # Recording count of wheels msgs
        self.wheel_msg_count += 1

        #If there hasn't been a measurement yet set it to first
        if self.last_tick_count is None or self.last_tick_time is None:
            self.last_tick_count = msg.tick_count
            self.last_tick_time = msg.timestamp
            return

        #Find delta variables and set current tick/time to last tick/time
        self.delta_tick_count = msg.tick_count - self.last_tick_count
        self.delta_tick_time = msg.timestamp - self.last_tick_time
        self.last_tick_count = msg.tick_count
        self.last_tick_time = msg.timestamp

        #If is the same rating then don't do anything more
        #This was supposed to be step 4, but I accidentally did it in step 3
        #Pretty much if the change in the count or the time is the same it will not output anything to the publisher
        #I cannot detect dropped ticks because there is no difference in the tick_time or abnormality in the tick count
        #It is purely a 1 number difference that I cannot detect since delta_tick_count varies
        if self.delta_tick_count == 0 or self.delta_tick_time == 0:
            return

        #Find the encoder velocity and distance
        self.velocity = self.delta_tick_count / self.delta_tick_time
        self.distance = self.delta_tick_count * DIST_PER_TICK
        self.x+=self.distance*(math.cos(self.heading))
        self.y+=self.distance*(math.sin(self.heading))

        #Make the message to publish
        self.publish_odometry()
        pass

    """
     Called each time a GPS estimate arrives (~1 Hz, with occasional outages).
     Calculates heading
     Calculates a predicted position for the rover
    """
    def gps_callback(self, msg: GPSEstimate) -> None:

        #If there hasn't been a measurement yey set it to first and set variables
        if self.last_gps_time is None:
            self.last_gps_time = msg.timestamp
            self.last_gps_x = msg.x
            self.last_gps_y = msg.y
            self.last10GPSx[0] = msg.x
            self.last10GPSy[0] = msg.y
            self.last10GPS_Times[0] = msg.timestamp
            self.gps_msg_count += 1
            self.last_heading = 0.0
            return

        #If there isn't 5 or more total gps points yet fill up the array of points
        if self.gps_msg_count < 5:
            self.last10GPS_Times[self.gps_msg_count] = msg.timestamp
            self.last10GPSx[self.gps_msg_count] = msg.x
            self.last10GPSy[self.gps_msg_count] = msg.y
            self.coefficientsX = np.polyfit(self.last10GPS_Times[0:self.gps_msg_count + 1], self.last10GPSx[0:self.gps_msg_count + 1], deg=1)
            self.coefficientsY = np.polyfit(self.last10GPS_Times[0:self.gps_msg_count + 1], self.last10GPSy[0:self.gps_msg_count + 1], deg=1)
        # Otherwise, shift all the points down so you have the 5 most recent
        else:
            for i in range(4):
                self.last10GPSx[i] = self.last10GPSx[i+1]
                self.last10GPSy[i] = self.last10GPSy[i+1]
                self.last10GPS_Times[i] = self.last10GPS_Times[i + 1]
            self.last10GPSx[4] = msg.x
            self.last10GPSy[4] = msg.y
            self.last10GPS_Times[4] = msg.timestamp
            # Create a regression for x and y relative to time
            self.coefficientsX = np.polyfit(self.last10GPS_Times, self.last10GPSx, deg=1)
            self.coefficientsY = np.polyfit(self.last10GPS_Times, self.last10GPSy, deg=1)

        # Calculate heading using arctan with the derivatives and update other variables
        self.heading = math.atan2(self.coefficientsY[0], self.coefficientsX[0])

        # Update delta Variables
        self.delta_gps_heading = self.heading - self.last_heading
        self.delta_gps_time = msg.timestamp - self.last_gps_time
        self.last_heading = self.heading

        # Simple weighting
        self.w_gps = (1-msg.covariance)*0.5

        # Weighting position based off w_gpa, previous calculated pos and gps pos
        self.x = (1 - self.w_gps) * self.x + self.w_gps * msg.x
        self.y = (1 - self.w_gps) * self.y + self.w_gps * msg.y
        self.last_gps_time = msg.timestamp

        # Increases count at the end
        self.gps_msg_count +=1
        pass

    # This function publishes the odometry to the ROS system
    def publish_odometry(self) -> None:
        # Create the msg that will be used by ROS
        msgs = Odometry()
        msgs.header.frame_id = "odom"
        msgs.child_frame_id = "base_link"
        msgs.pose.pose.position.x = self.x
        msgs.pose.pose.position.y = self.y
        msgs.pose.pose.orientation.x = 0.0
        msgs.pose.pose.orientation.y = 0.0
        msgs.pose.pose.orientation.w = math.cos(self.heading / 2)
        msgs.pose.pose.orientation.z = math.sin(self.heading / 2)
        msgs.twist.twist.linear.x = self.velocity
        msgs.twist.twist.angular.z = self.delta_gps_heading / self.delta_gps_time
        self.odom_pub.publish(msgs)
        pass

    # A telemetry statement for the terminal that shows data to be interpreted in testing
    def monitoring_callback(self) -> None:
        if (time.monotonic() - self.start_time) >= 1:
            hz = self.wheel_msg_count/(time.monotonic() - self.start_time)
            self.wheel_msg_count = 0
            self.start_time = time.monotonic()
            print (f"[odom] post=({self.x:.2f}, {self.y:.2f}m  enc = {self.delta_tick_time:.2f}"
                   f"s ago @{hz:.2f}Hz  gps= {self.delta_gps_time:.2f}s ago")
        pass

def main() -> None:
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
