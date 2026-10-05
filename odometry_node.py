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

# --------------------------------------------------------------------------
# Constants  (do not change — these match the real rover's encoder setup)
# --------------------------------------------------------------------------
WHEEL_RADIUS_M = 0.075          # metres
TICKS_PER_REVOLUTION = 360
DIST_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS_M) / TICKS_PER_REVOLUTION  # ~0.00131 m


class OdometryNode(Node):
    """
    Fuses wheel encoder ticks with GPS estimates to produce odometry.

    Subscribes:
        /wheel_ticks   (sim.messages.WheelTicks)   ~50 Hz, BEST_EFFORT
        /gps_estimate  (sim.messages.GPSEstimate)   ~1 Hz,  BEST_EFFORT

    Publishes:
        /odom          (nav_msgs/Odometry)          matches input rate
    """

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
        self.last_gps_heading: float = 0.0
        self.last_wheel_time: float | None = None

        #All the delta variables
        self.delta_tick_count : int | None = None
        self.delta_tick_time : float | None = None
        self.delta_gps_time : float = 1.0
        self.delta_gps_heading : float = 0.0

        self.w_gps : float = 1.0                    #Weighting
        self.wheel_msg_count: int = 0               #Wheel msg count
        self.start_time: float = time.monotonic()   #Start time
# ------------------------------------------------------------------
        # Subscribers

        #qos that I am using for wheelticks and odometry
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Subscriber, publisher and timer definitions
        self.tick_sub = self.create_subscription(WheelTicks, "/wheel_ticks", self.wheel_tick_callback, qos)
        self.gps_sub = self.create_subscription(GPSEstimate, "/gps_estimate", self.gps_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos)
        self.monitor_timer = self.create_timer(1.0, self.monitoring_callback)

        # IMPORTANT: Check what QoS the encoder publisher uses (see
        # sim/encoder_publisher.py). A mismatch means you will receive
        # zero messages — with no error or warning, just silence.
        # This is intentional and mirrors real ROS 2 behaviour.
        #
        # Hint: the QoS profile you need looks like:
        #   QoSProfile(reliability=ReliabilityPolicy.???, depth=10)
        #
        # self.tick_sub = self.create_subscription(
        #     WheelTicks, "/wheel_ticks", self.wheel_tick_callback, ???
        # )
        #
        # self.gps_sub = self.create_subscription(GPSEstimate, "/gps_estimate", self.gps_callback, qos)
        # # self.gps_sub = self.create_subscription(
        #     GPSEstimate, "/gps_estimate", self.gps_callback, ???
        # )

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        #
        # self.odom_pub = self.create_publisher(Odometry, "/odom", qos)
        # # ------------------------------------------------------------------
        # # Timer — 1 Hz monitoring output
        # # ------------------------------------------------------------------
        # #
        # self.monitor_timer = self.create_timer(1.0, self.monitoring_callback)
        # use time.monotonic() to measure durations

    # -----------------------------------------------------------------------
    # Wheel encoder callback
    # -----------------------------------------------------------------------
    def wheel_tick_callback(self, msg: WheelTicks) -> None:
        """
        Called each time a wheel tick message arrives (~50 Hz).

        Convert the change in tick count to a distance, then update your
        position estimate and publish odometry.

        Args:
            msg.tick_count   Cumulative encoder ticks (monotonically increases).
            msg.timestamp    Wall-clock seconds when the reading was taken.
                             Subject to slow clock drift — see README.

        Noise to handle:
            DUPLICATE messages   The same (tick_count, timestamp) pair is
                                 occasionally re-sent within a few ms.
                                 If you compute velocity as delta_ticks / delta_time
                                 and delta_time is 0, you get a NaN, an infinite
                                 velocity, or a ZeroDivisionError. Check for this.

            DROPPED ticks        Now and then the encoder misses one tick (~1.3mm)
                                 and the cumulative counter is simply 1 lower than
                                 it should be. The data never shows a gap, so you
                                 cannot spot an individual drop. The error just
                                 adds up (~6cm per minute), and GPS fusion is your
                                 correction. (Think about why you can't see it.)

            CLOCK DRIFT          msg.timestamp wanders slowly away from the real
                                 time. Use differences between timestamps as
                                 durations; never compare them with time.time().

        Useful constants:
            DIST_PER_TICK   metres per encoder tick (~0.00131 m)
        """
        # TODO: implement

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
        #This was supposed to be step 4 but I accidentally did it in step 3
        #Pretty much if the change in the count or the time is the same it will not output anything to the publisher
        #I cannot detect dropped ticks because there is no difference in the tick_time or abnormality in the tick count
        #It is purely a 1 number difference that I cannot detect since delta_tick_count varies
        if self.delta_tick_count == 0 or self.delta_tick_time == 0:
            return

        #Find the encoder velocity and distance
        self.velocity = self.delta_tick_count / self.delta_tick_time
        self.distance = self.delta_tick_count * DIST_PER_TICK
        self.x+=self.distance*(math.sin(self.heading))
        self.y+=self.distance*(math.cos(self.heading))

        #Make the message to publish
        self.publish_odometry()
#
        # Suggested approach:
        #   1. Handle first message (initialize last_tick_count / last_tick_time).
        #   2. Compute delta_ticks = msg.tick_count - self.last_tick_count
        #   3. Guard against duplicate (delta_ticks == 0 and delta_time ≈ 0).
        #   4. Convert ticks to distance: distance = delta_ticks * DIST_PER_TICK
        #   5. Update self.x and self.y: the rover moves `distance` in the direction
        #      it is FACING. Ticks tell you how far, not which way. Where does your
        #      heading come from and how does it change? See "Heading is not
        #      measured" in the README before you write this line.
        #   6. Call self.publish_odometry().
        pass

    # -----------------------------------------------------------------------
    # GPS callback
    # -----------------------------------------------------------------------

    def gps_callback(self, msg: GPSEstimate) -> None:
        """
        Called each time a GPS estimate arrives (~1 Hz, with occasional outages).

        Fuse the GPS absolute position with your wheel-derived position.
        A weighted average is sufficient — no EKF required.

        Args:
            msg.x           GPS east position (metres from start).
            msg.y           GPS north position (metres from start).
            msg.timestamp   Seconds since epoch when the reading was taken.
            msg.covariance  Uncertainty (m²). Higher = less reliable.
                            Use this to adjust how much you trust the GPS
                            reading relative to your wheel estimate.

        Design question:
            GPS is noisy but absolute (no accumulated drift).
            Wheel odometry drifts but is precise over short distances.
            How do you weight them? Should the weighting change over time?
            (That's Stretch Goal B — but even a fixed weighting is fine here.)
        """

        #If there hasn't been a measurement yey set it to first
        if self.last_gps_time is None:
            self.last_gps_time = msg.timestamp
            self.last_gps_x = msg.x
            self.last_gps_y = msg.y
            return

        #Find delta variables and set current tick/time to last tick/time
        self.w_gps = 1-msg.covariance
        self.heading = math.atan2(msg.y-self.last_gps_y, msg.x - self.last_gps_x)

        #Calculate Deltas for GPS
        self.delta_gps_time = msg.timestamp - self.last_gps_time
        self.delta_gps_heading = self.heading - self.last_gps_heading

        # Weighting position based off w_gpa, previous calculated pos and gps pos
        self.x = (1 - self.w_gps) * self.x + self.w_gps * msg.x
        self.y = (1 - self.w_gps) * self.y + self.w_gps * msg.y
        self.last_gps_time = msg.timestamp

        # Suggested approach:
        #   1. Compute a weight w_gps based on msg.covariance (lower cov = higher trust).
        #   2. Blend: self.x = (1 - w_gps) * self.x + w_gps * msg.x
        #             self.y = (1 - w_gps) * self.y + w_gps * msg.y
        #   3. Update self.last_gps_time = msg.timestamp
        pass

    # -----------------------------------------------------------------------
    # Odometry publisher
    # -----------------------------------------------------------------------

    def publish_odometry(self) -> None:
        """
        Build a nav_msgs/Odometry message from the current state and publish it.

        The Odometry message layout (real ROS 2 spec, mirrored by the shim):
            msg.header.stamp         float, seconds since epoch
            msg.header.frame_id      str,   "odom"
            msg.child_frame_id       str,   "base_link"
            msg.pose.pose.position.x float, metres east
            msg.pose.pose.position.y float, metres north
            msg.pose.pose.orientation  Quaternion(x, y, z, w)
                For a 2D robot rotating around z: yaw → quaternion:
                    z = sin(heading / 2)
                    w = cos(heading / 2)
                    x = 0, y = 0
            msg.twist.twist.linear.x float, forward speed (m/s)
            msg.twist.twist.angular.z float, yaw rate (rad/s)

        Look up the real nav_msgs/Odometry definition for the full field list.
        Not every field is required — position is the minimum.
        """

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

    # -----------------------------------------------------------------------
    # Monitoring callback  (1 Hz)
    # -----------------------------------------------------------------------

    def monitoring_callback(self) -> None:
        """
        Print a one-line status update to the terminal once per second.

        Required output (see README rubric):
            - Seconds since last /wheel_ticks message
            - Seconds since last /gps_estimate message
            - Measured /wheel_ticks receive rate vs expected ~50 Hz
            - Current fused position (x, y)

        Tip: record time.monotonic() each time a message ARRIVES and subtract.
        Don't use msg.timestamp for this: the encoder's clock drifts.

        Example format (feel free to change the layout):
            [odom] pos=(1.23, 0.45)m  enc=0.02s ago @49.8Hz  gps=0.91s ago
        """


        if (time.monotonic() - self.start_time) >= 1:
            hz = self.wheel_msg_count/(time.monotonic() - self.start_time)
            self.wheel_msg_count = 0
            self.start_time = time.monotonic()
            print (f"[odom] post=({self.x:.2f}, {self.y:.2f}m  enc = {self.delta_tick_time:.2f}"
                   f"s ago @{hz:.2f}Hz  gps= {self.delta_gps_time:.2f}s ago")

        pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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
