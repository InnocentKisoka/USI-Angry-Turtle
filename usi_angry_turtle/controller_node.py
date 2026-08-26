import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile

import sys
from math import pi, sin, cos, atan2, sqrt
import random
from threading import Thread
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
from std_srvs.srv import Empty


class TurtleState:
    WRITING = "writing"
    ANGRY = "angry"
    RETURNING = "returning"
    FINAL = "final"


class Move2GoalNode(Node):
    PEN_ON = (255, 255, 255, 3, 0)  # r, g, b, width, off
    PEN_OFF = (0, 0, 0, 0, 1)

    def __init__(self, tolerance=0.1, k1=0.5, k2=2.0):
        super().__init__('move2goal')
        self.get_logger().info("Move2GoalNode initialized")

        # Attributes for goal, tolerance, and current pose
        self.tolerance = tolerance
        self.k1 = k1  # Capture tolerance
        self.k2 = k2  # Pursuit tolerance
        self.current_pose = None
        self.state = TurtleState.WRITING
        self.total_turtles = 4
        self.turtles_alive = self.total_turtles
        self.goal_turtle_name = None
        self.writing_index = 0  # Track progress in writing path
        self.initial_pose = Pose(x=1.0, y=8.0, theta=float(3 * pi / 2))

        # Path to write "USI" (ensure all theta values are floats)
        self.path = [
            Pose(x=1.0, y=5.0, theta=float(5 * pi / 3)), Pose(x=2.0, y=4.0, theta=0.0), Pose(x=3.0, y=4.0, theta=float(pi / 6)),
            Pose(x=4.0, y=5.0, theta=float(pi / 2)), Pose(x=4.0, y=8.0, theta=0.0),
            Pose(x=8.0, y=8.0, theta=float(5 * pi / 6)), Pose(x=5.5, y=7.0, theta=float(5 * pi / 4)),
            Pose(x=5.5, y=6.3, theta=float(11 * pi / 6)), Pose(x=8.0, y=5.7, theta=float(7 * pi / 4)),
            Pose(x=8.0, y=5.0, theta=float(4 * pi / 3)), Pose(x=5.5, y=4.0, theta=0.0),
            Pose(x=9.5, y=4.0, theta=float(pi / 2)), Pose(x=9.5, y=8.0, theta=float(pi / 2))
        ]
        self.pen_offline = [5, 11]  # Points where pen is off

        # Track recently spawned turtles to ignore them temporarily
        self.recently_spawned = {}  # {turtle_name: spawn_time}

        # Writing focus mode: ignore turtles for a short period while writing each letter
        self.focus_mode = False
        self.focus_start_time = 0.0
        self.focus_duration = 3.0  # Focus for 3 seconds per segment

        # Publisher for turtle1 velocity
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber for turtle1 pose
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Dictionary to store target turtle poses
        self.pose_target_turtles = {}

        # Service clients
        self.setpen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.kill_client = self.create_client(Kill, '/kill')
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.clear_client = self.create_client(Empty, '/clear')

        # Wait for services to be available
        self.setpen_client.wait_for_service()
        self.kill_client.wait_for_service()
        self.spawn_client.wait_for_service()
        self.clear_client.wait_for_service()

        # Subscribe to teleoperated turtle (turtle2) pose
        self.create_subscription(Pose, '/turtle2/pose', lambda msg: self.turtle_target_pose(msg, 'turtle2'), 10)

    def start_moving(self):
        # Start a timer to periodically call move_callback
        self.timer = self.create_timer(0.1, self.move_callback)
        self.done_future = Future()

        # Start a thread to spawn and control target turtles
        thread = TargetsController(args=[self])
        thread.start()

        return self.done_future

    def pose_callback(self, msg):
        """Callback for turtle1 pose updates."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)
        self.get_logger().info(f"current pose is: {self.current_pose}")

    def turtle_target_pose(self, msg, name):
        """Callback for target turtle pose updates."""
        msg.x = round(msg.x, 4)
        msg.y = round(msg.y, 4)
        self.pose_target_turtles[name] = msg
        self.get_logger().info(f"Updated pose for {name}: x={msg.x}, y={msg.y}")

    def euclidean_distance(self, goal_pose, current_pose=None):
        """Euclidean distance between two poses."""
        if current_pose is None:
            current_pose = self.current_pose
        return sqrt(pow((goal_pose.x - current_pose.x), 2) + pow((goal_pose.y - current_pose.y), 2))

    def angular_difference(self, goal_theta, current_theta):
        """Shortest rotation from current_theta to goal_theta."""
        return atan2(sin(goal_theta - current_theta), cos(goal_theta - current_theta))

    def linear_vel(self, goal_pose, constant=1.5):
        """Proportional linear velocity."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """Steering angle toward goal pose."""
        return atan2(goal_pose.y - self.current_pose.y, goal_pose.x - self.current_pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """Proportional angular velocity."""
        goal_theta = self.steering_angle(goal_pose)
        return constant * self.angular_difference(goal_theta, self.current_pose.theta)

    def angular_vel_rot(self, goal_pose, constant=6):
        """Angular velocity for rotation to goal orientation."""
        return constant * self.angular_difference(goal_pose.theta, self.current_pose.theta)

    def set_pen(self, on=True):
        """Set the pen state (on/off)."""
        req = SetPen.Request()
        if on:
            req.r, req.g, req.b, req.width, req.off = self.PEN_ON
        else:
            req.r, req.g, req.b, req.width, req.off = self.PEN_OFF
        self.setpen_client.call_async(req)

    def stop_walking(self):
        """Stop the turtle."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(cmd_vel)

    def move_to_goal(self, goal_pose):
        """Move the turtle to the goal pose."""
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_vel(goal_pose)
        cmd_vel.angular.z = self.angular_vel(goal_pose)
        self.vel_publisher.publish(cmd_vel)

    def rotate_to_goal(self, goal_pose, rot_tolerance=0.017):
        """Rotate the turtle to the goal orientation."""
        cmd_vel = Twist()
        if abs(self.angular_difference(goal_pose.theta, self.current_pose.theta)) >= rot_tolerance:
            cmd_vel.angular.z = self.angular_vel_rot(goal_pose)
            self.vel_publisher.publish(cmd_vel)
            return False
        self.stop_walking()
        return True

    def get_closer_turtle(self):
        """Find the closest target turtle, ignoring recently spawned ones."""
        min_distance = float("inf")
        min_turtle = None
        current_time = time.time()
        for name, pose in self.pose_target_turtles.items():
            # Skip recently spawned turtles (within 5 seconds)
            if name in self.recently_spawned:
                spawn_time = self.recently_spawned[name]
                if current_time - spawn_time < 5.0:  # 5-second cooldown
                    continue
            distance = self.euclidean_distance(pose)
            if distance < min_distance:
                min_distance = distance
                min_turtle = name
        self.goal_turtle_name = min_turtle
        return min_turtle

    def get_future_pose(self):
        """Compute a future pose m meters ahead of the target turtle."""
        target_pose = self.pose_target_turtles[self.goal_turtle_name]
        constant = 1.1
        m = constant * target_pose.linear_velocity * self.euclidean_distance(target_pose)
        goal_pose = Pose()
        goal_pose.x = target_pose.x + m * cos(target_pose.theta)
        goal_pose.y = target_pose.y + m * sin(target_pose.theta)
        return goal_pose, target_pose

    def spawn_far_away(self):
        """Spawn a new turtle at least k2 meters away from the current position."""
        attempts = 0
        max_attempts = 100
        while attempts < max_attempts:
            x = random.uniform(1, 10)
            y = random.uniform(1, 10)
            temp_pose = Pose(x=x, y=y)
            if self.current_pose is None or self.euclidean_distance(temp_pose) > self.k2:
                return x, y
            attempts += 1
        # Fallback: return a position far away if we can't find one
        return 10.0, 10.0  # Top-right corner of the Turtlesim window

    def writing(self):
        """Write 'USI' by following the predefined path."""
        if self.writing_index >= len(self.path):
            if self.turtles_alive > 0:
                self.get_logger().info("USI written, but turtles remain. Clearing and continuing...")
                self.clear_client.call_async(Empty.Request())
                self.writing_index = 0
                self.state = TurtleState.RETURNING
            else:
                self.state = TurtleState.FINAL
            return

        # Check if we're in focus mode (ignoring turtles)
        current_time = time.time()
        if self.focus_mode:
            if current_time - self.focus_start_time >= self.focus_duration:
                self.focus_mode = False
                self.get_logger().info("Focus mode ended, checking for nearby turtles again")
            else:
                self.get_logger().debug("In focus mode, ignoring turtles")

        goal_pose = self.path[self.writing_index]
        if self.writing_index in self.pen_offline:
            self.set_pen(False)
        else:
            self.set_pen(True)

        if self.euclidean_distance(goal_pose) >= self.tolerance:
            self.move_to_goal(goal_pose)
            if not self.focus_mode and self.turtles_alive > 0:
                self.goal_turtle_name = self.get_closer_turtle()
                if self.goal_turtle_name and self.euclidean_distance(self.pose_target_turtles[self.goal_turtle_name]) < self.k2:
                    self.set_pen(False)
                    self.get_logger().info(f"Turtle {self.goal_turtle_name} too close, becoming angry...")
                    self.state = TurtleState.ANGRY
        else:
            if self.rotate_to_goal(goal_pose):
                self.writing_index += 1
                # Enter focus mode after reaching a waypoint (except at pen-off points)
                if self.writing_index not in self.pen_offline:
                    self.focus_mode = True
                    self.focus_start_time = time.time()
                    self.get_logger().info(f"Reached waypoint {self.writing_index}, entering focus mode for {self.focus_duration} seconds")

    def become_angry(self):
        """Pursue the offender turtle and kill it if close enough."""
        goal_pose, target_pose = self.get_future_pose()
        if self.euclidean_distance(target_pose) >= self.k1:
            self.move_to_goal(goal_pose)
        else:
            # Kill the turtle
            req = Kill.Request()
            req.name = self.goal_turtle_name
            self.kill_client.call_async(req)
            self.get_logger().info(f"Killed {self.goal_turtle_name}")
            self.turtles_alive -= 1
            del self.pose_target_turtles[self.goal_turtle_name]

            # Spawn a new turtle far away
            x, y = self.spawn_far_away()
            spawn_req = Spawn.Request()
            spawn_req.x = x
            spawn_req.y = y
            spawn_req.theta = 0.0
            spawn_req.name = self.goal_turtle_name
            spawn_future = self.spawn_client.call_async(spawn_req)
            spawn_future.add_done_callback(lambda future: self.spawn_callback(future, self.goal_turtle_name, x, y))
            self.turtles_alive += 1

            # Mark the turtle as recently spawned
            self.recently_spawned[self.goal_turtle_name] = time.time()

            self.state = TurtleState.RETURNING
            self.goal_turtle_name = None

    def spawn_callback(self, future, name, x, y):
        """Callback for spawn service response."""
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle {name} at x={x}, y={y}")
        except Exception as e:
            self.get_logger().warn(f"Failed to spawn turtle {name}: {str(e)}")

    def return_to_position(self):
        """Return to the last writing position or initial position."""
        if self.euclidean_distance(self.initial_pose) >= self.tolerance:
            self.move_to_goal(self.initial_pose)
            if self.turtles_alive > 0:
                self.goal_turtle_name = self.get_closer_turtle()
                if self.goal_turtle_name and self.euclidean_distance(self.pose_target_turtles[self.goal_turtle_name]) < self.k2:
                    self.get_logger().info(f"Turtle {self.goal_turtle_name} too close while returning, becoming angry...")
                    self.state = TurtleState.ANGRY
        else:
            if self.rotate_to_goal(self.initial_pose):
                self.state = TurtleState.WRITING

    def move_callback(self):
        """Main control loop."""
        if self.current_pose is None:
            return

        if self.state == TurtleState.WRITING:
            self.writing()
        elif self.state == TurtleState.ANGRY:
            self.become_angry()
        elif self.state == TurtleState.RETURNING:
            self.return_to_position()
        elif self.state == TurtleState.FINAL:
            self.get_logger().info("All turtles killed and USI written. Shutting down...")
            self.stop_walking()
            self.done_future.set_result(True)


class TargetsController(Thread):
    def __init__(self, args=()):
        super().__init__()
        self.node = args[0]
        self.target_velocity_publishers = {}
        self.total_turtles = self.node.total_turtles
        self.c_lin = 0
        self.c_ang = 0

    def spawn_callback(self, future, name, x, y):
        """Callback for spawn service response."""
        try:
            response = future.result()
            self.node.get_logger().info(f"Spawned turtle with name: {name}")
        except Exception as e:
            self.node.get_logger().warn(f"Failed to spawn turtle {name}: {str(e)}")

    def spawn_turtles(self, t):
        """Spawn a target turtle at a random position."""
        name = f'turtleTarget{t}'
        req = Spawn.Request()
        req.x = random.uniform(1, 10)
        req.y = random.uniform(1, 10)
        req.theta = 0.0
        req.name = name
        spawn_future = self.node.spawn_client.call_async(req)
        spawn_future.add_done_callback(lambda future: self.spawn_callback(future, name, req.x, req.y))

        # Subscribe to the turtle's pose
        self.node.create_subscription(Pose, f'/{name}/pose', lambda msg, n=name: self.node.turtle_target_pose(msg, n), 10)

        # Create a publisher for the turtle's velocity
        self.target_velocity_publishers[name] = self.node.create_publisher(Twist, f'/{name}/cmd_vel', 10)

        # Set pen off
        pen_client = self.node.create_client(SetPen, f'/{name}/set_pen')
        pen_client.wait_for_service()
        pen_req = SetPen.Request()
        pen_req.r, pen_req.g, pen_req.b, pen_req.width, pen_req.off = self.node.PEN_OFF
        pen_client.call_async(pen_req)

    def random_walking(self, t):
        """Move a target turtle randomly with reduced speed."""
        name = f'turtleTarget{t}'
        cmd_vel = Twist()
        # Reduced speed: linear velocity between 0.5 and 1.5, angular velocity between 0.5 and 1
        cmd_vel.linear.x = self.c_lin + 1.5 - random.random()  # Range: 0.5 to 1.5
        cmd_vel.angular.z = self.c_ang + 1 - random.random() * 0.5  # Range: 0.5 to 1
        self.c_lin = t * cmd_vel.linear.x / self.total_turtles
        self.c_ang = t * cmd_vel.angular.z / self.total_turtles
        self.target_velocity_publishers[name].publish(cmd_vel)

    def run(self):
        """Spawn and move target turtles."""
        for t in range(self.total_turtles):
            self.spawn_turtles(t)

        timer = self.node.create_timer(0.1, lambda: [self.random_walking(t) for t in range(self.total_turtles)])


def main():
    rclpy.init(args=sys.argv)
    node = Move2GoalNode()
    done = node.start_moving()
    rclpy.spin_until_future_complete(node, done)


if __name__ == '__main__':
    main()
