import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from controller import Supervisor
import random
import math

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


class MyRobotDriver:
    """Robot that plays football."""
    def init(self, webots_node, properties):
        """Initialisation node to start the robot with Webots."""
        self.__robot = webots_node.robot
        print(self.__robot)
        self.__left_motor = self.__robot.getDevice("left wheel motor")
        self.__right_motor = self.__robot.getDevice("right wheel motor")

        self.__left_motor.setPosition(float("inf"))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float("inf"))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        # State machine states: reposition -> approach_ball -> push_ball -> wait -> reposition ...
        self.state: str = "reposition"
        self.wait_start_time: float = 0
        self.wait_duration: float = 2.0  # Wait for 2 seconds
        self.push_start_time: float = 0
        self.push_duration: float = 3.0  # Push for 3 seconds before waiting
        self.reposition_distance: float = 0.1  # Distance behind the ball

        rclpy.init(args=None)
        self.__node = rclpy.create_node("my_robot_main")
        self.__node.create_subscription(
            Twist,
            "cmd_vel",
            self.__cmd_vel_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )
        self.movementPublisher = self.__node.create_publisher(
            Twist, "cmd_vel", qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        # Publisher for ball position.
        self.ballPublisher = self.__node.create_publisher(
            Float32MultiArray, "/ball_pos", qos_profile=1
        )

        self.createBall()

        self.goals = 0
        self.leftWallNode = self.supervisor.getFromDef("LEFTGOAL")
        self.rightWallNode = self.supervisor.getFromDef("RIGHTGOAL")

        self.leftWallPos = self.leftWallNode.getPosition()
        self.rightWallPos = self.rightWallNode.getPosition()

        self.__node.get_logger().info(
            f"The left wall is at {self.leftWallNode.getPosition()}"
        )
        self.__node.get_logger().info(
            f"The right wall is at {self.rightWallNode.getPosition()}"
        )

        # Find the middle of the goal.
        self.MiddleOfGoal = (
            (self.leftWallPos[0] + self.rightWallPos[0]) / 2,
            (self.leftWallPos[1] + self.rightWallPos[1]) / 2,
        )

        self.__node.get_logger().info(f"Middle of the goal: {self.MiddleOfGoal}")

        self.puck = self.supervisor.getFromDef("PUCK")
        self.puck_translation_field = self.puck.getField("translation")

    def createBall(self):
        """Creates a new ball initially."""
        self.supervisor = Supervisor()

        ball_string = """DEF BALL Ball {translation 0.21 -0.270544 0.05, rotation 1 0 0 1.5707963267948966, radius 0.05
        }"""
        rootNode = self.supervisor.getRoot()
        rootNode.getField("children").importMFNodeFromString(-1, ball_string)
        self.ballNode = self.supervisor.getFromDef("BALL")
        self.ball_translation_field = self.ballNode.getField("translation")
        self.__node.get_logger().info("Ball has been created.")

    def publishBallPosition(self):
        """Publishes the ball's position to a ROS2 topic."""
        position = self.ballNode.getPosition()
        self.ballPos = Float32MultiArray()
        self.ballPos.data = position
        self.ballPublisher.publish(msg=self.ballPos)

    def dist(self) -> float:
        """Gets the distance between the ball and the puck."""
        puckPos = self.puck_translation_field.getSFVec3f()
        ballPos = self.ball_translation_field.getSFVec3f()
        return math.sqrt(
            (ballPos[0] - puckPos[0]) ** 2 + (ballPos[1] - puckPos[1]) ** 2
        )

    def movetoPoint(self, xPoint: float, yPoint: float):
        """Moves to a point xPoint, yPoint, by calculating the angle of approach."""
        # Proportional control logic from your last code snippet
        puckX, puckY = self.puck_translation_field.getSFVec3f()[0:2]
        rotation = self.puck.getField("rotation").getSFRotation()
        axis = rotation[:3]
        angle = rotation[3]

        # Determine heading angle based on rotation axis
        if axis[2] == 0:
            heading_angle = 0.0
        else:
            heading_angle = angle
            if axis[2] < 0:
                heading_angle = -heading_angle

        # Normalize heading_angle to [-π, π]
        heading_angle = (heading_angle + math.pi) % (2 * math.pi) - math.pi

        deltaX = xPoint - puckX
        deltaY = yPoint - puckY
        desired_heading = math.atan2(deltaY, deltaX)

        angle_diff = desired_heading - heading_angle
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        distance = math.hypot(deltaX, deltaY)

        # Gains and speeds
        Kp_linear = 0.5
        Kp_angular = 2.0
        max_linear_speed = 0.1
        max_angular_speed = 1.0

        linear_speed = min(Kp_linear * distance, max_linear_speed)
        angular_speed = max(
            -max_angular_speed, min(Kp_angular * angle_diff, max_angular_speed)
        )

        # Stop forward movement if angle is too large
        if abs(angle_diff) > math.pi / 2:
            linear_speed = 0.0

        command_message = Twist()
        command_message.linear.x = linear_speed
        command_message.angular.z = angular_speed

        self.__target_twist = command_message

        # self.__node.get_logger().info(
        #     f"movetoPoint Debug - Heading: {heading_angle:.2f}, Desired: {desired_heading:.2f}, "
        #     f"Angle Diff: {angle_diff:.2f}, Lin Speed: {linear_speed:.2f}, Ang Speed: {angular_speed:.2f}"
        # )

    def isColliding(self) -> bool:
        """Checks if the puck is colliding with the ball."""
        collision_distance = 0.07
        current_distance = self.dist()
        # self.__node.get_logger().info(f"Distance to Ball: {current_distance:.2f}")
        if current_distance < collision_distance:
            self.__node.get_logger().info("Collision occurred!")
            return True
        return False

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.publishBallPosition()

        ballPos = self.ball_translation_field.getSFVec3f()
        puckPos = self.puck_translation_field.getSFVec3f()
        goalPos = [self.MiddleOfGoal[0], self.MiddleOfGoal[1]]

        # Compute behind_ball_point
        dx_bg = goalPos[0] - ballPos[0]
        dy_bg = goalPos[1] - ballPos[1]
        magnitude = math.hypot(dx_bg, dy_bg)
        # Don't really understand this code here - Harry
        if magnitude == 0:
            magnitude = 1e-5
        unit_vector = [dx_bg / magnitude, dy_bg / magnitude]

        behind_ball_point = [
            ballPos[0] - unit_vector[0] * self.reposition_distance,
            ballPos[1] - unit_vector[1] * self.reposition_distance,
        ]

        current_time = self.supervisor.getTime()

        # self.__node.get_logger().info(f"State: {self.state}")
        # self.__node.get_logger().info(f"Robot Pos: ({puckPos[0]:.2f},{puckPos[1]:.2f}) Ball Pos: ({ballPos[0]:.2f},{ballPos[1]:.2f})")
        # self.__node.get_logger().info(f"Behind Ball Point: ({behind_ball_point[0]:.2f},{behind_ball_point[1]:.2f})")

        # Distances for logic
        dist_behind = math.hypot(
            behind_ball_point[0] - puckPos[0], behind_ball_point[1] - puckPos[1]
        )

        if self.state == "reposition":
            # Move behind the ball
            if dist_behind > 0.05:
                self.movetoPoint(behind_ball_point[0], behind_ball_point[1])
            else:
                # Once behind ball, approach the ball
                self.state = "approach_ball"

        elif self.state == "approach_ball":
            # Approach the ball until collision
            if self.isColliding():
                # Start pushing the ball
                self.push_start_time = current_time
                self.state = "push_ball"
            else:
                # Move towards the ball's point.
                self.movetoPoint(ballPos[0], ballPos[1])

        elif self.state == "push_ball":
            # Push the ball forward towards the goal line
            if self.isColliding():
                # Push if still colliding
                if (current_time - self.push_start_time) < self.push_duration:
                    self.movetoPoint(goalPos[0], goalPos[1])
                else:
                    # After pushing for push_duration seconds, go to wait
                    self.state = "wait"
                    self.wait_start_time = current_time
            else:
                # Lost contact with ball, re-approach
                self.state = "approach_ball"

        elif self.state == "wait":
            # Stop and wait
            self.__target_twist.linear.x = 0.0
            self.__target_twist.angular.z = 0.0

            if (current_time - self.wait_start_time) > self.wait_duration:
                # After waiting, reposition behind ball again
                self.state = "reposition"

        # Apply motor commands
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z
        command_motor_left = (
            forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS
        ) / WHEEL_RADIUS
        command_motor_right = (
            forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS
        ) / WHEEL_RADIUS
        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

        # Goal logic
        if self.ballPos.data[0] > 1.75:
            new_ball_value = [
                random.uniform(-1.35, 1.35),
                random.uniform(-0.7, 0.7),
                0.1,
            ]
            self.ball_translation_field.setSFVec3f(new_ball_value)
            self.ballNode.addForce([0, 0, 0], relative=False)
            self.ballNode.resetPhysics()

            new_puck_value = [
                random.uniform(-1.35, 1.35),
                random.uniform(-0.7, 0.7),
                0.001,
            ]
            self.puck_translation_field.setSFVec3f(new_puck_value)
            self.puck.resetPhysics()
            self.goals += 1
            self.state = "reposition"  # Reset cycle after scoring
