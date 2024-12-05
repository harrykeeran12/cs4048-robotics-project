import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from controller import Supervisor
import random
import math

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


class MyRobotDriver:
    def init(self, webots_node, properties):
        """Initialisation node to move the robot."""
        self.__robot = webots_node.robot
        print("Supervisor initialized!!")

        print(self.__robot)
        self.__left_motor = self.__robot.getDevice("left wheel motor")
        self.__right_motor = self.__robot.getDevice("right wheel motor")

        self.__left_motor.setPosition(float("inf"))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float("inf"))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node("my_robot_main")
        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)
        # Publisher for ball position.
        self.ballPublisher = self.__node.create_publisher(
            Float32MultiArray, "/ball_pos", qos_profile=1
        )

        self.movementPublisher = self.__node.create_publisher(Twist, "cmd_vel", 1)

        # create the ball and store some of its details as an attribute of this class instance
        # ideally we would have a ball class that stores some of this but idc
        self.createBall()
        # Get the translation field for the ball, so we can amend it later.
        self.ball_translation_field = self.ballNode.getField("translation")

        # counts the number of goals, currently we dont do anything with this
        self.goals = 0
        # Gets the node of the walls.
        self.leftWallNode = self.supervisor.getFromDef("LEFTGOAL")
        self.rightWallNode = self.supervisor.getFromDef("RIGHTGOAL")
        # Gets the exact position
        self.leftWallPos = self.leftWallNode.getPosition()
        self.rightWallPos = self.rightWallNode.getPosition()
        # Print the position logging
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

        # stores a reference to the physical robot as an attribute of this class instance
        self.puck = self.supervisor.getFromDef("PUCK")
        # similarly to the ball, we get a reference to the position of the robot so we can change it later
        self.puck_translation_field = self.puck.getField("translation")

    def createBall(self):
        """Creates a new ball when the ball goes out of bounds."""
        self.supervisor = Supervisor()

        ball_string = """DEF BALL Ball {translation 0.21 -0.270544 0.05, rotation 1 0 0 1.5707963267948966, radius 0.05
        }"""
        rootNode = self.supervisor.getRoot()

        rootNode.getField("children").importMFNodeFromString(-1, ball_string)

        # Get the ball node.
        self.ballNode = self.supervisor.getFromDef("BALL")

        self.__node.get_logger().info("Ball has been created.")

    def publishBallPosition(self):
        """Publishing ball position."""
        # Gets the x + y positions.
        position = self.ballNode.getPosition()
        # self.__node.get_logger().info(f"Ball is at position: {position}")
        self.ballPos = Float32MultiArray()
        self.ballPos.data = position

        self.ballPublisher.publish(msg=self.ballPos)

    def dist(self) -> float:
        """Get the distance between the ball and the robot. SF is a single field."""
        puckPos = self.puck_translation_field.getSFVec3f()
        ballPos = self.ball_translation_field.getSFVec3f()
        # Distance away from the robot's position and the ball's position.
        robotAwayBall = math.sqrt(
            (ballPos[0] - puckPos[0]) ** 2 + (ballPos[1] - puckPos[1]) ** 2
        )
        # self.__node.get_logger().info(f"Distance between puck and ball is: {robotAwayBall} ")
        return robotAwayBall

    def movetoPoint(self, xPoint: float, yPoint: float):
        """Moves the robot to a new point."""
        
        # Calculate the angle at which you need to travel, using atan2.
        puckX, puckY = self.puck_translation_field.getSFVec3f()[0:2]

        thetaZ = self.puck.getField("rotation").getSFRotation()[3]

        command_message = Twist()

        # distance = math.sqrt((puckX - xPoint) ** 2 + (puckY - yPoint) ** 2)

        deltaX = abs(puckX - xPoint)

        deltaY = abs(puckY - yPoint)

        grad = deltaY / deltaX

        pointAngle = math.atan2(deltaY, deltaX)        

        # self.__node.get_logger().info(f"Angle between current + point: {pointAngle} rad or {(180 / math.pi) * pointAngle} deg")

        # self.__node.get_logger().info(f"Current angle: {thetaZ} rad / {(180 / math.pi) * thetaZ} deg ")

        # Calculates the difference in angle

        if pointAngle - thetaZ < 0:
            command_message.angular.z = math.pi * 0.5
        else:
            command_message.angular.z = - math.pi * 0.5

        # Move to the point
        # Check if the goal point is less than the puck's
        if xPoint > puckX:
            command_message.linear.x = 0.1
        else:
            command_message.linear.x = -0.1
        # Check if the goal point is less than the puck's
        if yPoint > puckY: 
            command_message.linear.y = 0.1
        else:
            command_message.linear.y = -0.1

        self.movementPublisher.publish(command_message)

    def isColliding(self):
        """Checks for a collision with the ball."""
        if self.dist() < 0.05 * 2:
            self.__node.get_logger().info("Collision occurred!")
            return True
        return False

    def __cmd_vel_callback(self, twist):
        """This is a callback. This sets the twist output -> topic output twist to the twist."""
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.publishBallPosition()

        # Getting the distance between the ball and middle of goal.

        if not self.isColliding():
            # Move to the point
            self.movetoPoint(xPoint=self.ballPos.data[0], yPoint=self.ballPos.data[1])



        self.distanceBetweenGoalAndBall = math.sqrt(
            (self.ballPos.data[0] - self.MiddleOfGoal[0]) ** 2
            + (self.ballPos.data[1] - self.MiddleOfGoal[1]) ** 2
        )

        self.deltaY = abs(self.ballPos.data[1] - self.MiddleOfGoal[1])

        self.deltaX = abs(self.ballPos.data[0] - self.MiddleOfGoal[0])
        # gradient between the goal and the ball.
        self.GABgrad = self.deltaY / self.deltaX

        # angle between the goal and the ball.
        self.GABangle = math.atan2(self.deltaY, self.deltaX)

        # self.__node.get_logger().info(
        #     f" Angle between goal and ball: {self.GABangle}, \n Gradient between goal and ball: {self.GABgrad}"
        # )

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

        # this handles goal logic, detecting a goal has happened and randomizing the positions of the ball and the robot
        # currently does not clear their velocity but this is a trivial modification
        if self.ballPos.data[0] > 1.75:
            new_value = [random.uniform(-1.35, 1.35), random.uniform(-0.7, 0.7), 0.1]
            self.ball_translation_field.setSFVec3f(new_value)
            self.ballNode.addForce([0, 0, 0], relative=False)
            self.ballNode.resetPhysics()

            newer_value = [
                random.uniform(-1.35, 1.35),
                random.uniform(-0.7, 0.7),
                0.001,
            ]
            self.puck_translation_field.setSFVec3f(newer_value)
            self.puck.resetPhysics()
            self.goals += 1
