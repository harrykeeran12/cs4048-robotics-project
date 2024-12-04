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
        self.ballPublisher = self.__node.create_publisher(
            Float32MultiArray, "/ball_pos", qos_profile=1
        )

        # create the ball and store some of its details as an attribute of this class instance
        # ideally we would have a ball class that stores some of this but idc
        self.createBall()
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
        self.MiddleOfGoal = ((self.leftWallPos[0] + self.rightWallPos[0]) / 2, (self.leftWallPos[1] + self.rightWallPos[1]) / 2)

        self.__node.get_logger().info(
            f"Middle of the goal: {self.MiddleOfGoal}"
        )
        
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

    def __cmd_vel_callback(self, twist):
        """This is a callback. This sets the twist output -> topic output twist to the twist."""
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.publishBallPosition()

        # Getting the distance between the ball and middle of goal.
        
        # Get the gradient first:

        self.distanceBetweenGoalAndBall = math.sqrt((self.ballPos.data[0] + self.MiddleOfGoal[0]) - (self.ballPos.data[1] + self.ballPos.data[1]))

        self.deltaY = abs(self.ballPos.data[1] - self.MiddleOfGoal[1])

        self.deltaX = abs(self.ballPos.data[0] - self.MiddleOfGoal[0])
        # gradient between the goal and the ball.
        self.GABgrad = self.deltaY / self.deltaX 
        
        # angle between the goal and the ball.
        self.GABangle = math.atan2(self.deltaY, self.deltaX)


        self.__node.get_logger().info(
            f" Angle between goal and ball: {self.GABangle}"
        )

        self.__node.get_logger().info(
            f" Gradient between goal and ball: {self.GABgrad}"
        )

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
        if self.ballNode.getPosition()[0] > 1.75:
            new_value = [random.uniform(-1.35, 1.35), random.uniform(-0.7, 0.7), 0.1]
            self.ball_translation_field.setSFVec3f(new_value)

            newer_value = [
                random.uniform(-1.35, 1.35),
                random.uniform(-0.7, 0.7),
                0.001,
            ]
            self.puck_translation_field.setSFVec3f(newer_value)
            self.goals += 1
