import rclpy
from geometry_msgs.msg import Twist
from controller import Supervisor

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
        # create the ball using the ROS2 Supervisor
        self.createBall()

    def createBall(self):
        """Creates a new ball when the ball goes out of bounds."""
        self.supervisor = Supervisor()

        ball_string = """DEF BALL Ball {translation 0.21 -0.270544 0.05, rotation 1 0 0 1.5707963267948966, radius 0.05
        }"""
        rootNode = self.supervisor.getRoot()
        
        rootNode.getField("children").importMFNodeFromString(-1, ball_string)

        self.__node.get_logger().info("Ball has been created.")
        # Get the ball node.
        ballNode = self.supervisor.getFromDef("BALL")
        # Gets the x + y positions.
        self.__node.get_logger().info(f"Ball is at position: {ballNode.getPosition()[0:2]}")

    def __cmd_vel_callback(self, twist):
        """This is a callback. This sets the twist output -> topic output twist to the twist."""
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

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
