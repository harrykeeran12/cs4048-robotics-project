import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class BallSupervisor(Node):
    def __init__(self):
        super().__init__("ballSup")
        self.__subscriber = self.create_subscription(Float32MultiArray, "/ballPos", callback=self.step(), qos_profile=1)

    def step(self, message):
        pass



def main(args=None):
    rclpy.init(args=args)
    ballSupervisor = BallSupervisor()
    rclpy.spin(ballSupervisor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ballSupervisor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
