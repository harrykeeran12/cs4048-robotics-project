import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from rclpy.logging import get_logger

class BallSupervisor(Node):
  def init(self, webots_node, properties):
        super().__init__('ballSup')
        self.__publisher = self.create_publisher(String, '/ballPos', 1)
        self.logger = get_logger("log1")
  def step(self):
      print("Find ball.")
        


def main(args=None):
    rclpy.init(args=args)
    ballSupervisor = BallSupervisor()
    rclpy.spin(ballSupervisor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ballSupervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()