import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci  # Contoh action bawaan ROS

class LampAdjustAction(Node):
  def __init__(self):
    super().__init__('lamp_action')
    self._action_server = ActionServer(
      self,
      Fibonacci,
      'adjust_light',
      self.execute_callback
    )

  def execute_callback(self, goal_handle):
    self.get_logger().info('Received request to adjust light intensity')
    feedback_msg = Fibonacci.Feedback()
    result_msg = Fibonacci.Result()

    for i in range(1, goal_handle.request.order + 1):
      feedback_msg.partial_sequence.append(i)
      self.get_logger().info(f'Adjusting Light Intensity to {i * 10}%')
      goal_handle.publish_feedback(feedback_msg)

    result_msg.sequence = feedback_msg.partial_sequence
    goal_handle.succeed()
    return result_msg

def main(args=None):
  rclpy.init(args=args)
  node = LampAdjustAction()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
