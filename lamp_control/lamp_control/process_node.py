import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from example_interfaces.srv import SetBool

class LightProcessorNode(Node):
  def __init__(self):
    super().__init__('process_node')
    self.subscriber = self.create_subscription(Float32, '/light_intensity', self.process_light_intensity, 10)
    self.client = self.create_client(SetBool, '/toggle_lamp')

    while not self.client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('Waiting for /toggle_lamp service...')

  def process_light_intensity(self, msg):
    intensity = msg.data
    self.get_logger().info(f'Received Light Intensity: {intensity}')
    if intensity < 50.0:
      self.control_lamp(True)
    else:
      self.control_lamp(False)

  def control_lamp(self, turn_on):
    request = SetBool.Request()
    request.data = turn_on
    future = self.client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    self.get_logger().info(f'Response from Lamp: {future.result().message}')

def main(args=None):
  rclpy.init(args=args)
  node = LightProcessorNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
