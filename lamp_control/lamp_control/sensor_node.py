import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class LightSensorNode(Node):
  def __init__(self):
    super().__init__('sensor_node')
    self.publisher = self.create_publisher(Float32, '/light_intensity', 10)
    self.timer = self.create_timer(1.0, self.publish_light_intensity)
    self.start_time = time.time()

  def publish_light_intensity(self):
    elapsed_time = time.time() - self.start_time
    if elapsed_time > 10:
      self.get_logger().info("Stopping the Node after 10 seconds.")
      rclpy.shutdown()
      return
    
    import random
    light_intensity = random.uniform(0.0, 100.0)
    msg = Float32()
    msg.data = light_intensity
    self.publisher.publish(msg)
    self.get_logger().info(f'Publishing Light Intensity: {light_intensity}')

def main(args=None):
  rclpy.init(args=args)
  node = LightSensorNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
