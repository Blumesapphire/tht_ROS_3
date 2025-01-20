import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class LampControlService(Node):
  def __init__(self):
    super().__init__('lamp_service')
    self.get_logger().info('LampControlService Node has been started.')
    self.service = self.create_service(
      SetBool, 
      '/toggle_lamp', 
      self.handle_lamp_control
    )
    self.get_logger().info('Service /toggle_lamp is up and running.')

  def handle_lamp_control(self, request, response):
    if request.data:
      response.message = "Lamp is ON"
      self.get_logger().info("Lamp turned ON")
    else:
      response.message = "Lamp is OFF"
      self.get_logger().info("Lamp turned OFF")
    
    response.success = True
    return response

def main(args=None):
  rclpy.init(args=args)
  node = LampControlService()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
