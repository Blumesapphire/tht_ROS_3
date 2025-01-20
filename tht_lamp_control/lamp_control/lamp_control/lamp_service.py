import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # Tipe data untuk Service

class LampControlService(Node):  # Node untuk menyediakan kontrol lampu
  def __init__(self):
    super().__init__('lamp_service')  # Inisialisasi Node dengan nama 'lamp_service'
    
    # Log saat Node dimulai
    self.get_logger().info('LampControlService Node has been started.')

    # Membuat Service '/toggle_lamp'
    self.service = self.create_service(
      SetBool, 
      '/toggle_lamp', 
      self.handle_lamp_control
    )
    
    # Log tambahan setelah Service dibuat
    self.get_logger().info('Service /toggle_lamp is up and running.')

  def handle_lamp_control(self, request, response):
    # Menghidupkan atau mematikan lampu berdasarkan permintaan
    if request.data:
      response.message = "Lamp is ON"
      self.get_logger().info("Lamp turned ON")
    else:
      response.message = "Lamp is OFF"
      self.get_logger().info("Lamp turned OFF")
    
    response.success = True  # Menyatakan permintaan berhasil
    return response

def main(args=None):
  # Inisialisasi ROS 2
  rclpy.init(args=args)
  
  # Membuat Node
  node = LampControlService()
  
  # Menjalankan Node
  rclpy.spin(node)
  
  # Mematikan Node setelah selesai
  rclpy.shutdown()

if __name__ == '__main__':
  main()
