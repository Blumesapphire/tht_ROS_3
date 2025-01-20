import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Tipe data untuk Topic
from example_interfaces.srv import SetBool  # Tipe data untuk Service (boolean)

class LightProcessorNode(Node):  # Node untuk memproses data cahaya
  def __init__(self):
    super().__init__('process_node')  # Inisialisasi Node
    self.subscriber = self.create_subscription(Float32, '/light_intensity', self.process_light_intensity, 10)  # Membuat Subscriber untuk Topic '/light_intensity'
    self.client = self.create_client(SetBool, '/toggle_lamp')  # Membuat Client untuk Service '/toggle_lamp'

    while not self.client.wait_for_service(timeout_sec=1.0):  # Menunggu Service tersedia
      self.get_logger().info('Waiting for /toggle_lamp service...')

  def process_light_intensity(self, msg):
    intensity = msg.data  # Membaca nilai intensitas cahaya
    self.get_logger().info(f'Received Light Intensity: {intensity}')

    # Jika intensitas cahaya di bawah 50.0, hidupkan lampu
    if intensity < 50.0:
      self.control_lamp(True)
    else:
      self.control_lamp(False)

  def control_lamp(self, turn_on):
    request = SetBool.Request()
    request.data = turn_on  # True untuk hidupkan, False untuk matikan lampu
    future = self.client.call_async(request)  # Mengirim permintaan ke Service
    rclpy.spin_until_future_complete(self, future)  # Menunggu respons
    self.get_logger().info(f'Response from Lamp: {future.result().message}')  # Logging respons

def main(args=None):
  rclpy.init(args=args)
  node = LightProcessorNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
