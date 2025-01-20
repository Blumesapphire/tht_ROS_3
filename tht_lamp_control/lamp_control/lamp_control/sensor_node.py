import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class LightSensorNode(Node):  # Node untuk mensimulasikan sensor cahaya
  def __init__(self):
    super().__init__('sensor_node')  # Inisialisasi Node dengan nama
    self.publisher = self.create_publisher(Float32, '/light_intensity', 10)  # Membuat Publisher untuk Topic '/light_intensity'
    self.timer = self.create_timer(1.0, self.publish_light_intensity)  # Timer untuk menjalankan fungsi setiap 1 detik
    self.start_time = time.time()  # Simpan waktu mulai

  def publish_light_intensity(self):
    elapsed_time = time.time() - self.start_time
    if elapsed_time > 10:  # Batasi waktu eksekusi 10 detik
      self.get_logger().info("Stopping the Node after 10 seconds.")
      rclpy.shutdown()  # Hentikan Node
      return
    
    import random
    light_intensity = random.uniform(0.0, 100.0)  # Simulasi nilai intensitas cahaya
    msg = Float32()
    msg.data = light_intensity  # Mengisi pesan dengan nilai cahaya
    self.publisher.publish(msg)  # Menerbitkan pesan ke Topic '/light_intensity'
    self.get_logger().info(f'Publishing Light Intensity: {light_intensity}')  # Logging untuk debugging

def main(args=None):
  rclpy.init(args=args)  # Inisialisasi ROS 2
  node = LightSensorNode()  # Membuat Node
  rclpy.spin(node)  # Menjalankan Node
  rclpy.shutdown()  # Mematikan ROS 2 setelah selesai

if __name__ == '__main__':
  main()  # Menjalankan program
