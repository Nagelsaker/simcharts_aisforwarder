import rclpy
import rclpy.node
from simcharts_aisforwarder.nodes import AISpublisher

def main():
    rclpy.init(args=None)
    ais_forwarder = AISpublisher()
    rclpy.spin(ais_forwarder)
    ais_forwarder.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()