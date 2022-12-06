import rclpy
from nodes import AISsubscriber
    
def main(args=None):
    rclpy.init(args=args)
    ais_subscriber = AISsubscriber()
    rclpy.spin(ais_subscriber)
    ais_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()