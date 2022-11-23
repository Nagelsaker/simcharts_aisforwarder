import rclpy
from rclpy.node import Node
from simcharts_aisforwarder.utils import Config
from simcharts_aisforwarder.utils import dcp # dcp = directory_config_paths
from simcharts_interfaces.msg import ListOfAIS

class AISsubscriber(Node):
    '''
    Dummy subscribes to the topic 'latest_ais' and prints the received message
    Useful to test installation of the package
    '''
    def __init__(self):
        super().__init__('ais_subscriber')
        settings = Config(dcp.config).settings
        self.subscription = self.create_subscription(
            ListOfAIS,
            'latest_ais',
            self.listener_callback,
            10  )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
    
def main(args=None):
    rclpy.init(args=args)
    ais_subscriber = AISsubscriber()
    rclpy.spin(ais_subscriber)
    ais_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()