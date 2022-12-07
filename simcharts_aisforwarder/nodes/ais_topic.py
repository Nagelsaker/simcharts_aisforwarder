import rclpy
from rclpy.node import Node
import random
from simcharts_aisforwarder.api_readers import BarentsWatchReader
from simcharts_aisforwarder.utils import Config
from simcharts_aisforwarder.utils import dcp # dcp = directory_config_paths
from simcharts_interfaces.msg import ListOfAIS

class AISpublisher(Node):
    '''
    Publishes the latest AIS messages from Kystverkets BarentsWatch API
    '''
    def __init__(self, callback_group=None):
        super().__init__('simcharts__ais_forwarder')
        settings = Config(dcp.config).settings
        self.publisher_ = self.create_publisher(
            ListOfAIS,
            'simcharts_aisforwarder/latest_ais',
            10,
            callback_group=callback_group
            )
        timer_period = settings['api']['norway_barents_watch']['T']  # seconds
        self.timer = self.create_timer(timer_period, self.timerCallback)

        self.bwReader = BarentsWatchReader()

    def timerCallback(self):
        self.get_logger().debug('AIS Forwarder timer callback')
        latest_ais_msgs = self.bwReader.getLatestAISMsgs()
        self.get_logger().debug(f"Random AIS message: {latest_ais_msgs.ais_msgs[random.randint(0, len(latest_ais_msgs.ais_msgs)-1)]}")
        self.publisher_.publish(latest_ais_msgs)

class AISsubscriber(Node):
    '''
    Dummy subscribes to the topic 'latest_ais' and prints the received message
    Useful to test installation of the package
    '''
    def __init__(self, callback_group=None):
        super().__init__('ais_subscriber')
        self.subscription = self.create_subscription(
            ListOfAIS,
            'latest_ais',
            self.listener_callback,
            10,
            callback_group=callback_group
            )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)