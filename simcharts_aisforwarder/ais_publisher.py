import rclpy
import rclpy.node
from . import BarentsWatchReader
from . import Config
from . import dcp # dcp = directory_config_paths
from simcharts_interfaces.msg import ListOfAIS

class AISpublisher(rclpy.node.Node):
    '''
    Publishes the latest AIS messages from Kystverkets BarentsWatch API
    '''
    def __init__(self):
        super().__init__('ais_forwarder')
        settings = Config(dcp.config).settings
        self.publisher_ = self.create_publisher(ListOfAIS, 'latest_ais', 10)
        timer_period = settings['api']['norway_barents_watch']['T']  # seconds
        self.timer = self.create_timer(timer_period, self.timerCallback)

        self.bwReader = BarentsWatchReader()

    def timerCallback(self):
        self.get_logger().debug('AIS Forwarder timer callback')
        latest_ais_msgs = self.bwReader.getLatestAISMsgs()
        self.publisher_.publish(latest_ais_msgs)


def main():
    rclpy.init(args=None)
    ais_forwarder = AISpublisher()
    rclpy.spin(ais_forwarder)
    ais_forwarder.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()