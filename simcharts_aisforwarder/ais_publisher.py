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

        # self.declare_parameter('ais_forwarder', 'simcharts_interfaces/ListOfAIS')
        # self.get_logger().info('AIS Forwarder started')
        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.bwReader = BarentsWatchReader()

    def timerCallback(self):
        self.get_logger().info('AIS Forwarder timer callback')
        # msg = ListOfAIS()
        latest_ais_msgs = self.bwReader.getLatestAISMsgs()
        # msg.timestamp = latest_ais_msgs.timestamp
        # msg.ais_msgs = latest_ais_msgs.ais_mgs

        self.publisher_.publish(latest_ais_msgs)
        self.get_logger().info(f'Publishing: {latest_ais_msgs.ais_msgs[0]} ...' )





def main():
    rclpy.init(args=None)
    ais_forwarder = AISpublisher()
    rclpy.spin(ais_forwarder)
    ais_forwarder.destroy_node()
    rclpy.shutdown()

    # Load settings
    # config = Config()
    # bwReader = BarentsWatchReader()
    # ais_msgs = bwReader.getLatestAISMsgs()
    # print("wow")

if __name__ == "__main__":
    main()