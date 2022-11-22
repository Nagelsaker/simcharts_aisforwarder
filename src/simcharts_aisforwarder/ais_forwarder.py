import rclpy
import rclpy.node
from api_readers import BarentsWatchReader
from utils import Config
# from simcharts_interfaces.msg import ListOfAIS

class AISForwarder(rclpy.node.Node):
    def __init__(self):
        super().__init__('ais_forwarder')
        self.declare_parameter('ais_forwarder', 'simcharts_interfaces/ListOfAIS')
        self.get_logger().info('AIS Forwarder started')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bwReader = BarentsWatchReader()

    def timerCallback(self):
        self.get_logger().info('AIS Forwarder timer callback')

        latest_ais_msgs = self.bwReader.getLatestAISMsgs()
        




def main():
    # Load settings
    config = Config()
    bwReader = BarentsWatchReader()
    ais_msgs = bwReader.getLatestAISMsgs()
    print("wow")

if __name__ == "__main__":
    main()