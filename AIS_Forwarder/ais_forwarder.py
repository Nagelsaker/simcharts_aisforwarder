from api_readers import BarentsWatchReader
from utils import Config

def main():
    # Load settings
    config = Config()
    bwReader = BarentsWatchReader()
    ais_msgs = bwReader.getLatestAISMsgs()
    print("wow")

if __name__ == "__main__":
    main()