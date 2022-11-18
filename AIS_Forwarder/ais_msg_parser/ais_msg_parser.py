from ais_msg import AIS_msg

class AIS_msg_parser(AIS_msg):
    '''
    AIS message parser class
    Parses raw AIS message strings into the general AIS_msg data class
    It can be used to parse a single AIS message string, or a list of AIS message strings

    The following AIS parameters are a minimum requirement for the AIS message to be valid:
    - MMSI (Unique ID of the vessel)
    - timestamp (Timestamp of the AIS message)
    - longitude (Longitude of the vessel)
    - latitude (Latitude of the vessel)

    '''

    def __init__(self, mmsi, timestamp, longitude, latitude, SOG=None, COG=None, heading=None, ROT=None, name=None, shipType=None):
        super().__init__(mmsi, timestamp, longitude, latitude, SOG, COG, heading, ROT, name, shipType)