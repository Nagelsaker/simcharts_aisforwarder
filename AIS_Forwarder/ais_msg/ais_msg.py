
class AIS_msg:
    '''
    AIS message class
    Stores information from different types of AIS messages in a general data class

    The following AIS parameters are a minimum requirement for the AIS message to be valid:
    - MMSI (Unique ID of the vessel)
    - timestamp (Timestamp of the AIS message)
    - longitude (Longitude of the vessel)
    - latitude (Latitude of the vessel)

    The following AIS parameters are optional:
    - SOG : Speed over ground
    - COG : Course over ground
    - heading : Heading angle
    - ROT : Rate of turn
    - name : Vessel name
    - shipType : Vessel type
    '''

    # Required parameters
    mmsi = None
    timestamp = None
    longitude = None
    latitude = None

    # Optional parameters
    SOG = None
    COG = None
    heading = None
    ROT = None
    name = None
    shipType = None


    def __init__(self, mmsi, timestamp, longitude, latitude, SOG=None, COG=None, heading=None, ROT=None, name=None, shipType=None):
        self.mmsi = mmsi
        self.timestamp = timestamp
        self.longitude = longitude
        self.latitude = latitude
        self.SOG = SOG
        self.COG = COG
        self.heading = heading
        self.ROT = ROT
        self.name = name
        self.shipType = shipType