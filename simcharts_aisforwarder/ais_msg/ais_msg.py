
class AISmsg:
    '''
    AIS message class
    Stores information from different types of AIS messages in a general data class

    The following AIS parameters are a minimum requirement for the AIS message to be valid:
    - mmsi : int (Unique ID of the vessel)
    - timestamp : str (Timestamp of the AIS message)
    - longitude : float (Longitude of the vessel)
    - latitude : float (Latitude of the vessel)

    The following AIS parameters are optional:
    - SOG : float (Speed over ground)
    - COG : float (Course over ground)
    - heading : float (Heading angle)
    - ROT : float (Rate of turn)
    - name : str (Vessel name)
    - shipType : int (Vessel type)
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
    ship_type = None


    def __init__(self, mmsi=None, timestamp=None, longitude=None, latitude=None, SOG=None, COG=None, heading=None, ROT=None, name=None, ship_type=None):
        self.mmsi = mmsi
        self.timestamp = timestamp
        self.longitude = longitude
        self.latitude = latitude
        self.SOG = SOG
        self.COG = COG
        self.heading = heading
        self.ROT = ROT
        self.name = name
        self.ship_type = ship_type
    
    def isValid(self):
        '''
        Checks if the AIS message is valid
        '''
        if self.mmsi is None or self.timestamp is None or self.longitude is None or self.latitude is None:
            return False
        else:
            return True