from simcharts_interfaces.msg import AIS

class AISmsg(AIS):
    '''
    Overloaded AIS msg class from the simcharts_interfaces package
    Easier to use and more readable. Also useful for debugging.
    Stores information from different types of AIS messages in a general data class

    The following AIS parameters are a minimum requirement for the AIS message to be valid:
    - mmsi : int (Unique ID of the vessel)
    - timestamp : str (Timestamp of the AIS message)
    - longitude : float (Longitude of the vessel)
    - latitude : float (Latitude of the vessel)

    The following AIS parameters are optional, and are given as strings
    since they are not always available in the AIS message:
    - sog : str (Speed over ground)
    - cog : str (Course over ground)
    - heading : str (Heading angle)
    - rot : str (Rate of turn)
    - name : str (Vessel name)
    - shiptype : str (Vessel type)
    '''

    def __init__(self):
        super().__init__()
    
    def isValid(self):
        '''
        Checks if the AIS message is valid
        '''
        if self.mmsi is None or self.timestamp is None or self.longitude is None or self.latitude is None:
            return False
        else:
            return True