from simcharts_aisforwarder.ais_msg import AISmsg
from simcharts_aisforwarder.utils import log_unparsed_parameter
from simcharts_interfaces.msg import ListOfAIS
from typing import List
import datetime

class AISmsgParser:
    '''
    AIS message parser class
    Parses raw AIS message strings into the general AISmsg data class
    It can be used to parse a single AIS message string, or a list of AIS message strings

    The following AIS parameters are a minimum requirement for the AIS message to be considered valid:
    - MMSI (Unique ID of the vessel)
    - timestamp (Timestamp of the AIS message)
    - longitude (Longitude of the vessel)
    - latitude (Latitude of the vessel)

    '''

    def __init__(self, ais_strings: List[str]):
        self.ais_strings = ais_strings
        self.ais_msgs = []
    
    def parse(self) -> ListOfAIS:
        '''
        Parses a list of AIS message strings into the general AISmsg data class
        '''
        for msg in self.ais_strings:
            ais_msg = self._parse_single_msg(msg)
            if ais_msg is None: continue
            self.ais_msgs.append(ais_msg)
            listOfAIS = ListOfAIS()
            listOfAIS.ais_msgs = self.ais_msgs
            timestamp = datetime.datetime.now().timestamp()
            listOfAIS.timestamp = timestamp
        return listOfAIS

    def _parse_single_msg(self, msg: dict) -> AISmsg:
        '''
        Parses a single AIS message string into the general AISmsg data class
        '''
        # Get the keys and values from the AIS message string
        ais_msg = AISmsg()
        for key in msg.keys():
            try:
                if key == 'mmsi': ais_msg.mmsi = int(msg[key])
                elif key == 'msgtime': ais_msg.timestamp = msg[key]
                elif key == 'longitude': ais_msg.longitude = float(msg[key])
                elif key == 'latitude': ais_msg.latitude = float(msg[key])
                elif key == 'speedOverGround':ais_msg.sog = str(msg[key]) # float(msg[key])
                elif key == 'courseOverGround': ais_msg.cog = str(msg[key]) # float(msg[key])
                elif key == 'trueHeading': ais_msg.heading = str(msg[key]) # float(msg[key])
                elif key == 'rateOfTurn': ais_msg.rot = str(msg[key]) # float(msg[key])
                elif key == 'name': ais_msg.name = msg[key]
                elif key == 'shipType': ais_msg.shiptype = str(msg[key]) # int(msg[key])
                else: log_unparsed_parameter(key, msg[key])
            except:
                if msg[key] == '' or msg[key] == 'null': continue
                print(f'Error parsing AIS parameter: {key} = {msg[key]}')
        
        if ais_msg.isValid():
            return ais_msg
        else:
            return None
    