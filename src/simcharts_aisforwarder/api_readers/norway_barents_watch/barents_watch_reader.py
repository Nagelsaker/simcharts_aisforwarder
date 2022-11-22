from utils import read_yaml_into_dict, dcp
from ais_msg_parser import AISmsgParser
from ais_msg import AISmsg
from typing import List
import requests
import ast

class BarentsWatchReader:
    '''
    BarentsWatchReader class
    Obtains access token to the BarentsWatch API, and then uses the access token to obtain the latest AIS messages

    '''
    token = None

    def __init__(self):
        self.loadCredentials()

    def loadCredentials(self) -> None:
        self.credentials = read_yaml_into_dict(dcp.credentials_norway_barents_watch)['credentials']
        self.client_id = self.credentials['client_id']
        self.client_secret = self.credentials['client_secret']
        self.scope = self.credentials['scope']
        self.access_url = self.credentials['access_url']
        self.ais_url = self.credentials['ais_url']

    def refreshAccessToken(self) -> str:
        ret = requests.post(
            self.access_url,
            headers={'Content-Type' : 'application/x-www-form-urlencoded'},
            data={'client_id' : self.client_id, 'client_secret' : self.client_secret,
                'scope' : self.scope, 'grant_type' : 'client_credentials'})
        self.token = ast.literal_eval(ret.content.decode('utf-8'))['access_token']

    def getLatestAISMsgs(self) -> List[AISmsg]:
        self.refreshAccessToken()
        ret = requests.get(
            self.ais_url,
            headers={'Authorization' : 'Bearer ' + self.token})
        list_of_ais_strings = self._reformatRawAISData(ret.text)
        parser = AISmsgParser(list_of_ais_strings)
        ais_msgs = parser.parse()
        return ais_msgs

        

    def _reformatRawAISData(self, ais_string: str) -> List[dict]:
        # Remove redundant symbols
        ais_string = ais_string[1:-1].replace('{', '')
        ais_list_of_strings = ais_string.split('}')[:-1]
        ais_msgs = []
        for ais_string in ais_list_of_strings:
            ais_msg_string = ais_string.split(',')
            ais_msg = {}
            for ais_param in ais_msg_string:
                key = None
                val = None
                ais_param_list = ais_param.split('"')
                if len(ais_param_list) == 3:
                    # val is float or null
                    key = ais_param_list[1]
                    val = ais_param_list[2][1:]
                    if val != 'null':
                        val = float(val) 
                elif len(ais_param_list) == 5:
                    # val is string
                    key = ais_param_list[1]
                    val = ais_param_list[3]
                elif len(ais_param.split('"')) == 1:
                    # Empty, so we do not care
                    continue
                else:
                    print("Error: Unknown format! Skipping parameter...")
                    continue
                ais_msg[key] = val
            ais_msgs.append(ais_msg)
        return ais_msgs