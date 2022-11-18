from authlib.integrations.httpx_client import AsyncOAuth2Client
from authlib.oauth2.rfc7523 import ClientSecretJWT
import requests
import ast


def main():
    print("main")
    client_id = 'simon.lexau@ntnu.no:dev_ais_forwarder'
    client_secret = '4iYnr5HZkFtHk!S'
    scope = 'ais'
    access_url = 'https://id.barentswatch.no/connect/token'
    ais_url = 'https://live.ais.barentswatch.no/v1/latest/combined'
    
    # Get an access token
    ret = requests.post(
        access_url,
        headers={'Content-Type' : 'application/x-www-form-urlencoded'},
        data={'client_id' : client_id, 'client_secret' : client_secret,
            'scope' : scope, 'grant_type' : 'client_credentials'})
    token = ast.literal_eval(ret.content.decode('utf-8'))['access_token']

    # Get the latest position of all vessels
    ret = requests.get(
        ais_url,
        headers={'Authorization' : f'Bearer {token}'}
    )

    # This code reformats the returned information into a list of AIS messages containing a list of key:data
    i, j = 0, 0
    ret.text[1:-1].replace('{', '').replace('"', '').split('}')[i].split(',')[j]
    ais_msgs = reformatRawAISData(ret.text)
    print("wow")

def reformatRawAISData(ais_string):
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
                # print("val is float or null")
                key = ais_param_list[1]
                val = ais_param_list[2][1:]
                if val != 'null':
                    val = float(val) 
            elif len(ais_param_list) == 5:
                # print("val is string")
                key = ais_param_list[1]
                val = ais_param_list[3]
            elif len(ais_param.split('"')) == 1:
                # Empty, so we do not care
                continue
            else:
                print("Error: Unknown format! Exiting...")
                exit()
            ais_msg[key] = val
        ais_msgs.append(ais_msg)
    return ais_msgs

if __name__ == "__main__":
    main()