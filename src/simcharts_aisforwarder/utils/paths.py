'''
Defines hard-coded paths to relevant files in the AIS_Forwarder package
'''

import pathlib

root = pathlib.Path(__file__).parents[2]
package = root / 'AIS_Forwarder'

config = root / 'config.yaml'
config_schema = root / 'config_schema.yaml'

# Credentials
credentials_norway_barents_watch = root / 'api_credentials' / 'norway_barents_watch' / 'credentials.yaml'

# Logs
log_unparsed_parameters = root / 'logs' / 'unparsed_ais_params.csv'