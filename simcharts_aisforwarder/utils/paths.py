'''
Defines hard-coded paths to relevant files in the simcharts_aisforwarder package
'''

import pathlib

root = pathlib.Path(__file__).parents[2]
package = root / 'simcharts_aisforwarder'

config = root / 'config.yaml'
config_schema = root / 'config_schema.yaml'

# Credentials
credentials_norway_barents_watch = root / 'api_credentials' / 'norway_barents_watch' / 'credentials.yaml'

# Logs
log_unparsed_parameters = root / 'logs' / 'unparsed_ais_params.csv'