'''
Defines hard-coded paths to relevant files in the simcharts_aisforwarder package
'''

import pathlib

root = pathlib.Path(__file__).parents[2] / '../../../'
package = root / 'src/simcharts_aisforwarder/'

config = package / 'config.yaml'
config_schema = package / 'config_schema.yaml'

# Credentials
credentials_norway_barents_watch = package / 'api_credentials' / 'norway_barents_watch' / 'credentials.yaml'
credentials_info = package / 'api_credentials' / 'norway_barents_watch' / 'info.md'

# Logs
log_unparsed_parameters = package / 'logs' / 'unparsed_ais_params.csv'