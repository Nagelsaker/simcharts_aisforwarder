import os
import csv
from utils import paths as dcp # dcp = directory_config_paths


def log_unparsed_parameter(key: str, value: str):
    '''
    Logs an unparsed AIS parameter
    '''
    pass
    # TODO: Implement this function
    # header = ['key', 'last_value', 'nr_of_times']
    # # Check if file exists
    # if not os.path.isfile(dcp.log_unparsed_parameters):
    #     with open(dcp.log_unparsed_parameters, 'w', encoding='UTF8') as f:
    #         csv_writer = csv.DictWriter(f, header)
    #         csv_writer.writeheader()

    # with open(dcp.log_unparsed_parameters, 'r') as f:
    #     csv_reader = csv.DictReader(f)
    #     # csv_writer = csv.DictWriter(f, header)
    #     for row in csv_reader:
    #         if row == []:
    #             csv_writer.writerow([key, value, 1])
    #             continue
    #         elif row[0] == key:
    #             row[1] = value
    #             row[2] = int(row[2]) + 1
    #             csv_writer.writerows(row)
    # print("heh")