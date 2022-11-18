from pathlib import Path
from typing import List
import yaml

from cerberus import Validator
from . import paths as dcp # dcp = directory_config_paths

class Config:
    '''
    This class is used to load the configuration file
    '''
    def __init__(self, config_file_name: Path = dcp.config, **kwargs):
        self._schema = read_yaml_into_dict(dcp.config_schema)
        self.validator = Validator(self._schema)
        self._valid_sections = self.extract_valid_sections()

        self.parse(config_file_name)
    
    @property
    def settings(self):
        return self._settings
    
    @settings.setter
    def settings(self, new_settings: dict) -> None:
        self._settings = new_settings

    def extract_valid_sections(self) -> List[str]:
        if self._schema is None:
            raise ValueError("No configuration schema provided!")

        sections = []
        for section in self._schema.keys():
            sections.append(section)
        return sections
    
    def validate(self, settings: dict) -> None:
        if not settings:
            raise ValueError("Empty settings!")

        if not self._schema:
            raise ValueError("Empty schema!")

        if not self.validator.validate(settings):
            raise ValueError(f"Cerberus validation Error: {self.validator.errors}")
    
    def parse(self, file_name: Path) -> None:
        self._settings = read_yaml_into_dict(file_name)
        self.validate(self._settings)


def read_yaml_into_dict(file_name: Path) -> dict:
    '''
    Reads a yaml file into a dictionary
    '''
    with open(file_name, 'r') as f:
        output_dict = yaml.load(f, Loader=yaml.SafeLoader)
    return output_dict