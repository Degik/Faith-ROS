import configparser

# Class to read the config.ini file
class Collector:
    def __init__(self):
        self.config = configparser.ConfigParser()
        self.config.read('config.ini')

    def get_cameras_topic(self):
        return self.get_all_config('cameras_topic')

    def get_cameras_ids(self):
        return self.get_all_config('cameras_ids')
    
    def get_model(self):
        return self.get_config('model')