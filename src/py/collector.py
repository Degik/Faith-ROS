import configparser as configparser

# Class to read the config.ini file
class Collector:
    def __init__(self):
        self.config = configparser.ConfigParser()
        self.config.read('config.ini')

    def get_cameras_topic(self) -> list:
        return [item[1] for item in self.config.items('cameras_topic')]

    def get_cameras_ids(self) -> list:
        # Return the cameras ids with a integer list
        return [item[1] for item in self.config.items('cameras_ids')]
    def get_model(self) -> str:
        return self.config.get('model_name', 'model')