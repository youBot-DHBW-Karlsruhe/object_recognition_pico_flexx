import json
import os


class JsonManager:

    def __init__(self, file_path):
        self.file_path = os.path.join(os.path.dirname(__file__), file_path)
        pass

    def save_json(self, object_list):
        # Save list of objects
        outfile = open(self.file_path, 'w+')
        json.dump(object_list, outfile)

    def load_json(self):
        directory = os.path.dirname(self.file_path)

        # load list of saved objects
        try:
            json_file = open(self.file_path)
            object_list = json.load(json_file)
        except (EnvironmentError, ValueError):
            if not os.path.exists(directory):
                os.makedirs(directory)
            object_list = []

        return object_list
