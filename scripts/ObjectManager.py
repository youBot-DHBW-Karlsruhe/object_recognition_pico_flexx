import json
import os


class ObjectManager:

    def __init__(self, file_path):
        self.file_path = file_path

    def save(self, object_to_save):
        object_list = self.load_objects()

        # append object
        object_list.append(object_to_save)

        # save list of objects
        outfile = open(self.file_path, 'w+')
        json.dump(object_list, outfile)

    def load_objects(self):
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
