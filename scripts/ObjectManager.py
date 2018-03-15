import json
import os


class ObjectManager:

    def __init__(self, file_path):
        # Converting relative file path to absolute file path
        self.file_path = os.path.join(os.path.dirname(__file__), file_path)

    def save(self, name, object_contour, object_angle, object_center):
        # Create JSON object
        object_to_save = {
            "name": name,
            "contour": object_contour.tolist(),
            "angle": object_angle,
            "center": object_center
        }

        # Load existent objects
        object_list = self.load_objects()
        # Append object
        object_list.append(object_to_save)

        # Save list of objects
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
