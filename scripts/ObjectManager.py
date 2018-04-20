import numpy as np

from JsonManager import JsonManager


class ObjectManager(JsonManager):

    def __init__(self, file_path):
        # Converting relative file path to absolute file path
        JsonManager.__init__(self, file_path)

    def save_object(self, name, object_contour, object_angle, object_center):
        # Create JSON object
        object_to_save = {
            "name": name,
            "contour": object_contour.tolist(),
            "angle": object_angle,
            "center": object_center
        }

        # Load existent objects
        object_list = self.load_json()
        # Append object
        object_list.append(object_to_save)

        self.save_json(object_list)

    def load_objects(self):
        # Load existent objects
        object_list = self.load_json()

        for obj in object_list:
            obj["contour"] = np.array(obj["contour"])

        return object_list
