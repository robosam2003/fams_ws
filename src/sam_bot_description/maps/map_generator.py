import os
import yaml
import cv2
import numpy as np

# Create a map generator for ROS2
# INPUTS: rectangles with their size and placement in absolute (metre) coordinates
# OUTPUTS: a map image (.pgm file) and a map yaml file with the map metadata


class Rectangle:
    def __init__(self, x, y, w, h, hollow=False, line_thickness=1):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.hollow = hollow
        self.line_thickness = line_thickness

class MapGenerator:
    def __init__(self, map_path, map_name, map_size_m, resolution, origin):
        self.map_path = map_path
        self.map_name = map_name
        self.map_size = int(map_size_m[0] / resolution), int(map_size_m[1] / resolution)
        self.resolution = resolution
        self.origin = origin
        self.map = None

    def create_map(self, rectangles):
        # Create a blank image
        self.map = np.zeros((self.map_size[1], self.map_size[0], 3), np.uint8)
        self.map.fill(255) # Fill with white

        # Draw rectangles on the map
        for rect in rectangles:
            x = int(rect.x / self.resolution)
            y = int(rect.y / self.resolution)
            w = int(rect.w / self.resolution)
            h = int(rect.h / self.resolution)
            if rect.hollow:
            # Draw a hollow rectangle
                cv2.rectangle(self.map, (x, y), (x+w, y+h), (0, 0, 0), rect.line_thickness)
            else:
            # Draw a filled rectangle
                cv2.rectangle(self.map, (x, y), (x+w, y+h), (0, 0, 0), -1)

        # Save the map image
        map_image_path = os.path.join(self.map_path, self.map_name + '.pgm')
        gray = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(map_image_path, gray)

        # Save the map yaml file
        map_yaml_path = os.path.join(self.map_path, self.map_name + '.yaml')
        map_yaml = {
            'image': self.map_name + '.pgm',
            'mode': 'trinary',

            'resolution': self.resolution,
            'origin': self.origin,
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }
        with open(map_yaml_path, 'w') as file:
            yaml.dump(map_yaml, file, sort_keys=False, default_flow_style=None)

        return map_image_path, map_yaml_path
    
# Example usage
map_path = '/home/robosam/fams_ws/src/sam_bot_description/maps'
map_name = 'test_map'
resolution = 0.05 # in metres per pixel
origin = [0.0, 0.0, 0.0] # Origin of the map in absolute coordinates
map_size = (3.65, 2.06) # Size of the map in metres (width, height)
rectangles = [
    Rectangle(0, 0, 3.6, 2.00, hollow=True), # Hollow rectangle, (x, y, w, h)
    Rectangle(1.9+0.025, 2.06-(0.42 + 0.13), 0.2, 0.42), # Filled rectangle, (x, y, w, h)
    # Rectangle(3.6/2-0.2, 0, 0.4, 0.7) # Filled rectangle, (x, y, w, h)
] # Rectangles in absolute coordinates, (x, y, w, h)
map_generator = MapGenerator(map_path, map_name, map_size, resolution, origin)
map_image_path, map_yaml_path = map_generator.create_map(rectangles)
print('Map image saved to:', map_image_path)
