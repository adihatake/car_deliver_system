import heapq
from hcsr04 import HCSR04
from time import sleep
from servo import Servo
from machine import Pin


# create a 16x8 matrix of the course layout
# each space is 15x15cm

course_layout =[[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

  
class sensor_data:
    
    def __init__(self, placement, distance_cm):
        global global_orientation_angle
        self.sensor_placement = placement
        self.distance_cm = distance_cm

        # Determine orientation adjustment based on sensor placement
        if placement == "front":
            self.adjustment = 0
        elif placement == "back":
            self.adjustment = 180
        elif placement == "right":
            self.adjustment = -90
        elif placement == "left":
            self.adjustment = 90
            
        self.sensor_orientation = (global_orientation_angle + self.adjustment) % 360


    def parse_sensor_data(self):
        incremented_scale = self.distance_cm // 15  # Convert distance to tile increments
        return incremented_scale

    def update_map(self, current_position, course_layout):
        """
        Update the map based on the sensor data.
        Marks the obstacle position on the map.
        """
        global global_orientation_angle
        
        # Calculate sensor's orientation relative to global orientation
        
        print(f"sensor_orientation: {self.sensor_orientation}")

        # Convert orientation to direction index
        if self.sensor_orientation == 0:  # Up
            row_direction, col_direction = -1, 0
        elif self.sensor_orientation == 180:  # Down
            row_direction, col_direction = 1, 0
        elif self.sensor_orientation == 90:  # Left
            row_direction, col_direction = 0, -1
        elif self.sensor_orientation == 270:  # Right
            row_direction, col_direction = 0, 1

        # Get distance in tiles
        distance_tiles = self.parse_sensor_data()
        print(distance_tiles)

        # Calculate obstacle position
        obstacle_row = int(current_position[0] + (row_direction * distance_tiles))
        obstacle_col = int(current_position[1] + (col_direction * distance_tiles))
        
        print(f"obstacle_row : {obstacle_row}, obstacle_col: {obstacle_col}")

        # Check if the obstacle position is within bounds
        if 0 <= obstacle_row < len(course_layout) and 0 <= obstacle_col < len(course_layout[0]):
            course_layout[obstacle_row][obstacle_col] = 1  # Mark obstacle on the map
            print(f"Obstacle detected at ({obstacle_row}, {obstacle_col}) and updated on the map.")
        else:
            print("Obstacle detected out of bounds. Map not updated.")
            
            
def print_course_layout(current_position=None):
    """Print the course layout and mark the current position."""
    global course_layout

    for row_idx, row in enumerate(course_layout):
        for col_idx, cell in enumerate(row):
            if current_position and (row_idx, col_idx) == current_position:
                print("C", end=" ")  # Mark the current position
            else:
                print("x" if cell == 1 else "·", end=" ")
        print()  # Newline after each row
    print()

    
    
def get_average_distance(sensor, num_readings=10):
    total_distance = 0

    for i in range(num_readings):
        distance = sensor.distance_cm()  # Get sensor reading
        total_distance += distance

    # Calculate average
    average_distance = total_distance / num_readings
    return average_distance

       

global_orientation_angle = 270
current_position = (6, 6)

# Initialize sensor with trigger and echo pins
sensor_front = HCSR04(trigger_pin=21, echo_pin=20)


while True:
    
    front_distance = get_average_distance(sensor_front)
    front_sensor_parsed = sensor_data('front', front_distance)
    front_sensor_parsed.update_map(current_position, course_layout)
    
    print_course_layout(current_position)






