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

course_layout =[[0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0],
                [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]


# Movement directions (row, col)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
MOVES = ["up", "down", "left", "right"]

def heuristic(a, b):
    """Calculate Manhattan distance as heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(course_layout, start, goal):
    """Perform A* search to find the shortest path."""
    rows, cols = len(course_layout), len(course_layout[0])
    open_list = []
    heapq.heappush(open_list, (0, start))  # (f, (row, col))

    g_cost = {start: 0}
    parent = {start: None}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            return reconstruct_path(parent, start, goal)

        for i, (dr, dc) in enumerate(DIRECTIONS):
            neighbor = (current[0] + dr, current[1] + dc)

            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue  # Out of bounds
            if course_layout[neighbor[0]][neighbor[1]] == 1:
                continue  # Obstacle

            new_g = g_cost[current] + 1

            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                f_cost = new_g + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_cost, neighbor))
                parent[neighbor] = (current, MOVES[i])

    return None  # No path found

def reconstruct_path(parent, start, goal):
    """Reconstruct the path and moves from start to goal."""
    path = []
    moves = []
    current = goal
    while current != start:
        prev, move = parent[current]
        moves.append(move)
        path.append(current)
        current = prev
    path.reverse()
    moves.reverse()
    return path, moves





def find_min_rotations(start_angle, target_angle):
    """
    Find the minimum number of 90-degree rotations to reach target_angle from start_angle.
    Returns the number of steps and the direction (right or left).
    """
    # Normalize angles to [0, 360)
    start_angle %= 360
    target_angle %= 360

    # Calculate clockwise and counterclockwise steps
    clockwise_steps = (target_angle - start_angle) % 360 // 90
    counterclockwise_steps = (start_angle - target_angle) % 360 // 90

    # Determine the minimum steps and direction
    if clockwise_steps <= counterclockwise_steps:
        return clockwise_steps, "right"
    else:
        return counterclockwise_steps, "left"


# Corrected direction angles (up is 0°)
direction_angles = {"right": 270, "left": 90, "up": 0, "down": 180}


def execute_move(move):
    """Execute move functions."""
    global global_orientation_angle

    target_angle = direction_angles.get(move)

    # If already facing the correct direction, move forward
    if global_orientation_angle == target_angle:
            move_forward()

    else:
        # Find the minimum number of turns and the direction
        num_turns, turn_type = find_min_rotations(target_angle, global_orientation_angle)

        # Execute the correct number of turns
        if turn_type == "right":
            for _ in range(num_turns):
                turn_right()
        elif turn_type == "left":
            for _ in range(num_turns):
                turn_left()

        # Move forward after turning
        move_forward()

        # Finally, update the global angle orientation
        global_orientation_angle = target_angle


def move_forward():
  print("moving forward")

def turn_right():
  print("turning right")


def turn_left():
  print("turning left")
  
  
  
  
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
        self.sensor_orientation = (global_orientation_angle + self.adjustment) % 360

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

        # Calculate obstacle position
        obstacle_row = int(current_position[0] + (row_direction * distance_tiles))
        obstacle_col = int(current_position[1] + (col_direction * distance_tiles))

        # Check if the obstacle position is within bounds
        if 0 <= obstacle_row < len(course_layout) and 0 <= obstacle_col < len(course_layout[0]):
            course_layout[obstacle_row][obstacle_col] = 1  # Mark obstacle on the map
            print(f"Obstacle detected at ({obstacle_row}, {obstacle_col}) and updated on the map.")
        else:
            print("Obstacle detected out of bounds. Map not updated.")
            
            
def print_course_layout(course_layout):
    """Prints the course layout using '·' for empty spaces and 'x' for obstacles with spacing."""
    for row in course_layout:
        print(" ".join("x" if cell == 1 else "·" for cell in row))
    print()
    
    
def get_average_distance(sensor, num_readings=10):
    total_distance = 0

    for i in range(num_readings):
        distance = sensor.distance_cm()  # Get sensor reading
        total_distance += distance

    # Calculate average
    average_distance = total_distance / num_readings
    return average_distance

       

  
start = (0, 0)
goal = (7, 15)
path, moves = a_star(course_layout, start, goal)

global_orientation_angle = 0
current_position = (0, 0)

objective = "pick-up" # deliver


Running = True
picking_up = True
delivering = True


# Initialize sensor with trigger and echo pins
sensor_front = HCSR04(trigger_pin=21, echo_pin=20)
sensor_back =HCSR04(trigger_pin=21, echo_pin=20)
sensor_rotating = HCSR04(trigger_pin=21, echo_pin=20)

# Initialize servo
sg90 = Servo(Pin(15))



while Running:
    while picking_up:
        
        # Use sensors to update the map
        
        
        front_distance = get_average_distance(sensor_front)
        front_sensor_parsed = sensor_data('front', front_distance)
        front_sensor_parsed.update_map(current_position, course_layout)
        
        back_distance = get_average_distance(sensor_back)
        back_sensor_parsed = sensor_data('back', back_distance)
        back_sensor_parsed.update_map(current_position, course_layout)
        
        
        # turn servo right
        
        sg90.move(90)
        sleep(0.5)
        
        right_distance = get_average_distance(sensor_rotating)
        right_sensor_parsed = sensor_data('right', right_distance)
        right_sensor_parsed.update_map(current_position, course_layout)
        
        sg90.move(180)
        sleep(0.5)
        
        # turn servo left
        left_distance = get_average_distance(sensor_rotating)
        left_sensor_parsed = sensor_data('left', left_distance)
        left_sensor_parsed.update_map(current_position, course_layout)
        
        # re-centre the servo
        sg90.move(0)
        sleep(0.5)
        
        # Find the optimal path
        path, moves = a_star(course_layout, start, goal)
        
        # Execute the directed moves
        execute_moves(moves[0])
        
        # Detect if there is a payload
        if pay_load_detected:
            break
        
    # pick up the pay_load
        
        
    while delivering:
        # Adjust our global position:
        global_orientation_angle = (global_orientation_angle + 180) % 360
        
        # Rely only on the back and rotating sensors
    
        back_distance = get_average_distance(sensor_back)
        back_sensor_parsed = sensor_data('front', back_distance)
        back_sensor_parsed.update_map(current_position, course_layout)
        
        
        # turn servo right
        sg90.move(90)
        sleep(0.5)
        
        right_distance = get_average_distance(sensor_rotating)
        right_sensor_parsed = sensor_data('right', right_distance)
        right_sensor_parsed.update_map(current_position, course_layout)
        
        
        # turn servo left
        sg90.move(180)
        sleep(0.5)
    
        left_distance = get_average_distance(sensor_rotating)
        left_sensor_parsed = sensor_data('left', left_distance)
        left_sensor_parsed.update_map(current_position, course_layout)
        
        # re-centre the servo
        sg90.move(0)
        sleep(0.5)
        
        # Find the optimal path
        path, moves = a_star(course_layout, start, goal)
        
        # Execute the directed moves
        execute_moves(moves[0])
        
        # Detect if we have crossed the IR line
        if reached_IR:
            # Re-adjust our global position again:
            global_orientation_angle = (global_orientation_angle + 180) % 360
            break
        
    # release the pay_load
        


