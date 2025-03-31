import time
import Core_functs as bot

objective = "pick-up"

def search_line():
  while True:
    move_forward()
    time.sleep(1)

    # returns True or false
    IR_present = read_IR()

    if IR_present:
      break

# Follow black line until obstacle
def follow_and_avoid():
  while IR_diode() == True:
    move_forward()
    time.sleep(1)

    # these values should return True or False
    obstacle_in_front = parse_front_sensor()
    pay_load_present = reed_switch()
    IR_present = IR_diode()

    if pay_load_present:
      turn360()

      time.sleep(1)

      move_backwards()

      pickup_payload()

      return "pay load detected and picked up"

    elif obstacle_in_front:
      turnleft()

      # Check for object before moving forward
      if not parse_front_sensor():
        move_forward()
        time.sleep(1)

      return "obstacle present"

    elif IR_present:
      return "passed line"

# Go around object
def navigate_obstacle():
  turn_left()

  for i in range(2):
    while True:
      move_forward()
      cleared_obstacle = parse_right_sensor()

      if cleared_obstacle:
        break

    turn_right()

    search_line()
    
    turn_left()


def pickup_payload():
  pass

def drop_off():
  pass

  

search_line()
turn_right()


RUNNING = True
PICKING_UP = True
DELIVERING = True



def Main():
  """
      Main function
  """
  while Running:

    while PICKING_UP:
      state = follow_and_avoid()

      if state == "pay load present":
        pickup_payload()
        break

      elif state == "obstacle present":
        navigate_obstacle()

    while DELIVERING:
      state = follow_and_avoid()

      if state == "passed line":
        drop_off()
        break

      elif state == "obstacle present":
        navigate_obstacle()







      
      
      
