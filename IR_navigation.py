import time

objective = "pick-up"

def search_line():
  while True:
    move_forward()
    time.sleep(1)

    # returns True or false
    IR_present = read_IR()

    if IR_present:
      break

def follow_and_avoid():
  while True
    move_forward()
    time.sleep(1)

    # these values should return True or False
    obstacle_in_front = parse_front_sensor()
    pay_load_present = reed_switch()
    IR_present = IR_diode()

    if pay_load_present:
      return "pay load present"

    elif obstacle_present:
      return "obstacle present"

    elif IR_present:
      return "passed line"


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







    
    
    
