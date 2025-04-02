import time
import Core_functs as bot

RUNNING = True
PICKING_UP = True
DELIVERING = True


def main():  
    bot.search_line()
    bot.turn_right()
  
  
    while RUNNING:
        while PICKING_UP:
            state = bot.follow_and_avoid()
            if state == "pay load present":
                bot.pickup_payload()
                break

            elif state == "obstacle detected":
                bot.navigate_obstacle()

main()


