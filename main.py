import time
import Core_functs as bot
import asyncio

RUNNING = True
PICKING_UP = True
DELIVERING = True


def Main():  
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


async def main(right_adjust, left_adjust):
    cancel_event = asyncio.Event()  # Event to signal task cancellation

    task1 = asyncio.create_task(bot.move_forward(run_time=2, right_power=right_adjust, left_power=left_adjust, cancel_event=cancel_event))  # Start the motor movement asynchronously
    task2 = asyncio.create_task(bot.read_IMU(cancel_event=cancel_event))  # Start IMU readings asynchronously


    try:
        await task1
    except asyncio.CancelledError:
        print("Motor task was cancelled.")
    
    try:
        await task2
    except asyncio.CancelledError:
        print("IMU reading task was cancelled.")



with open('/imu_data.txt','a') as file:
            file.write("New Recording: \n")
            file.close()


# Run the event loop
right_power = 80
left_power = 75.5

while True:
    asyncio.run(main(right_power, left_power))
    
    
    with open("imu_correction_data.txt", "r") as file:
        correction = file.read()
        
    if correction == "left":
        right_power = 90
        left_power = 75.5
        
    if correction == "right":
        right_power = 80
        left_power = 85.5
        
    with open("imu_correction_data.txt", "w") as file:
            file.write("")
            file.close()
        

        
        





    


