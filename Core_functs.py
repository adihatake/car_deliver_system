from hcsr04 import HCSR04 # Must have this library saved on Pico to work
from machine import Pin, PWM, ADC, I2C
from imu import MPU6050


import time
from time import sleep

import asyncio
import math


"""
Motor Control

"""


# Function to control Motor A
def motor_a(direction = "stop", speed = 0):
        
    motor_a_in1 = Pin(6, Pin.OUT)
    motor_a_in2 = Pin(7, Pin.OUT)
    motor_a_en = PWM(Pin(8))
    motor_a_en.freq(1000)
    motor_a_correction = 1.0 # Adjust so both motors have same speed
        
        
    adjusted_speed = int(speed * motor_a_correction)  # Apply correction
        
    if direction == "forward":
        motor_a_in1.value(0)
        motor_a_in2.value(1)
    elif direction == "backward":
        motor_a_in1.value(1)
        motor_a_in2.value(0)
    else:  # Stop
        motor_a_in1.value(0)
        motor_a_in2.value(0)
    motor_a_en.duty_u16(int(adjusted_speed * 65535 / 100))  # Speed: 0-100%

# Function to control Motor B
def motor_b(direction = "stop", speed = 0):

    motor_b_in3 = Pin(4, Pin.OUT)
    motor_b_in4 = Pin(3, Pin.OUT)
    motor_b_en = PWM(Pin(2))
    motor_b_en.freq(1000)
    motor_b_correction = 1.0 # Adjust so both motors have same speed
        
        
        
    adjusted_speed = int(speed * motor_b_correction)  # Apply correction
    if direction == "forward":
        motor_b_in3.value(1)
        motor_b_in4.value(0)
    elif direction == "backward":
        motor_b_in3.value(0)
        motor_b_in4.value(1)
    else:  # Stop
        motor_b_in3.value(0)
        motor_b_in4.value(0)
    motor_b_en.duty_u16(int(adjusted_speed * 65535 / 100))  # Speed: 0-100%
    

async def move_forward(run_time = 0.25, right_power = 60, left_power = 55.5, cancel_event = None):
    while not cancel_event.is_set(): 
        # left motor
        motor_b("forward", left_power)
        # right motor
        motor_a("backward", right_power)
            
        await asyncio.sleep(run_time)

        # Turn off motors
        motor_a()
        motor_b()
    
    
    
 
### cannot go lower than 40
def turn_right(run_time = 0.54):
    motor_a("forward", 40)
    motor_b("forward", 40)
    sleep(run_time)
    
    motor_a()
    motor_b()
    
    sleep(1)
    
    

def turn_left(run_time = 0.54):
    motor_a("backward", 40)
    motor_b("backward", 40)
    sleep(run_time)
    
    motor_a()
    motor_b()
    
    sleep(1)

def turn360():
    pass


def move_backwards():
    pass





def read_sensor(trigger_pin, echo_pin):
    # Initialize sensor with trigger and echo pins
    sensor = HCSR04(trigger_pin, echo_pin)
    distance = sensor.distance_cm()
    print('Distance:', distance, 'cm')
    sleep(0.1) # sensor doesn't work well without delay 

    return distance





"""
Initialize all sensors readings

"""


def parse_front_sensor():
    """
    Checks if there is an obstacle in front

    """
    
    
    front_distance = read_sensor(21,20)
    
    # if there is an obstacle in front, return True
    if front_distance < 8:
        return True
    
    # return false if there is no obstacle in front
    else:
        return False
    
    
    
def parse_right_sensor():
    """
    Checks if we have cleared the wall on the side

    """
    
    right_distance = read_sensor(14,15)
    
    # checks if the distance detectec is greater than the point blank wall
    # if it is, then we know that we have cleared the wall segment 
    if right_distance > 20:
        return True
    
    else:
        return False

async def read_IMU(threshold = 0.1, cancel_event = None):
    while not cancel_event.is_set():
        i2c = I2C(1, scl=Pin(19), sda=Pin(18))
        imu = MPU6050(i2c)
        
        reading = imu.accel.x
        

            
        if math.fabs(reading) > threshold:
            
            if reading < 0:
                correction = "right"
                
            if reading > 0:
                correction = "left"
            
            with open('/imu_data.txt','a') as file:
                file.write(f"terminating: with: {reading} and correction: {correction} \n")
                file.close()
                
            with open("imu_correction_data.txt", "w") as file:
                file.write("")
                file.close()
                
            with open('/imu_correction_data.txt','a') as file:
                file.write(f"{correction}")
                file.close()
                
            cancel_event.set()
            
            break
         
    
    return imu.accel.x



def read_photodiode():
    ir = ADC(28)
    
    return ir.read_u16()

def read_IR():
    line_sen = Pin(17, Pin.IN)
    
    # if a black is detected, return True
    if line_sen.value() == 0:
        return True
    else:
        return False
    
    sleep(0.1)

def read_reed():
    # Reed swich on pin 0 using internal pull down resistor, other wire of switch connects to 3.3V
    reed_switch = Pin(0, Pin.IN, Pin.PULL_DOWN)
    
    # if there is a magnet, return true
    if reed_switch.value() == 1:
        return True
    else:
        return False
    
    
    
    
"""
Define navigational logic

"""



def search_line():
  while True:
    move_forward()

    # returns True or false
    IR_present = read_IR()

    if IR_present:
      break

# Follow black line until obstacle
def follow_and_avoid():
    move_forward()

    # Check if we are approaching an obstacle and if there is a pay_load
    obstacle_in_front = parse_front_sensor()
    pay_load_present = read_reed()
    IR_present = read_photodiode()

    if pay_load_present:
        return "pay load present"

    elif obstacle_in_front:
        print('obstacle detected')
        return'obstacle detected'       
          
    elif IR_present:
        return 'passed line'
        
            


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
      move_forward()
      
  search_line()
  turn_left()            




