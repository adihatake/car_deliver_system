from hcsr04 import HCSR04 # Must have this library saved on Pico to work
from machine import Pin, PWM, ADC
from time import sleep

def drive():
    # === L298N Motor Driver ===
    # Motor A
    motor_a_in1 = Pin(6, Pin.OUT)
    motor_a_in2 = Pin(7, Pin.OUT)
    motor_a_en = PWM(Pin(8))
    motor_a_en.freq(1000)
    motor_a_correction = 1.0 # Adjust so both motors have same speed

    # Motor B
    motor_b_in3 = Pin(4, Pin.OUT)
    motor_b_in4 = Pin(3, Pin.OUT)
    motor_b_en = PWM(Pin(2))
    motor_b_en.freq(1000)
    motor_b_correction = 1.0 # Adjust so both motors have same speed

    # Function to control Motor A
    def motor_a(direction = "stop", speed = 0):
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


    # Main code
    motor_a("backward", 50)

    sleep(2)

    motor_b("forward", 50)

    sleep(2)

    # Turn off motor a using default values
    motor_a()

    # Turn off motor b using set values
    motor_b("stop", 0)

    return

def read_sensor(trigger_pin, echo_pin):
    # Initialize sensor with trigger and echo pins
    sensor = HCSR04(trigger_pin, echo_pin)

    try:
        distance = sensor.distance_cm()
        print('Distance:', distance, 'cm')
        sleep(0.1) # sensor doesn't work well without delay
    except OSError as ex:
        print('ERROR getting distance:', ex)    

    return distance

# Reads the front sensor
def read_front():
    read_sensor(21,20)

# Read the right sensor
def read_right():
    read_sensor(14,15)

def read_photodiode():
    ir = ADC(28)
    
    return ir.read_u16()

def read_IR():
    line_sen = Pin(17, Pin.IN)

    return line_sen.value()

def read_reed():
    # Reed swich on pin 0 using internal pull down resistor, other wire of switch connects to 3.3V
    reed_switch = Pin(0, Pin.IN, Pin.PULL_DOWN)

    return reed_switch.value()
