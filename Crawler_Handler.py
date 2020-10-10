# Author: Ori Levi - orilevi026@gmail.com
import RPi.GPIO as GPIO
import time
import pygame
import pigpio
import os     

os.system ("sudo  killall pigpiod")
time.sleep(0.2)
os.system ("sudo pigpiod")
time.sleep(0.2)

#initate program ?



#----------------------------------------------------MY FUNCTIONS
def ESC_Binding():
    print("CONNECTING ESC")
    pi.set_servo_pulsewidth(ESC_pin,1500)# GROUND ZERO OF MOTOR
    time.sleep(1.5)
    print("BIP!")
    time.sleep(0.5)
    pi.set_servo_pulsewidth(ESC_pin,0)
    print("CONNECTED SUCCESSFULY")
    
def DanceMode():
    print("PARTTTYYY TIME")
    MoveServo(servo_front_left,1.5)
    MoveServo(servo_front_right,1.5)
    time.sleep(0.3)
    MoveServo(servo_front_left,-1.5)
    MoveServo(servo_front_right,-1.5)
    time.sleep(0.3)
    MoveServo(servo_front_left,1.5)
    MoveServo(servo_front_right,1.5)
    time.sleep(0.3)
    MoveServo(servo_front_left,-1.5)
    MoveServo(servo_front_right,-1.5)
    time.sleep(0.3)
    
    MoveServo(servo_rear_right,1.5)
    MoveServo(servo_rear_left,1.5)
    time.sleep(0.3)
    MoveServo(servo_front_left,1.5)
    MoveServo(servo_front_right,1.5)
    time.sleep(0.3)
    MoveServo(servo_rear_right,-1.5)
    MoveServo(servo_rear_left,-1.5)
    time.sleep(0.3)
    MoveServo(servo_front_left,-1.5)
    MoveServo(servo_rear_left,-1.5)
    time.sleep(0.3)
    MoveServo(servo_front_right,1.5)
    MoveServo(servo_rear_right,1.5)
    time.sleep(0.3)
#--------------------------------------------------------MOTOR FUNCTIONS
def AxisToRPM_Gas(val):#throttle
    return 1560+(val+1)*70*Gear[gear_index]

def AxisToRPM_Reverse(val):#reverse
    return 1560-((1+val)*(105)*Gear[gear_index])

def SetSpeed (speed):
    pi.set_servo_pulsewidth(ESC_pin,speed)
    
#--------------------------------------------------------STEERING FUNCTIONS    
def SteerServo(servo,steer):# SAME AS MOVE SERVO - DIFFRENT BOUNDS
    steer = round(steer,3)
    print(steer)
    if(steer < 6.41 and steer > 6.39):
        steer = 6.4
    elif (steer) > 9.0:
        steer = 9
    elif (steer) < 4.7:
        steer = 4.1
    servo.ChangeDutyCycle(steer)
    time.sleep(0.02)
    servo.ChangeDutyCycle(0)
    return 0
    
def AxisToRPM(val):
    return (val + 1 )*500 +800

def AxisToDuty (val):
    return float((val +1)*2.6 +4)
    
    
def ReverseDuty (val):
    return float(val * (-1))+13


def MoveServo(servo, pos):
    if (7.5 + pos) > 10.2:
        return pos-0.3
    if (7.5 + pos) < 4.8:
        return pos-0.3
    servo.ChangeDutyCycle(7.5 + pos)
    time.sleep(0.1)
    return pos
#-------------------------------------------------GPIO PIN NUMS
ESC_pin = 21
steerPin = 20
servoFR = 13
servoFL = 19
servoRL = 5
servoRR = 6
buzzerPin = 23



#------------------------------------------INTAITE SERVOS AND MOTOR PART
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)# turning off warnings
pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC_pin,0)
time.sleep(0.1)
ESC_Binding()
GPIO.setup(steerPin, GPIO.OUT)
GPIO.setup(servoFR, GPIO.OUT)
GPIO.setup(servoFL, GPIO.OUT)
GPIO.setup(servoRL, GPIO.OUT)
GPIO.setup(servoRR, GPIO.OUT)
STEER_SERVO = GPIO.PWM(20, 50)
servo_front_right = GPIO.PWM(13, 50) # GPIO 17 for PWM with 50Hz
servo_front_left = GPIO.PWM(19, 50)
servo_rear_left = GPIO.PWM(5, 50)
servo_rear_right = GPIO.PWM(6, 50)
STEER_SERVO.start(6.4)
time.sleep(0.4)
STEER_SERVO.ChangeDutyCycle(0)
servo_front_right.start(7.5)
time.sleep(0.2)
servo_front_left.start(7.5)
time.sleep(0.2)
servo_rear_left.start(7.5)
time.sleep(0.2)
servo_rear_right.start(7.5)
time.sleep(0.2)

servo_front_right.ChangeDutyCycle(0)
servo_front_left.ChangeDutyCycle(0)
servo_rear_left.ChangeDutyCycle(0)
servo_rear_right.ChangeDutyCycle(0)
GPIO.setup(buzzerPin,GPIO.OUT)
# GPIO.output(buzzerPin,GPIO.HIGH)
# print("buzzerrrr")
# time.sleep(0.3)
# GPIO.output(buzzerPin,GPIO.LOW)


#-------------------------------------------------------VARS
done = False
active = True # true for right, false for left

motor_speed = 0
steer_step = 0
step_FR = 0
step_FL = 0
step_RR = 0
step_RL = 0
step_pos = 0
step_neg = 0
#--------------------------------------------------------GEAR SETUP
Gear = [1.2,1.8,3]
gear_index = 0

#-------------------------------------------------------PYGAME INTIATE
pygame.init() # initaite pygame module
pygame.joystick.init() # initaite joystick module
joystick = pygame.joystick.Joystick(0)
joystick.init()

#------------------------------------------------------EVENT HANDLER
print("CONTROLLER IS ACTIVE")
while done==False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
            print(done)
            
        if event.type == pygame.JOYAXISMOTION:# STEERING & THROTTLE FUNCTION----------------------
             if (event.axis == 0):
                 j_axis = ReverseDuty(AxisToDuty(joystick.get_axis(0))) # steering is reverse on car
                 SteerServo(STEER_SERVO,j_axis)
             
             
             if (event.axis == 4):
                motor_speed = AxisToRPM_Gas(joystick.get_axis(4))
                SetSpeed(motor_speed)
            
            
             if (event.axis == 5):
                motor_speed = AxisToRPM_Reverse(joystick.get_axis(5))
                SetSpeed(motor_speed)
             pygame.event.clear()
             
          
        if event.type == pygame.JOYHATMOTION:# HAT SECTION ------------------------------
            if joystick.get_hat(0) == (0, 1):#up
                step_FR = MoveServo(servo_front_right, step_FR+0.3)
                step_FL = MoveServo(servo_front_left, step_FL-0.3)
                step_RR = MoveServo(servo_rear_right, step_RR+0.3)
                step_RL = MoveServo(servo_rear_left, step_RL-0.3)
                time.sleep(0.1)
                
            if joystick.get_hat(0) == (-1, 0):#right
                step_FR = MoveServo(servo_front_right, step_FR-0.3)
                step_FL = MoveServo(servo_front_left, step_FL-0.3)
                step_RR = MoveServo(servo_rear_right, step_RR+0.3)
                step_RL = MoveServo(servo_rear_left, step_RL+0.3)
                time.sleep(0.1)
                
            if joystick.get_hat(0) == (1, 0):#left
                step_FR = MoveServo(servo_front_right, step_FR+0.3)
                step_FL = MoveServo(servo_front_left, step_FL+0.3)
                step_RR = MoveServo(servo_rear_right, step_RR-0.3)
                step_RL = MoveServo(servo_rear_left, step_RL-0.3)
                time.sleep(0.1)
                
            if joystick.get_hat(0) == (0, -1):#down
                step_FR = MoveServo(servo_front_right, step_FR-0.3)
                step_FL = MoveServo(servo_front_left, step_FL+0.3)
                step_RR = MoveServo(servo_rear_right, step_RR-0.3)
                step_RL = MoveServo(servo_rear_left, step_RL+0.3)
                time.sleep(0.1)
                
            servo_front_left.ChangeDutyCycle(0)
            servo_front_right.ChangeDutyCycle(0)
            servo_rear_left.ChangeDutyCycle(0)
            servo_rear_right.ChangeDutyCycle(0)

            
                
        if event.type == pygame.JOYBUTTONDOWN:# BUTTONS----------------------
            if joystick.get_button(11):# RESET THE SERVOS TO MIDDLE
                ESC_Binding()
                
            
            elif joystick.get_button(7):
                DanceMode()
                
            
            elif joystick.get_button(6):#GEAR
                if( gear_index == 2):
                    gear_index = 0
                    GPIO.output(buzzerPin,GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(buzzerPin,GPIO.LOW)
                else :
                    gear_index += 1
                    count = gear_index+1
                    while count > 0:
                        GPIO.output(buzzerPin,GPIO.HIGH)
                        time.sleep(0.1)
                        GPIO.output(buzzerPin,GPIO.LOW)
                        count= count-1
                        time.sleep(0.1)
                    print(gear_index+1)
             
            elif joystick.get_button(4):#ALL RISE!
                step_FR = 0
                step_FL = 0
                step_RR = 0
                step_RL = 0
                servo_front_left.ChangeDutyCycle(9.3)
                time.sleep(0.1)
                servo_front_right.ChangeDutyCycle(5.6)
                time.sleep(0.1)
                servo_rear_left.ChangeDutyCycle(4.8)
                time.sleep(0.1)
                servo_rear_right.ChangeDutyCycle(10.1)
                time.sleep(0.1)
                servo_front_left.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_front_right.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_rear_left.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_rear_right.ChangeDutyCycle(0)
                time.sleep(0.1)
            
            elif joystick.get_button(0):#LOW RIDER!
                step_FR = 0
                step_FL = 0
                step_RR = 0
                step_RL = 0
                servo_front_left.ChangeDutyCycle(4.7)
                time.sleep(0.1)
                servo_front_right.ChangeDutyCycle(10.2)
                time.sleep(0.1)
                servo_rear_left.ChangeDutyCycle(10.2)
                time.sleep(0.1)
                servo_rear_right.ChangeDutyCycle(4.7)
                time.sleep(0.1)
                servo_front_left.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_front_right.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_rear_left.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_rear_right.ChangeDutyCycle(0)
                time.sleep(0.1)
            elif joystick.get_button(1):
                step_pos = 0
                step_neg = 0
                servo_front_left.ChangeDutyCycle(5.6)
                time.sleep(0.1)
                servo_front_right.ChangeDutyCycle(5.6)
                time.sleep(0.1)
                servo_rear_left.ChangeDutyCycle(9.3)
                time.sleep(0.1)
                servo_rear_right.ChangeDutyCycle(9.3)
                time.sleep(0.1)
                servo_front_left.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_front_right.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_rear_left.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_rear_right.ChangeDutyCycle(0)
                time.sleep(0.1)
                
            elif joystick.get_button(3):
                step_pos = 0
                step_neg = 0
                servo_front_left.ChangeDutyCycle(9.3)
                time.sleep(0.1)
                servo_front_right.ChangeDutyCycle(9.3)
                time.sleep(0.1)
                servo_rear_left.ChangeDutyCycle(5.6)
                time.sleep(0.1)
                servo_rear_right.ChangeDutyCycle(5.6)
                time.sleep(0.1)
                servo_front_left.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_front_right.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_rear_left.ChangeDutyCycle(0)
                time.sleep(0.1)
                servo_rear_right.ChangeDutyCycle(0)
                time.sleep(0.1)
            
            STEER_SERVO.ChangeDutyCycle(0)
            servo_front_left.ChangeDutyCycle(0)
            servo_front_right.ChangeDutyCycle(0)
            servo_rear_left.ChangeDutyCycle(0)
            servo_rear_right.ChangeDutyCycle(0)
        pygame.event.pump()
            
 
pygmae.quit
p.stop()
GPIO.cleanup() 


