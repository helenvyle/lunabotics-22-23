from evdev import InputDevice, categorize, ecodes, KeyEvent
import gpiozero
import math
import rospy
from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

gamepad = InputDevice('/dev/input/event3')
#factory = PiGPIOFactory(host='169.254.136.63')
factory = PiGPIOFactory(host='e4:5f:01:66:0c:69')
last = {"ABS_RZ": 128, "ABS_Y": 128}
transCO = .175



class Driver: 
    def __init__(self):
        """
        Class constructor
        """
        # motors
        self.exc = gpiozero.PWMOutputDevice(17,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
        self.exc_speed = .36
        self.sto = gpiozero.PWMOutputDevice(26,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
        self.sto_speed = .36
        self.step = gpiozero.PWMOutputDevice(22,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
        self.step_speed = .36
        self.piv = gpiozero.PWMOutputDevice(16,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
        self.piv_speed = .36
        self.m1 = gpiozero.PWMOutputDevice(5,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
        self.speedR = .36
        self.m2 = gpiozero.PWMOutputDevice(6,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
        self.speedL = .36

        # Variables

        #Orientation
        self.local_orientation = 0
        self.global_orientation = 0
        self.use_local= True

        #Position
        self.local_x = 0
        self.local_y = 0
        self.global_x = 0
        self.global_y = 0

        #Speed
        self.linear_speed = 0
        self.angluar_speed = 0

        

    def send_speed(self, speed_1, speed_2):
        ## TODO 
        ## Match speeds to corresponding motors in documentation
        self.m1.value = speed_1
        self.m2.value = speed_2

    def drive(self, distance, linear_speed):
        start_pose=[]
        if(self.use_local):
            start_pose = [self.local_x, self.local_y]
        else:
            start_pose = [self.global_x, self.global_y]
        ## While the current distance is less than the target distance, drive
        while(math.sqrt( pow((self.px - start_pose[0]), 2) + pow((self.py - start_pose[1]), 2)) < distance ):
            self.send_speed(self.linear_speed, self.linear_speed)
            rospy.sleep(0.5)

        ## When done stop
        self.send_speed(0.0,0.0)

    def rotate(self, angle, aspeed):
        ## Change turn direction for negative angles
        if (angle < 0): ## TODO figure out what orientation system we want to use
            aspeed = -aspeed
        targetAngle = angle
        fast_tolerance = 0.5 #TODO relate this to orientation system, its the angle difference in which we turn quickly
        angle_tolerance = 0.02
        if(self.use_local):
            ## Turn towards target angle until within tolerance
            while (abs(targetAngle - self.local_orientation) > angle_tolerance):
                if (abs(targetAngle - self.local_orientation) > fast_tolerance):
                    self.send_speed(aspeed/2, aspeed)
                else:
                    self.send_speed(aspeed/4, aspeed/2)
                rospy.sleep(0.05)
        else:
            ## Turn towards target angle until within tolerance
            while (abs(targetAngle - self.global_orientation) > angle_tolerance):
                if (abs(targetAngle - self.pth) > fast_tolerance):
                    self.send_speed(aspeed/2, aspeed)
                else:
                    self.send_speed(aspeed/4, aspeed/2)
                rospy.sleep(0.05)
        
        ## Stop
        self.send_speed(0, 0)