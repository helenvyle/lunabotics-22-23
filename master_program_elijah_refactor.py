from evdev import InputDevice, categorize, ecodes, KeyEvent
import gpiozero
from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Button

gamepad = InputDevice('/dev/input/event2')
factory = PiGPIOFactory(host='169.254.136.63')
last = {"ABS_RZ": 128, "ABS_Y": 128}
transCO = .175
hall = Button(2, pin_factory=factory)

#classes to manage the different control schemes

class MotorControl: #contains all relevant information to drive the motors; should encapsulate the if/else statements regarding ABS_RZ and ABS_Y
    m1 = gpiozero.PWMOutputDevice(5,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
    speedR = .36
    m2 = gpiozero.PWMOutputDevice(6,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
    speedL = .36

    motor_dict = {"ABS_RZ": [m1, speedR], "ABS_Y":[m2, speedL]}

    def motor_adjust(self, abs, absevent): #adjusts the motors based on if the input is ABS_RZ or ABS_Y (it uses 255-last[abs] if it is Y)
        last[abs] = absevent.event.value
        event_parameter = last[abs]
        if abs == "ABS_Y":
            event_parameter = 255-event_parameter
            Y = True
        self.motor_dict[abs][1] = ((event_parameter*transCO)+14)/100
        self.motor_dict[abs][0].value = self.motor_dict[abs][1]
        sleep(0.01)
        print(event_parameter)
        print(self.motor_dict[abs][1])
        if Y:
            event_parameter = last[abs]
        if event_parameter == 127:
            self.motor_dict[abs][0].value = 0
            sleep(0.01)
        return



class ServoControl: #contains all relevant information to control the servos; should encapsulate the if/else statements regarding ABS_HAT0(X or Y) when servo = 0 or 1, and should also encapsulate keyevents 296 and 297 representing a change in the servo variable
    Fservo_vert = Servo(25, pin_factory=factory)
    Fvert = 0
    Fservo_horiz = Servo(16, pin_factory=factory)
    Fhoriz = 0
    Bservo_vert = Servo(24, pin_factory=factory)
    Bvert = 0
    Bservo_horiz = Servo(23, pin_factory=factory)
    Bhoriz = 0  
    servo = 1

    servo_dict= {["ABS_HAT0Y",1]: Fvert, ["ABS_HAT0X",1]: Fhoriz, ["ABS_HAT0Y",0]: Bvert, ["ABS_HAT0X",0]: Bhoriz}
    servo_value_dict = {Fhoriz:Fservo_horiz, Bvert:Bservo_vert, Bhoriz:Bservo_horiz, Fvert:Fservo_vert}

    def HAT0_servo(self, abs, absevent): #takes two inputs: "ABS_HAT0(X or Y)" and an integer (0 or 1, representing servo state). It pulls the relevant servo (Fhoriz, Fvert, Bhoriz, or Bvert) from servo_dict and performs the relevant movement
        key = [abs, self.servo]
        try:
            servo_direction = self.servo_dict[key] 
        except KeyError:
            print("Invalid Input (issue in HAT0_servo function)") #compare input to servo_dict to check for discrepancies if there is an error here
            return

        last[abs] = absevent.event.value 
        abs_last = last[abs]                  
        if abs_last == 1 or abs_last == -1:
                            
            servo_direction += (abs_last)*0.1 #serves the same purpose as adding or subtracting 0.1, since the function doesn't execute if abs_last != 1 or -1, and avoids the if statement
            if servo_direction < -1:
                servo_direction= -1
            elif servo_direction > 1: #elif statement allows code to ignore checking if servo_direction > 1 if we already know servo_direction < -1
                servo_direction = 1
            
            self.servo_value_dict[servo_direction].value = servo_direction 
            sleep(0.5)

        print(servo_direction)
        return

    def servo_switch(self,scancode): #switches between front and back servo
        if scancode == 296:
            self.servo = 0
            print(self.servo)
            return
        if scancode == 297:
            self.servo = 1
            print(self.servo)
            return
        print("Invalid scancode passed to servo_switch")
        return




class SpeedControl: #contains all relevant information to control the speeds; should encapsulate all the button statements besides 296 and 297
    
    exc = gpiozero.PWMOutputDevice(17,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
    exc_speed = .36
    sto = gpiozero.PWMOutputDevice(26,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
    sto_speed = .36
    step = gpiozero.PWMOutputDevice(22,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
    step_speed = .36
    piv = gpiozero.PWMOutputDevice(27,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
    piv_speed = .36

    #organizes scancodes based on order
    speed_scancodes = [288,289,290,291,292,293,294,295]
    neg_scancodes = [288, 289, 292, 293]
    pos_scancodes = [290, 291, 294, 295]
    exc_scancodes = [293, 295]
    sto_scancodes = [292, 294]
    step_scancodes = [288, 290]
    piv_scancodes = [289, 291]

    speedtype_dict = {exc_scancodes:exc, sto_scancodes:sto, step_scancodes:step, piv_scancodes:piv}
    speedvalue_dict = {exc:exc_speed, sto:sto_speed, step:step_speed, piv:piv_speed}

    def speed_adjust(self, scancode): #does a simple matching search to check if the scancode corresponds to a positive or negative change, and a second to determine what speed is changing. should be faster because once it finds its target it stops searching
        if scancode in self.neg_scancodes:
            direction = -1
        elif scancode in self.pos_scancodes:
            direction = 1
        else:
            print("Invalid scancode passed to speed_adjust")
            return
        type = 0
        for speedtype in self.speedtype_dict:
            if scancode in speedtype:
                type = self.speedtype_dict[speedtype] 
                break
        
        if type == 0:
            print("Invalid scancode passed to speed_adjust")
            return
        speed = self.speedvalue_dict[type]
        speed += direction*0.1
        type.value = speed
        sleep(0.1)
        print(str(type) + ": " + str(speed))
        return



class Robot: #defines the "Robot" class, which in turn contains all the relevant control schemes

    motors = MotorControl
    servos = ServoControl
    speed = SpeedControl

robot = Robot #here I declare an object "robot" of the type "Robot", the class I defined above. I am creating an "instance" of the class
print("start") #while True should be redundant; the evdev article from your code notes suggests that evdev manages the program loop for you
for event in gamepad.read_loop():
    if hall.is_pressed:
        robot.speed.step_speed = .36
        robot.speed.step.value = robot.speed.step_speed #these dots are member access variables; read this like directories (i.e. it looks for a variable by going through the robot folder, finds the speed folder, opens the speed folder, and edits the step.value variable inside. it sounds slow described like that but the compiler should be able to do it near instantly)
        print("end")
        sleep(1)
    if event.type == ecodes.EV_ABS:
        absevent = categorize(event)
        event_call = ecodes.bytype[absevent.event.type][absevent.event.code] #if event type is EV_ABS, assigns it to variable event_call
#drive motors
        if absevent in robot.motors.motor_dict: #checks if event is a motor event. if it is, it calls the relevant function, and never checks if the event is a servo or speed event (because each event can only be one at a time)
            print("Recieving input to motors")
            robot.motors.motor_adjust(event_call, absevent) #calls motor_adjust function on the robot object
#controls servo movement
        elif absevent in robot.servos.servo_dict:
            print("Recieving input to servos")
            robot.servos.HAT0_servo(event_call, absevent) #calls HAT0_servo function on the robot object
        else: #if the signal was sent here, and isn't in either dictionary, it prints an error for debugging
            print("Error routing ecodes.EV_ABS event call")
        
#buttons, two change which servo is getting input, and the rest adjust speed
    if event.type == ecodes.EV_KEY:
        keyevent = categorize(event)
        if keyevent.keystate == KeyEvent.key_down: #not sure if this is redundant with ecodes.EV_KEY, left it in just in case
            if keyevent == 296 or keyevent == 297: #if button is a servo switch order, calls servo_switch and doesn't check if button is a speed_adjust order
                print("Recieving servo switch order")
                robot.servos.servo_switch(keyevent.scancode)
            elif keyevent in robot.speed.speed_scancodes:
                print("Recieving speed adjust order")
                robot.speed.speed_adjust(keyevent.scancode)
            else:
                print("Error routing ecodes.EV_KEY event call") #if the signal was sent here, and isn't in either dictionary, it prints an error for debugging
        