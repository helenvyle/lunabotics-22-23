from evdev import InputDevice, categorize, ecodes, KeyEvent
import gpiozero
from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

gamepad = InputDevice('/dev/input/event3')
#factory = PiGPIOFactory(host='169.254.136.63')
factory = PiGPIOFactory(host='e4:5f:01:66:0c:69')
last = {"ABS_RZ": 128, "ABS_Y": 128}
transCO = .175

# motors
exc = gpiozero.PWMOutputDevice(17,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
exc_speed = .36
sto = gpiozero.PWMOutputDevice(26,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
sto_speed = .36
step = gpiozero.PWMOutputDevice(22,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
step_speed = .36
piv = gpiozero.PWMOutputDevice(16,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
piv_speed = .36
m1 = gpiozero.PWMOutputDevice(5,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
speedR = .36
m2 = gpiozero.PWMOutputDevice(6,active_high=True,initial_value=0,frequency=255,pin_factory=factory)
speedL = .36
Fservo_vert = Servo(25, pin_factory=factory)
Fvert = 0
Fservo_horiz = Servo(27, pin_factory=factory)
Fhoriz = 0
Bservo_vert = Servo(24, pin_factory=factory)
Bvert = 0
Bservo_horiz = Servo(23, pin_factory=factory)
Bhoriz = 0  

for event in gamepad.read_loop():
    if event.type == ecodes.EV_ABS:
        absevent = categorize(event)
#drive motors
        if ecodes.bytype[absevent.event.type][absevent.event.code] =='ABS_RZ':
            last["ABS_RZ"] = absevent.event.value
            
            speedR = ((last["ABS_RZ"]*transCO)+14)/100
            m1.value = speedR
            sleep(0.01)
            print (last["ABS_RZ"])
            print(speedR)
            
        if last["ABS_RZ"] == 127:
                m1.value = 0
                sleep(0.01)
                
        if ecodes.bytype[absevent.event.type][absevent.event.code] =='ABS_Y':
            last["ABS_Y"] = absevent.event.value
            
            speedL = (((255-last["ABS_Y"])*transCO)+14)/100
            m2.value = speedL
            sleep(0.01)
            print (255-last["ABS_Y"])
            print(speedL)
            
        if last["ABS_Y"] == 127:
                m2.value = 0
                sleep(0.01)
#servos
        if ecodes.bytype[absevent.event.type][absevent.event.code] =='ABS_HAT0Y':
            last["ABS_HAT0Y"] = absevent.event.value
            if last["ABS_HAT0Y"] == 1:
                Fservo_vert.value = Fvert
                sleep(0.5)
                Fvert = Fvert - 0.1
            if last["ABS_HAT0Y"] == -1:
                Fservo_vert.value = Fvert
                sleep(0.5)
                Fvert = Fvert + 0.1
            if Fvert < -1:
                    vert = -1
            if Fvert > 1:
                    vert = 1
            print(Fvert)
        if ecodes.bytype[absevent.event.type][absevent.event.code] =='ABS_HAT0X':
            last["ABS_HAT0X"] = absevent.event.value
            if last["ABS_HAT0X"] == -1:
                Fservo_horiz.value = Fhoriz
                sleep(0.5)
                Fhoriz = Fhoriz - 0.1
            if last["ABS_HAT0X"] == 1:
                Fservo_horiz.value = Fhoriz
                sleep(0.5)
                Fhoriz = Fhoriz + 0.1
            if Fhoriz < -1:
                    Fvert = -1
            if Fhoriz > 1:
                    Fhoriz = 1
            print(Fhoriz)
        
#buttons
    if event.type == ecodes.EV_KEY:
        keyevent = categorize(event)
        if keyevent.keystate == KeyEvent.key_down:
            if keyevent.scancode == 293:
                exc_speed = exc_speed - .01
                exc.value = exc_speed
                sleep(0.1)
                print('exc:')
                print(exc_speed)
            if keyevent.scancode == 295:
                exc_speed = exc_speed + .01
                exc.value = exc_speed
                sleep(0.1)
                print('exc:')
                print(exc_speed)
            if keyevent.scancode == 292:
                sto_speed = sto_speed - .01
                sto.value = sto_speed
                sleep(0.1)
                print('sto:')
                print(sto_speed)
            if keyevent.scancode == 294:
                sto_speed = sto_speed + .01
                sto.value = sto_speed
                sleep(0.1)
                print('sto:')
                print(sto_speed)
            if keyevent.scancode == 280:
                step_speed = step_speed - .01
                step.value = step_speed
                sleep(0.1)
                print('step:')
                print(step_speed)
            if keyevent.scancode == 290:
                step_speed = step_speed + .01
                step.value = step_speed
                sleep(0.1)
                print('step:')
                print(step_speed)
            if keyevent.scancode == 289:
                piv_speed = piv_speed - .01
                piv.value = piv_speed
                sleep(0.1)
                print('piv:')
                print(piv_speed)
            if keyevent.scancode == 291:
                piv_speed = piv_speed + .01
                piv.value = piv_speed
                sleep(0.1)
                print('piv:')
                print(piv_speed)