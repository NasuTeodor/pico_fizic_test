from machine import Pin, I2C, PWM
from uasyncio import sleep_ms   
from vl53l0x import setup_tofl_device, TBOOT
import utime
from time import sleep
from mpu6050 import MPU6050

def wrap(angle):
    while (angle > +180):
        angle -= 360
    while (angle < -180):
        angle += 360
    return angle

# shutdown pins for each device
device_0_xshut = Pin(14, Pin.OUT)
device_1_xshut = Pin(15, Pin.OUT)
device_2_xshut = Pin(18, Pin.OUT)

# setup led
glb = Pin(22, Pin.OUT)
vrd = Pin(28, Pin.OUT)

# setup dc mot
dr = Pin(21, Pin.IN)
st = Pin(20, Pin.IN)

START = Pin(9, Pin.IN)

glb.value(1)
vrd.value(0)

# setup i2c bus 0
i2c_0 = I2C(id=0, sda=Pin(16), scl=Pin(17))

# setup bus 1''
i2c_1 = I2C(id=1, sda=Pin(26), scl=Pin(27), freq=40000)

        

#PWM DC Mot
m_left1 = PWM(Pin(10))
m_left2 = PWM(Pin(11))

m_right1 = PWM(Pin(13))
m_right2 = PWM(Pin(12))

# set the PWM freq
PWM_FREQ = 1000

m_left1.freq(PWM_FREQ)
m_left2.freq(PWM_FREQ)
m_right1.freq(PWM_FREQ)
m_right2.freq(PWM_FREQ)

MOTOR_STATE = 'STOP'
MOTOR_MAX = 65025
MOTOR_SPEED = 0.5
REAL_SPEED = MOTOR_SPEED * MOTOR_MAX
REAL_SPEED = int(REAL_SPEED)

BRAKE_TIME = 350
LONG_BRAKE_TIME = 500

TURN_ANGLE = 45
TURN_SPEED = int(MOTOR_MAX)

LEFT_SPEED = REAL_SPEED
RIGHT_SPEED = REAL_SPEED

# setup a fuction to quickly use the motors 
def m_speed(speed_left, speed_right, direction):

    left_1 = speed_left
    left_2 = speed_left

    right_1 = speed_right
    right_2 = speed_right

    if direction == 1:
        left_1 = 0
        right_1 = 0
    elif direction == -1:
        left_2 = 0
        right_2 = 0
    elif direction == 0:
        m_left1.duty_u16(0)
        m_left2.duty_u16(0)
        m_right1.duty_u16(0)
        m_right2.duty_u16(0)
        #let the duty(0) take effect
        utime.sleep_ms(1)
        m_left1.deinit()
        m_left2.deinit()
        m_right1.deinit()
        m_right2.deinit()

    if direction != 0:
        m_left1.init(freq=PWM_FREQ, duty_u16=left_1) # type: ignore
        m_left1.duty_u16(left_1)
        m_left2.init(freq=PWM_FREQ, duty_u16=left_2) # type: ignore
        m_left2.duty_u16(left_2)
        m_right1.init(freq=PWM_FREQ, duty_u16=right_1) # type: ignore
        m_right1.duty_u16(right_1)
        m_right2.init(freq=PWM_FREQ, duty_u16=right_2) # type: ignore
        m_right2.duty_u16(right_2)

    # if direction == 1:
    #     m_left1.duty_u16(0)
    #     m_left2.init(freq=PWM_FREQ, duty_u16=speed) # type: ignore
    #     # m_left2.duty_u16(speed)
    #     m_right1.duty_u16(0)
    #     m_right2.init(freq=PWM_FREQ, duty_u16=speed) # type: ignore
    #     # m_right2.duty_u16(speed)
    # elif direction == -1:
    #     # m_left1.duty_u16(speed)
    #     m_left1.init(freq=PWM_FREQ, duty_u16=speed) # type: ignore
    #     m_left2.duty_u16(0)
    #     # m_right1.duty_u16(speed)
    #     m_right1.init(freq=PWM_FREQ, duty_u16=speed) # type: ignore
    #     m_right2.duty_u16(0)
    # elif direction == 0:
    #     m_left1.duty_u16(0)
    #     m_left2.duty_u16(0)
    #     m_right1.duty_u16(0)
    #     m_right2.duty_u16(0)

    #     m_left1.deinit()
    #     m_left2.deinit()
    #     m_right1.deinit()
    #     m_right2.deinit()

def ajutor_ma_mananca_mama():
    # SupportsIndex
    a = 9

def m_turn(turn_speed, direction):
    if direction == 'LEFT':
        m_left1.init(freq=PWM_FREQ, duty_u16=turn_speed) # type: ignore
        m_left1.duty_u16(turn_speed)
        m_left2.init(freq=PWM_FREQ, duty_u16=0) # type: ignore
        m_left2.duty_u16(0)
        m_right1.init(freq=PWM_FREQ, duty_u16=0) # type: ignore
        m_right1.duty_u16(0)
        m_right2.init(freq=PWM_FREQ, duty_u16=turn_speed) # type: ignore
        m_right2.duty_u16(turn_speed)
    elif direction == 'RIGHT':
        m_left1.init(freq=PWM_FREQ, duty_u16=0) # type: ignore
        m_left1.duty_u16(0)
        m_left2.init(freq=PWM_FREQ, duty_u16=turn_speed) # type: ignore
        m_left2.duty_u16(turn_speed)
        m_right1.init(freq=PWM_FREQ, duty_u16=turn_speed) # type: ignore
        m_right1.duty_u16(turn_speed)
        m_right2.init(freq=PWM_FREQ, duty_u16=0) # type: ignore
        m_right2.duty_u16(0)


#merge doar pentru miscare frontala
#power este viteza motoarelor
#delta este puterea care se scade dintr-o parte pentru turn
#direction este directia de turn
#1 = dreapta mereu
#-1 = stanga mereu
def m_divert(power, delta,  direction):
    '''
    Works only for forward movement
    '''
    left = power
    right = power
    if direction == 1:
        right -= delta
    else:
        left -= delta

    left = int(left)
    right = int(right)

    m_speed(left, right, 1)

def m_stop():
    m_speed(0, 0, 0)

def logic_decide(left, center, right):
    # if left > 200 and center > 200 and right > 200:
    #     return 'STOP'

    if right > 250:
        return 'RIGHT'
    elif center > 150 :
        return 'CENTER'
    elif left > 250:
        return 'LEFT'
    else:
        return 'BACK'
    
def go_forward():
    glb.value(1)
    vrd.value(1)
    m_speed(REAL_SPEED, REAL_SPEED, 1)
def go_right():
    glb.value(0)
    vrd.value(1)
    mpu.read()
    head = mpu._angZ
    target = wrap(head - TURN_ANGLE)
    while mpu._angZ > target:
        mpu.read()
        if tofl0.ping() < 20:
            brake()
        m_turn(turn_speed=TURN_SPEED, direction='RIGHT')
def go_left():
    glb.value(1)
    vrd.value(0)
    mpu.read()
    head = mpu._angZ
    target = wrap(head + TURN_ANGLE)
    while mpu._angZ < target:
        if tofl0.ping() < 20:
            brake()
        mpu.read()
        m_turn(turn_speed=TURN_SPEED, direction='LEFT')
def go_back():
        m_left1.init(freq=PWM_FREQ, duty_u16=REAL_SPEED) # type: ignore
        m_left1.duty_u16(REAL_SPEED)
        m_left2.init(freq=PWM_FREQ, duty_u16=0) # type: ignore
        m_left2.duty_u16(0)
        m_right1.init(freq=PWM_FREQ, duty_u16=REAL_SPEED) # type: ignore
        m_right1.duty_u16(REAL_SPEED)
        m_right2.init(freq=PWM_FREQ, duty_u16=0) # type: ignore
        m_right2.duty_u16(0)
def turn_back():
    glb.value(0)
    vrd.value(0)
    mpu.read()
    head = mpu._angZ
    target = wrap(head + (TURN_ANGLE * 2) + 15) # +15 not enough deg
    brake()
    while mpu._angZ < target:
        mpu.read()
        m_turn(turn_speed=TURN_SPEED, direction='LEFT')

def brake():
    mpu.read()
    startTime = utime.ticks_ms()
    while(utime.ticks_diff(utime.ticks_ms(), startTime) <= BRAKE_TIME):
        mpu.read()
        go_back()
    m_stop()

def long_brake():
    mpu.read()
    startTime = utime.ticks_ms()
    while(utime.ticks_diff(utime.ticks_ms(), startTime) <= LONG_BRAKE_TIME):
        mpu.read()
        go_back()
    m_stop()

# reset procedure for each TOF device
device_0_xshut.value(0)
device_1_xshut.value(0)
device_2_xshut.value(0)
sleep(0.1)
device_0_xshut.value(1)
device_1_xshut.value(1)
device_2_xshut.value(1)

# setting up device TOF 0
print("Setting up device 0")
# keep active just the 1st sensor
device_0_xshut.value(1)
device_1_xshut.value(0)
device_2_xshut.value(0)

utime.sleep_us(TBOOT)

tofl0 = setup_tofl_device(i2c_0, 40000, 12, 8) 
tofl0.set_address(0x31)

# setting up device TOF 1
print("Setting up device 1")
# turn on the 2nd sensor

device_1_xshut.value(1)
utime.sleep_us(TBOOT)

tofl1 = setup_tofl_device(i2c_0, 40000, 12, 8)
tofl1.set_address(0x32)

# setting up device TOF 2
print("Setting up device 2")
# turn on the 3rd sensor

device_2_xshut.value(1)
utime.sleep_us(TBOOT)

tofl2 = setup_tofl_device(i2c_0, 40000, 12, 8)
tofl2.set_address(0x33)

mpu = MPU6050(i2c_1)
print("init mpu")
mpu.Initialize()
print("calibrating")
mpu.Calibrate()
print('done calibrating')

#MARK FINISH INIT AND WAIT FOR START
glb.value(0)
vrd.value(0)

# active loop 
while START.value() == 0:
    a = 0

m_speed(MOTOR_MAX, MOTOR_MAX, 1)
sleep(0.2)

glb.value(0)
vrd.value(1)

first = 'CENTER'

while True:
    mpu.read()
    centerDist = tofl0.ping()
    rightDist = tofl1.ping()
    leftDist = tofl2.ping()

    # print(logic_decide(left=leftDist, center=centerDist, right=rightDist))
    logic = logic_decide(left=leftDist, center=centerDist, right=rightDist)
    while START.value() == 0:
            nuMerge = 'ADEVARAT'
            m_stop()
    print(logic)
    while logic == 'CENTER' or first == 'CENTER':
    # while logic == 'RIGHT':
        first = 0
        while START.value() == 0:
            nuMerge = 'ADEVARAT'
            m_stop()
        mpu.read()
        centerDist = tofl0.ping()
        rightDist = tofl1.ping()
        leftDist = tofl2.ping()
        go_forward()
        logic = logic_decide(leftDist, centerDist, rightDist)
    else:
        if logic == 'RIGHT':
        # if logic == 'CENTER':
            long_brake()
            go_right()
            m_speed(MOTOR_MAX, MOTOR_MAX, 1)
            sleep(0.3)
            mpu._angZ = 0
        elif logic == 'LEFT':
            long_brake()
            go_left()
            m_speed(MOTOR_MAX, MOTOR_MAX, 1)
            sleep(0.3)
            mpu._angZ = 0
        elif logic == 'BACK':
            turn_back()
            m_stop()
        # elif logic == 'STOP':
        #     while START.value() == 1:
        #         nuMerge = 'ADEVARAT'
        #         m_stop()

    # if START.value() == 1:
    #     m_speed(speed_left=LEFT_SPEED, speed_right=RIGHT_SPEED, direction=1)
        # utime.sleep_ms(2000)
    # else:
    #     m_speed(0,0,0)
    # m_speed(LEFT_SPEED, RIGHT_SPEED, 1)
    # utime.sleep_ms(500)
    # while mpu._gyroZ != 0 :
    #     m_speed(LEFT_SPEED, RIGHT_SPEED, -1)
    #     if START.value() == 0:
    #         break
    # m_speed(LEFT_SPEED, RIGHT_SPEED, 0)
    # while True:
    #     a = 0

    # utime.sleep_ms(10000)
    # while True:
        # a = 1
    # m_turn(turn_speed=int(REAL_SPEED), direction='LEFT')
    # vrd.value(0)
    # glb.value(1)
    # while mpu._angZ < target:
    #     if START.value() == 0:
    #         break
    #     mpu.read()
    # m_speed(LEFT_SPEED, RIGHT_SPEED, 0)
    #after turning set the gyro pose to 0
    # mpu._gyroZ = 0
    # head = 0
    # vrd.value(1)
    # glb.value(0)
    # while True:
    #     a = 0
    
    # m_speed(REAL_SPEED, 1)
    # ax = mpu._gyroX
    # ay = mpu._gyroY
    # mpu.read()
    # az = mpu._gyroZ
    # if az < 0:
    #     d = -1
    # else:
    #     d = 1
    # while az < 0:
    #     mpu.read()
    #     az = mpu._gyroZ
    #     m_speed(LEFT_SPEED, RIGHT_SPEED, -1)
    # else:
    #     while az > 0:
    #         mpu.read()
    #         az = mpu._gyroZ
    #         m_speed(LEFT_SPEED, RIGHT_SPEED, -1)
    # while True:
    #     m_stop()
    
    # print(f'accX {mpu._gyroX}','\r')
    # print(f'angle z {mpu._angZ}','\r')
    # print("Distance 1: ", tofl0.ping(), "mm", "Distance 2: ", tofl1.ping(), "mm", "Distance 3: ", tofl2.ping(), "mm")