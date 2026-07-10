# ------------------------------------------
# 
# 	Project: Victors Tigers Auto 5.0.0
#	Author: Victor Dvorson
# 
# ------------------------------------------

"""
TO DO:
--------------------------------
Create loop for moving forward.
Change tracking wheels port  to the real port instead of placeholder.
Rename tracking wheel to something else

--------------------------------

Commit Log:
--------------------------------
Finished loop to turn robot with pid loop. 7/10/2026
Created position tracking loop. 7/10/2026

--------------------------------
"""

# Library imports
import time
from vex import *
import math as m

# Brain should be defined by default
brain=Brain()

# Robot configuration code
motor_FL = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
motor_BL = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
motor_FR = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
motor_BR = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
inertial_1 = Inertial(Ports.PORT7)
rotation_FB = Rotation(Ports.PORT8, False)

#x and y position of the robot in inches
x = 0
y = 0


# wait for rotation sensor to fully initialize
wait(30, MSEC)

def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

# Library imports
from vex import *

# Begin project code

def drivetrain(leftSpeed, rightSpeed):
    motor_FL.spin(FORWARD, leftSpeed, PERCENT)
    motor_BL.spin(FORWARD, leftSpeed, PERCENT)
    motor_FR.spin(FORWARD, rightSpeed, PERCENT)
    motor_BR.spin(FORWARD, rightSpeed, PERCENT)

"""
This is a threaded function
"""
def position(startingX, startingY,startingAngle, trackingwheelDiamiter):
    global x
    global y

    x = startingX
    y = startingY
    angle = startingAngle

    previousTrackingAngle = 0
    rotation_FB.set_position(0, DEGREES)

    while True:

        x += m.sin(m.radians(angle))*(rotation_FB.angle()-previousTrackingAngle)
        y += m.cos(m.radians(angle))*(rotation_FB.angle()-previousTrackingAngle)

        previousTrackingAngle = rotation_FB.angle()




"""
Function to take x, y, and a direction, True being forward and False being reverse.
x: x position inputed into the moveTo function
    floating point number
y: y position inputed into the moveTo function
    floating point number
directionBool: a boolean version of direction, only taking two inputs, True (forward) and False (reverse)

Returns a floating point angle in degrees
"""
def calculateTargetAngle(x , y, directionBool):

    #If direction is not forward, take a point in the exact oppisite direction by negating both x and y
    if directionBool == False:
        x *= -1
        y *= -1

    #Uses atan2 to find the angle
    #Note: atan2 uses inputs, (y,x) but since our initial direction (angle = 0) is the y axis, we must use (x,y)
    targetAngle = math.atan2(x,y)
    #Converts to degrees
    targetAngle = 360*targetAngle/(math.pi*2)
    return targetAngle

"""
A function to move an angle (secondAngle) to within +-180 degrees of the reference angle (referanceAngle)
This is to ensure that the robot will always take the sortest path to an angle, which can always be achived with <= 180 degree turn
referanceAngle: float
secondAngle: float

returns a floating point angle in degrees
"""
def moveAngleWithinRange(referanceAngle, secondAngle):

    """
    takes the difference between both angles: (second-referance)
    divides that by 360 and rounds to see if the diference is closest to any multiple of 360 than 0
    this is done because if the diference is too large, >180 aka closer to a multiple of 360 then
    there is the same angle but -360 -- not actually changing the angle -- which is closer to the referance angle.
    multiplies by 360 to get it back into an angle again
    subtracts from second angle to get it to the closest value to the referance angle
    """
    return secondAngle-360*round((secondAngle-referanceAngle)/360)



"""
Gets the MTD which is a:

    A modified target angle that accounts for the ending angle inputted into this function.
    Basically it takes the ending angle, and the angle directly to the point and uses a formula to get the angle it needs to drive at.
    This will make it drive a curved path if the ending angle is not already the same as the angle directly to the point.
    The name legit stands for 'modified target angle'

Then it moves it to within 180 degrees of the robotAngle to ensure distance traveled is minimum.

targetAngle: Floating point angle in degrees
robotAngle: Floating point angle in degrees

returns a floating point angle in degrees
"""
def getMTD(targetAngle, endAngle, robotAngle):
    #calculates MTD
    MTD = 2*targetAngle-endAngle

    #Moves MTD
    MTD =moveAngleWithinRange(robotAngle,MTD)

    return MTD

"""
A function to linearize the motor speeds so an input of 50% will actually be 50% of the max speed, not 50% of the max voltage which is what the motors actually take in.
leftMS: the left , unlinearized, motor speed as a percentage from -100 to 100
rightMS: the right, unlinearized, motor speed as a percentage from -100 to 100
Returns a two digit tuple of the left and right motor speeds, linearized, as percentages from -100 to 100
"""

a = 8 #quadratic
b = 90 #linear
c = 2 #verticalTranslation
d = 1 #deadzone
p = 3 #power

def linearize(leftMS, rightMS):
    global a,b,c,d,p   
    leftMS = inputCurve(leftMS, a,b,c,d,p)
    rightMS = inputCurve(rightMS, a,b,c,d,p)

    return (leftMS, rightMS)


#Uses inputCurveRaw but takes into account a deadzone.
def inputCurve(input, a, b, c, d, p):

    if(input >= d/100):
        #Modified input
        y = inputCurveRaw((1+d/100)*(input-d/100), a,b,p) + c/100

    elif (input <= -d/100):
        input *= -1
        y = -inputCurveRaw((1+d/100)*(input-d/100), a,b,p) - c/100

    else:
        y = 0

    return y

#Linearization function that doesnt account for deadzone
def inputCurveRaw(input, a, b, p):
    y = (a/100)*pow(input,p) +  (b/100)*input
    return y

"""
moveTo is the parent function to move the robot. 
x: defines an x value which is however many inches to the right (positive) or left (negative) you want the robot to move
    floating point number
y: defines an y value which is however many inches forward (positive) or backward (negative) you want the robot to move
    floating point number
endAngle: At what angle from angle = 0 (or just from the angle the robots heading makes with the y axis) do you want to robot to finish at
    Floating point degree value from (-180,180]
direction: Direction the robot should travel in
    String value either stating "forward" or "reverse"

"""
def moveTo(x, y, endAngle, direction):
    if direction == "reverse":
        directionBool = False
    else:
        directionBool = True

    
    #P and D components for PID loop tunring
    pTurningComponent = 1
    dTurningComponent = 1
        
    #First PID loop to turn the robot to the position
    #Condition checked at the end
    while True:

        #Defines the angle as a heading in degrees
        robotAngle = inertial_1.heading(DEGREES)
        #Defines the angle change rate as a rate in degrees per second
        robotAngleChangeRate = inertial_1.gyro_rate(AxisType.XAXIS, VelocityUnits.DPS)

        #Raw straight line target angle to point
        targetAngle = calculateTargetAngle(x,y,directionBool)

        #Gets modifed target angle, explaied in detail above the method.
        MTD = getMTD(targetAngle, endAngle, robotAngle)

        leftSpeedRaw = pTurningComponent*(MTD-robotAngle) #Proportional component
        leftSpeedRaw =  dTurningComponent*robotAngleChangeRate  #Derivative component
        rightSpeedRaw = -leftSpeedRaw

        linearizedSpeeds = linearize(leftSpeedRaw, rightSpeedRaw)
        drivetrain(linearizedSpeeds[0], linearizedSpeeds[1])
        

def pre_autonomous():
    # actions to do when the program starts
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")
    wait(1, SECONDS)

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # hi

def user_control():
    brain.screen.clear_screen()
    # place driver control in this while loop
    while True:
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()
