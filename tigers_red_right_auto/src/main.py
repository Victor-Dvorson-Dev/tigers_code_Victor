# ------------------------------------------
# 
# 	Project: Victors Tigers Auto 5.0.0
#	Author: Victor Dvorson
# 
# ------------------------------------------

"""
TO DO:
--------------------------------
Test arc moving with new system
make path more realiable

--------------------------------

Commit Log:
--------------------------------

--------------------------------
"""

# Library imports
import time
from vex import *
import math as m

# Brain should be defined by default
brain=Brain()

# Robot configuration code
motorFL = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)   # front-left
motorFR = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)  # front-right
motorBL = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)   # back-left
motorBR = Motor(Ports.PORT3,  GearSetting.RATIO_6_1, False)  # back-right
motorML = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)   # mid-left
motorMR = Motor(Ports.PORT2,  GearSetting.RATIO_6_1, False)  # mid-right

elevationL = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
elevationR = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)

rotationMotor = Motor(Ports.PORT11,  GearSetting.RATIO_6_1, False) 

digital_out_a = DigitalOut(brain.three_wire_port.a)
digital_out_b = DigitalOut(brain.three_wire_port.b)
trackingWheelVertL= Rotation(Ports.PORT4)
inertial_1 = Inertial(Ports.PORT7)


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

drivetrainWidth = 0

# Begin project code
def drivetrain(leftSpeed, rightSpeed):
    motorFL.spin(FORWARD, leftSpeed, PERCENT)
    motorML.spin(FORWARD, leftSpeed, PERCENT)
    motorBL.spin(FORWARD, leftSpeed, PERCENT)
    motorFR.spin(FORWARD, rightSpeed, PERCENT)
    motorMR.spin(FORWARD, rightSpeed, PERCENT)
    motorBR.spin(FORWARD, rightSpeed, PERCENT)

"""
This is a threaded function
"""
def position(startingX, startingY, trackingwheelDiamiter):
    global x
    global y

    x = startingX
    y = startingY

    previousTrackingAngle = 0
    #set_position resets what .position() reports, NOT what .angle() reports, so this only lines up
    #with previousTrackingAngle = 0 because the loop below reads .position().
    trackingWheelVertL.set_position(0, DEGREES)

    while True:

        angle = inertial_1.heading(DEGREES)

    
        currentTrackingAngle = trackingWheelVertL.position(DEGREES)
        deltaTrackingAngle = currentTrackingAngle-previousTrackingAngle

        x -= m.sin(m.radians(angle))*deltaTrackingAngle/360*trackingwheelDiamiter*m.pi
        y -= m.cos(m.radians(angle))*deltaTrackingAngle/360*trackingwheelDiamiter*m.pi

        previousTrackingAngle = currentTrackingAngle
        if (mode == 1):
            print("x: ", round(x,2), "y: ", round(y,2), "angle: ", round(angle,2))

        wait(10, MSEC)
        



"""
Function to take x, y, and a direction, True being forward and False being reverse.
x: x position inputed into the moveTo function
    floating point number
y: y position inputed into the moveTo function
    floating point number
directionBool: a boolean version of direction, only taking two inputs, True (forward) and False (reverse)

Returns a floating point angle in degrees
"""
def calculateTargetAngle(targX, targY, directionBool):
    global x
    global y

    #Vector from where the robot actually is to the target
    deltaX = targX - x
    deltaY = targY - y

    #If direction is not forward, take a point in the exact oppisite direction by negating the vector.
    #This must happen AFTER subtracting the robot position -- negating targX/targY first only gives
    #the same answer when the robot sits at the origin.
    if directionBool == False:
        deltaX *= -1
        deltaY *= -1

    #Uses atan2 to find the angle
    #Note: atan2 uses inputs, (y,x) but since our initial direction (angle = 0) is the y axis, we must use (x,y)
    targetAngle = m.atan2(deltaX,deltaY)
    #Converts to degrees
    targetAngle = 360*targetAngle/(m.pi*2)
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
Gets the MTD which is a modified target angle that accounts for the ending angle inputted into this function.

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
Converts the straight line distance to the target into the distance the robot 
will take if it travels in an arc.
Returns float of distance in inches.
"""
def convertToArcDistance(linearDistance, MTD, endAngle):
    arcAngle = moveAngleWithinRange(0, MTD-endAngle)

    #Straight shot. Also guards the 0/0 in the formula below (whose limit is exactly linearDistance).
    if abs(arcAngle) < 1e-6:
        return linearDistance

    #Wrapping caps |arcAngle| at 180, so sin(arcAngle/2) can no longer be zero here.
    return linearDistance*m.radians(arcAngle)/(2*m.sin(m.radians(arcAngle/2)))


"""
Speed scale for each side so each side covers the distance its own wheel travels on the arc.

arcDistance: signed arc length to the target in inches, negative for reverse
MTD: modified target direction in degrees
endAngle: heading the robot should finish at in degrees
width: drivetrain track width in inches, wheel to wheel

returns (leftScale, rightScale)
"""
def getArcModifiers(arcDistance, MTD, endAngle, width, arcLockRadius = 1):
    if arcDistance < arcLockRadius:
        return (1,1)
    #makes math a little simpler
    arcAngle = moveAngleWithinRange(0, MTD-endAngle)
    
    #arcDistance is already negative for reverse, which swaps which side travels further.
    halfTrack = width*m.radians(arcAngle)/(2*arcDistance)
    leftScale = 1-halfTrack
    rightScale = 1+halfTrack

    #Makes sure maximum modifier is 1.
    """
    biggest = max(abs(leftScale), abs(rightScale))
    if biggest > 1:
        leftScale /= biggest
        rightScale /= biggest
    """
    return (leftScale, rightScale)


"""
A function to linearize the motor speeds so an input of 50% will actually be 50% of the max speed, not 50% of the max voltage which is what the motors actually take in.
leftMS: the left , unlinearized, motor speed as a percentage from -100 to 100
rightMS: the right, unlinearized, motor speed as a percentage from -100 to 100
Returns a two digit tuple of the left and right motor speeds, linearized, as percentages from -100 to 100
"""

a = 0#quadratic
b = 99.2 #linear
c = 0.8 #verticalTranslation
d = 0.4 #deadzone
p = 1#power

def linearize(leftMS, rightMS):
    global a,b,c,d,p   
    leftMS = inputCurve(leftMS, a,b,c,d,p)
    rightMS = inputCurve(rightMS, a,b,c,d,p)

    return (leftMS, rightMS)


#Uses inputCurveRaw but takes into account a deadzone.
def inputCurve(input, a, b, c, d, p):
    input /= 100

    if(input >= d/100):
        #Modified input
        y = inputCurveRaw((1+d/100)*(input-d/100), a,b,p) + c/100

    elif (input <= -d/100):
        input *= -1
        y = -inputCurveRaw((1+d/100)*(input-d/100), a,b,p) - c/100

    else:
        y = 0

    return y*100

#Linearization function that doesnt account for deadzone
def inputCurveRaw(input, a, b, p):
    y = (a/100)*pow(input,p) +  (b/100)*input
    return y


def moveTo(targX, targY, endAngle, direction, dontTurn = False):
    global x
    global y

    if direction == "reverse":
        directionBool = False
    elif direction == "forward":
        directionBool = True
    else:
        print("Invalid direction")
        return

    
    #P and D components for PID loop tunring
    pTurningComponent = 0.6

    dTurningComponent = 0.05 #Dont know why this is so low but it works

    #P and D components for DRIVING loop tunring
    pDriveTurningComponent = 0.4

    dDriveTurningComponent = 0

    #How close to MTD (in degrees) counts as done. Recomended to stay wider than inverse of p component.
    turnExitWindow = 1.5

    #P and D components for PID loop moving forward
    pMoveComponent = 2.2
    dMoveComponent = 0.00

    #Must stay wider than deadzone/pMoveComponent (0.28in) or the curve zeroes the motors first.
    moveExitWindow = 0.4
    moveSettleCount = 3

    #Inside this radius (inches) stop re-aiming at the point and just hold endAngle (ONLY APLIES TO TURNING FUNCTION, DOES NOT APPLY TO ARC LOCKING).
    headingLockRadius = 0.5

    arcLockingRadius = 2

    #timeout
    moveTimeout = 4000 #msec

    #Period of both loops and time between telemetry points.
    loopPeriod = 10 #msec

    telemetryCount = 0

    #turning loop

    timeStart = brain.timer.time(MSEC)
    if dontTurn == False:
        while True:

            #Defines the angle as a heading in degrees
            robotAngle = inertial_1.heading(DEGREES)
            #Defines the angle change rate as a rate in degrees per second
            robotAngleChangeRate = inertial_1.gyro_rate(AxisType.ZAXIS, VelocityUnits.DPS)

            #Raw straight line target angle to point
            targetAngle = calculateTargetAngle(targX,targY,directionBool)

            #Gets modifed target angle, explaied in detail above the method.
            MTD = getMTD(targetAngle, endAngle, robotAngle)

            leftSpeedRaw = pTurningComponent*(MTD-robotAngle) #Proportional component
            leftSpeedRaw -= pTurningComponent*dTurningComponent*robotAngleChangeRate  #Derivative component

            if MTD-robotAngle > 0:
                if leftSpeedRaw < 1:
                    leftSpeedRaw = 1
            else:
                if leftSpeedRaw > -1:
                    leftSpeedRaw = -1

            rightSpeedRaw = -leftSpeedRaw

            linearizedSpeeds = linearize(leftSpeedRaw, rightSpeedRaw)
            drivetrain(linearizedSpeeds[0], linearizedSpeeds[1])
            if telemetryCount % 10 == 0:
                print("speed ", round(linearizedSpeeds[0],1), "left to turn:", round(MTD-robotAngle,1), "p component:", round(pTurningComponent*(MTD-robotAngle),1), "d component:", round(-pTurningComponent*dTurningComponent*robotAngleChangeRate,1))

            if robotAngle <= MTD+turnExitWindow and robotAngle >= MTD-turnExitWindow:
                print("Done Turning Time elapsed: ", brain.timer.time(MSEC)-timeStart)
                break

            wait(loopPeriod, MSEC)
                
            telemetryCount += 1
    drivetrain(0,0)

    
    #Second PID loop to drive the robot along the arc onto the point, holding the heading with the
    #same turning PD so the path stays curved.
    #Condition checked at the end

    wait(100, MSEC) #Short settle so the gyro rate reads zero before anything starts steering off it.
    settleCounter = 0
    previousArcDistance = None
    moveStartTime = brain.timer.time(MSEC)

    lastMoveSpeed = 0
    
    while True:

        #Defines the angle as a heading in degrees
        robotAngle = inertial_1.heading(DEGREES)
        #Defines the angle change rate as a rate in degrees per second
        robotAngleChangeRate = inertial_1.gyro_rate(AxisType.ZAXIS, VelocityUnits.DPS)

        #Straight line distance left to cover
        linearDistance = m.sqrt(pow(targX-x,2)+pow(targY-y,2))

        #Raw straight line target angle to point
        targetAngle = calculateTargetAngle(targX,targY,directionBool)

        if linearDistance > headingLockRadius:
            #Gets modifed target angle, explaied in detail above the method.
            MTD = getMTD(targetAngle, endAngle, robotAngle)
            #Converts linear distance to arc
            arcDistance = convertToArcDistance(linearDistance, MTD, endAngle)
        else:
            MTD = moveAngleWithinRange(robotAngle, endAngle)
            #if within a small radius of target uses linear distance intead of arc.
            arcDistance = linearDistance


       
        
        if directionBool == False:
            arcDistance = -arcDistance
            linearDistance = -linearDistance

        #Straight moves return (1,1).  important: fix this bug later!
        arcScales = getArcModifiers(arcDistance, robotAngle, endAngle, drivetrainWidth, arcLockingRadius)
        leftScale = arcScales[0]
        rightScale = arcScales[1]

        turnSpeed = pDriveTurningComponent*(MTD-robotAngle) #Proportional component turning
        turnSpeed -= pDriveTurningComponent*dDriveTurningComponent*robotAngleChangeRate  #Derivative component turning

        moveSpeed = pMoveComponent*linearDistance #Proportional component moving
        """
        if previousArcDistance != None:
            #arc distance is just the dintance to target but accounting for the curved path.
            moveSpeed += pMoveComponent*dMoveComponent*(arcDistance-previousArcDistance)/(loopPeriod/1000)  #Derivative component moving
        previousArcDistance = arcDistance
        """

        turnSpeed = max(-100, min(100, turnSpeed))
        moveHeadroom = 100-abs(turnSpeed)
        moveSpeed = max(-moveHeadroom, min(moveHeadroom, moveSpeed))

        if moveSpeed > lastMoveSpeed+1/max(arcScales):
            moveSpeed = lastMoveSpeed+1/max(arcScales)
        lastMoveSpeed = moveSpeed

        #The scales carry the arc, so turnSpeed only corrects the leftover heading error.
        leftSpeedRaw = moveSpeed*leftScale+turnSpeed
        rightSpeedRaw = moveSpeed*rightScale-turnSpeed

        linearizedSpeeds = linearize(leftSpeedRaw, rightSpeedRaw)
        drivetrain(linearizedSpeeds[0], linearizedSpeeds[1])
        if telemetryCount % 10 == 0:
            print("MTD: ", round(MTD,1), " lag: ", round(MTD-robotAngle,1), " dist: ", round(linearDistance,2), " scales: ", round(leftScale,3), round(rightScale,3), " speeds: ", round(linearizedSpeeds[0],1), round(linearizedSpeeds[1],1))

        #Checks to make sure robot is within the exit windows
        if abs(linearDistance) <= moveExitWindow and robotAngle <= MTD+turnExitWindow and robotAngle >= MTD-turnExitWindow:
            settleCounter += 1
        else:
            settleCounter = 0

        #small settle
        if settleCounter >= moveSettleCount:
            print("Done Moving")
            break

        if brain.timer.time(MSEC)-moveStartTime > moveTimeout:
            print("Move timed out")
            break

        wait(loopPeriod, MSEC)
        telemetryCount += 1
        
    drivetrain(0,0)

def elevationTo(angle):
    elevationL.spin_to_position(angle, DEGREES, 100, PERCENT, False)
    elevationR.spin_to_position(angle, DEGREES, 100, PERCENT, False)

def rotateTo(angle):
    rotationMotor.spin_to_position(angle, DEGREES, 100, PERCENT, False)


mode = 0 #1 = calibrating pos

def pre_autonomous():
    # actions to do when the program starts
    brain.screen.clear_screen()

    if not inertial_1.installed():
        brain.screen.next_row()
        brain.screen.print("INERTIAL NOT FOUND")
        print("INERTIAL NOT FOUND on PORT7")

    inertial_1.calibrate()

    calibrateTimeout = 4000 #msec
    calibrateStartTime = brain.timer.time(MSEC)
    while inertial_1.is_calibrating():
        if brain.timer.time(MSEC)-calibrateStartTime > calibrateTimeout:
            brain.screen.next_row()
            brain.screen.print("CALIBRATION TIMED OUT")
            print("Inertial calibration timed out")
            break
        wait(50, MSEC)

    if mode !=1:
        inertial_1.set_heading(16.7, DEGREES)
    else:
        inertial_1.set_heading(0, DEGREES)

    brain.screen.clear_screen()
    brain.screen.print("pre auton code")
    if mode != 1:
        motorFL.set_stopping(HOLD)
        motorFR.set_stopping(HOLD)
        motorBL.set_stopping(HOLD)
        motorBR.set_stopping(HOLD)
        motorML.set_stopping(HOLD)
        motorMR.set_stopping(HOLD)

    rotationMotor.set_stopping(HOLD)

    elevationL.set_stopping(HOLD)
    elevationR.set_stopping(HOLD)

    rotationMotor.set_position(0, DEGREES)
    elevationL.set_position(0, DEGREES)
    elevationR.set_position(0, DEGREES)

    #Short settle so the gyro rate reads zero before anything starts steering off it.
    wait(100, MSEC)

def autonomous():
    global drivetrainWidth
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

    #width of the drivetrain in inches (wheel to wheel)
    drivetrainWidth = 12.8
    threadPosition = Thread(position, (0, 0, 2.0))

    if mode != 1:
        
        """
        moveTo(0,36,0, "forward")
        moveTo(24,36,135, "forward")
        moveTo(0,36,-90, "forward")
        """
        
        
        rotationMotor.spin(REVERSE, 100, PERCENT)
        drivetrain(-20,40)
        wait(80, MSEC)
        rotationMotor.stop()
        rotateTo(-130)
        drivetrain(-80,-40)
        wait(400, MSEC)
        drivetrain(80,80)
        wait(200, MSEC)
        drivetrain(-80,-80)
        wait(400, MSEC)
        drivetrain(80,80)
        wait(100, MSEC)

        elevationTo(700)
        moveTo(-10.0,10,-85, "forward", True)
        elevationTo(250)
        wait(500, MSEC)
        digital_out_a.set(True)
        wait(200, MSEC)
        drivetrain(-100,-100)
        elevationTo(230)
        wait(200, MSEC)
        drivetrain(0,0)
        wait(200, MSEC)
        moveTo(21,-3.6, 140, "forward")
        digital_out_a.set(False)
        wait(100, MSEC)
        elevationTo(1600)
        moveTo(24,2.5,20, "forward")
        elevationTo(1000)
        wait(500, MSEC)
        digital_out_a.set(True)
        drivetrain(-100,-100)
        wait(100, MSEC)
        drivetrain(0,0)
         
        
        """
        moveTo(-2, 5, 20, "forward")
        rotationMotor.spin_for(FORWARD, 90, DEGREES)
        wait(1000, MSEC)
        print(inertial_1.heading(DEGREES))

        moveTo(5, 0.1, 0, "reverse")
        moveTo(5, 1, 0, "forward", True)
        moveTo(5, 0.1, 0, "forward", True)
        """
        
    
    

def user_control():
    brain.screen.clear_screen()
    # place driver control in this while loop
    while True:
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()
#BENCH TESTING ONLY -- remove before a match, Competition above already runs this.
autonomous()