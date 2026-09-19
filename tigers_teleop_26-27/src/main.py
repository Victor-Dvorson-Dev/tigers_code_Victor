#region VEXcode Generated Robot Configuration
from vex import *
import random as urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)

motorFL = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)   # front-left
motorFR = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)  # front-right
motorBL = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)   # back-left
motorBR = Motor(Ports.PORT3,  GearSetting.RATIO_6_1, False)  # back-right
motorML = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)   # mid-left
motorMR = Motor(Ports.PORT2,  GearSetting.RATIO_6_1, False)  # mid-right

elevationL = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
elevationR = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)

rotationMotor = Motor(Ports.PORT11,  GearSetting.RATIO_6_1, False) 

digital_out_a = DigitalOut(brain.three_wire_port.a)
digital_out_b = DigitalOut(brain.three_wire_port.b)
inertial_21 = Inertial(Ports.PORT7)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()


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

# ------------------------------------------
# 
# 	Project:
#	Author:
#	Created:
#	Configuration:
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code

#VARIABLES

#The drivetrain will turn at this % velocity until changed by pressing x
startingTurnVelocity = 40 #default is 
otherTurnVelocity = 50

a = 38 #quadratic
b = 60 #linear
c = 2 #verticalTranslation
d = 2 #deadzone
p = 3 #power

#a fuction that allows drivers to change the constants from controller
def increaseQuadratic():
    pass

def decreaseQuadratic():
    pass


#Basic input curve without any translations
def inputCurveRaw(input, a, b, p):
    y = (a/100)*pow(input,p) +  (b/100)*input
    return y


#Modified input curve using deadzones and the velocity to initially move the motor
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


def driveFunction():     #Threaded function to drive motors based on controller input

    #Defines how fast robot should turn in %
    turnVelocity = startingTurnVelocity 

    toggle = False
    toggle2 = False
    

    slow = False


    maxVelocity = 100


    #changes how much of the turning velocity is kept at high speeds
    turnSpeedMult = 1.2
    
    #Creates telemetry loop variable
    telemLoop = 0

    #While loop because the function is threaded
    while (True):
        #print("\033[2J") #Clears console
        #tipPrevention()


        #Changes the curve to more linear or more exponential
        #controller_1.buttonUp.pressed()
        

        #if statement to allow changes to turn velocity while driving robot

        if (controller_1.buttonX.pressing() and toggle2 == False):
            if (slow == False):
                slow = True
                turnVelocity = startingTurnVelocity

                #Telemetry for drivers
                controller_1.screen.clear_row(1)
                controller_1.screen.set_cursor(1, 1)
                controller_1.screen.print("turn velocity " + str(startingTurnVelocity)) 
                print("turn velocity " + str(startingTurnVelocity))


            elif (slow == True):
                slow = False
                    
                turnVelocity = otherTurnVelocity

                #Telemetry for drivers
                controller_1.screen.clear_row(1)
                controller_1.screen.set_cursor(1, 1)
                controller_1.screen.print("turn velocity " + str(otherTurnVelocity))
                print("turn velocity "+ str(otherTurnVelocity))
                

            toggle2 = True

        elif ( controller_1.buttonX.pressing() == False and toggle2 == True):
            toggle2 = False


        #THIS TOGGLES THE CALIBRATION TO CHECK IF TOGGLED READ THE BOOLEAN VALUE OF "cal_on"
        '''
        if ( controller_1.buttonY.pressing() and toggle == False):
            
            if (cal_on == True):
                cal_on = False

            elif (cal_on == False):
                cal_on = True
            
            toggle = True

        elif ( controller_1.buttonY.pressing() == False and toggle == True):
            toggle = False
        '''

       


        #calculates left and right drivetrain velocity based on inputs and turn velocity

        forwardV = inputCurve(controller_1.axis3.position()/100,a,b,c,d,p)
        leftV = 100*forwardV + turnVelocity * controller_1.axis1.position()/100
        rightV = 100*forwardV - turnVelocity * controller_1.axis1.position() /100

        #note: axis3 is the forward axis and axis1 is the turning axis.


        #Checks if drivetrain velocity for either side is bigger than 100. If it is it adds it subtracts it from the other side to keep the robot turning the same speed.
        #better explained on july 8 in the engineering notebook

        if (leftV > maxVelocity):
            rightV -= turnSpeedMult*(leftV- maxVelocity)
            leftV = maxVelocity

        elif (leftV < -maxVelocity):
            rightV -= turnSpeedMult*(leftV + maxVelocity)
            leftV = -maxVelocity
                

        if (rightV > maxVelocity):
            leftV -= turnSpeedMult*(rightV- maxVelocity)
            rightV = maxVelocity

        elif (rightV < -maxVelocity):
            leftV -= turnSpeedMult*(rightV + maxVelocity)
            rightV = -maxVelocity


        #for telemetry
        if (telemLoop  >= 10): 

            print("|" + str(round(leftV)) + "|-|" + str(round(rightV)) + "|   " + str(controller_1.axis1.position()) + "|" + str(controller_1.axis3.position()) + "   "  +  "   Turn velocity:" + str(turnVelocity))


            #resets telemetry loop
            telemLoop = 0


        #sets speed for the drivetrain
        setSpeed(leftV,rightV)
        #setTorque(0.7*leftV+0.3,0.7*rightV+0.3)

        #increments telemetry loop by one
        telemLoop += 1

        #prevents the loop from taking up all the brains resources.
        wait(15, MSEC)


""" Unused functions to chenge the curve constants.
def changeCurveLin(a,b):
    a -= 1
    b += 1

def changeCurveExp(a,b): 
    a += 1
    b -= 1
"""

#sets the speed of the drivetrain motors based on the left and right velocity.
def setSpeed(leftV,rightV):

    #spins all the motors at the correct speed. negative values spin the motor in reverse,
    #so the signs calculated in driveFunction carry through unchanged.
    motorFL.spin(FORWARD, leftV, PERCENT)
    motorML.spin(FORWARD, leftV, PERCENT)
    motorBL.spin(FORWARD, leftV, PERCENT)
    motorFR.spin(FORWARD, rightV, PERCENT)
    motorMR.spin(FORWARD, rightV, PERCENT)
    motorBR.spin(FORWARD, rightV, PERCENT)

def setTorque(leftT,rightT):
    motorFL.set_max_torque(leftT, PERCENT)
    motorFR.set_max_torque(rightT, PERCENT)
    motorML.set_max_torque(leftT, PERCENT)
    motorMR.set_max_torque(rightT, PERCENT)
    motorBL.set_max_torque(leftT, PERCENT)
    motorBR.set_max_torque(rightT, PERCENT)



def elevationAndClaw():
    global clawPosition
    rotationMotor.set_stopping(HOLD)

    clawToggle = False

    position = 0
    elevationL.set_position(0, DEGREES)
    elevationR.set_position(0, DEGREES)

    elevationL.set_stopping(HOLD)
    elevationR.set_stopping(HOLD)

    stop = False
    
    while True:

        #Elevation control
        prevTime = brain.timer.time(MSEC)

        if (controller_1.buttonR1.pressing()):
            elevationL.spin(FORWARD)
            elevationR.spin(FORWARD)
            elevationL.set_velocity(100, PERCENT)
            elevationR.set_velocity(100, PERCENT)
            print(elevationL.position(DEGREES))
            stop = False

        elif (controller_1.buttonR2.pressing()):
            elevationL.spin(FORWARD)
            elevationR.spin(FORWARD)
            elevationL.set_velocity(-100, PERCENT)
            elevationR.set_velocity(-100, PERCENT)
            stop = False

        elif stop == False:
            elevationL.stop()
            elevationR.stop()
            stop = True

        #claw 
        if controller_1.buttonL1.pressing():
            if clawToggle == False:
                if digital_out_a.value() == False:
                    digital_out_a.set(True)
                else:
                    digital_out_a.set(False)
                clawToggle = True
        elif clawToggle == True:
            clawToggle = False

        wait(20, MSEC)

    
"""     
def tipPrevention():
    print(inertial_13.acceleration(AxisType.YAXIS))
    
    #positive bound for acceleration before it throttles the torque
    if inertial_13.acceleration(AxisType.YAXIS) > 0.3:
        
        maxTorque = 100-abs(40*inertial_13.acceleration(AxisType.YAXIS))

        motor_FL.set_max_torque(maxTorque, PERCENT)
        motor_FR.set_max_torque(maxTorque, PERCENT)
        motor_BL.set_max_torque(maxTorque, PERCENT)
        motor_BR.set_max_torque(maxTorque, PERCENT)
        motor_ML.set_max_torque(maxTorque, PERCENT)
        motor_MR.set_max_torque(maxTorque, PERCENT)


    #negative bound for acceleration before it throttles the torque
    elif (inertial_13.acceleration(AxisType.YAXIS) < -0.15):

        maxTorque = 100-abs(100*inertial_13.acceleration(AxisType.YAXIS))

        motor_FL.set_max_torque(maxTorque, PERCENT)
        motor_FR.set_max_torque(maxTorque, PERCENT)
        motor_BL.set_max_torque(maxTorque, PERCENT)
        motor_BR.set_max_torque(maxTorque, PERCENT)
        motor_ML.set_max_torque(maxTorque, PERCENT)
        motor_MR.set_max_torque(maxTorque, PERCENT)


    else:
        motor_FL.set_max_torque(100, PERCENT)
        motor_FR.set_max_torque(100, PERCENT)
        motor_BL.set_max_torque(100, PERCENT)
        motor_BR.set_max_torque(100, PERCENT)
        motor_ML.set_max_torque(100, PERCENT)
        motor_MR.set_max_torque(100, PERCENT)  
"""
"""
threaded function
"""
def MacroClawUp():
    #Macro to move the claw up to the top position
    toggle = False
    while (True):
        if controller_1.buttonL2.pressing() and toggle == False:
            print("Macro claw up")
            elevationL.spin(FORWARD)
            elevationR.spin(FORWARD)
            elevationL.set_velocity(100, PERCENT)
            elevationR.set_velocity(100, PERCENT)

            while (elevationL.position(DEGREES) < 280):
                wait(20, MSEC)

            elevationL.stop()
            elevationR.stop()
            toggle = True
        elif (controller_1.buttonL2.pressing() == False and toggle == True):
            toggle = False

        wait(20, MSEC)
    

def CIO():

    #prints the temeprature of the warmest motor
    controller_1.screen.print(max(motorFL.temperature(PERCENT),motorFR.temperature(PERCENT),motorML.temperature(PERCENT),motorMR.temperature(PERCENT),motorBL.temperature(PERCENT),motorBR.temperature(PERCENT)))



def pre_autonomous():
    # actions to do when the program starts
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")
    wait(1, SECONDS)
    elevationL.set_position(0, DEGREES)


def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    # place driver control in this while loop
    
    driveThread = Thread(driveFunction)
    elevationAndClawThread = Thread(elevationAndClaw)
    macroClawUpThread = Thread(MacroClawUp)
    #armThread = Thread(arm_descore)
    #intakeThread = Thread(intake)

    while True:
        wait(20, MSEC)
        

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()




