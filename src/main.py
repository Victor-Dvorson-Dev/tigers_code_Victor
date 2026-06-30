#region VEXcode Generated Robot Configuration
from vex import *
import random as urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
motor_FL = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
motor_BR = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
motor_FR = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)
motor_BL = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
digital_out_a = DigitalOut(brain.three_wire_port.a)
digital_out_b = DigitalOut(brain.three_wire_port.b)
inertial_21 = Inertial(Ports.PORT21)


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
startingTurnVelocity = 50 #default is 40
otherTurnVelocity = 65

a = 8 #quadratic
b = 90 #linear
c = 2 #verticalTranslation
d = 1 #deadzone
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

"""
function to flip the claw. Doesnt take any input and optionally returns the position of the claw after flipping it.
You can also access the position of the claw by using the global variable clawPosition. 0 = starting position, 1 = flipped position
"""
clawPosition = 0
def flip_claw():
    global clawPosition
    if digital_out_b.value() == False:
        digital_out_b.set(True)
        clawPosition = 1
    else:
        digital_out_b.set(False)
        clawPosition = 0

    #so kawaii :3 <3
    brain.screen.clear_line(1)
    brain.screen.set_cursor(1, 1)
    brain.screen.print("so kawaii :3 <3 "+str(clawPosition))

    return clawPosition

def scoring():

    flipToggle = False
    clawToggle = False



    while True:
        if controller_1.buttonR1.pressing():
            if not flipToggle:
                flip_claw()
                flipToggle = True
        elif (flipToggle):
            flipToggle = False
        
        #Opens an closes claw
        if controller_1.buttonR2.pressing():
            if clawToggle == False:
                if digital_out_a.value() == False:
                    digital_out_a.set(True)
                else:
                    digital_out_a.set(False)
                clawToggle = True
        elif clawToggle == True:
            clawToggle = False
            

def driveFunction():     #Threaded function to drive motors based on controller input

    #Defines how fast robot should turn in %
    turnVelocity = startingTurnVelocity 

    toggle = False
    toggle2 = False
    

    slow = False


    maxVelocity = 100


    #changes how much of the turning velocity is kept at high speeds
    turnSpeedMult = 0.8
    
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
            if (slow == True):
                slow = False
                turnVelocity = startingTurnVelocity

                #Telemetry for drivers
                controller_1.screen.clear_row(1)
                controller_1.screen.set_cursor(1, 1)
                controller_1.screen.print("turn velocity " + str(startingTurnVelocity)) 
                print("turn velocity " + str(startingTurnVelocity))


            elif (slow == False):
                slow = True
                    
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

"""
def changeCurveLin(a,b):
    a -= 1
    b += 1

def changeCurveExp(a,b): 
    a += 1
    b -= 1
"""
    
def setSpeed(leftV,rightV):

    #sets the velocities of all the motors to the correct amount
    motor_FL.set_velocity(leftV, PERCENT)
    motor_FR.set_velocity(rightV, PERCENT)
    motor_BL.set_velocity(leftV, PERCENT)
    motor_BR.set_velocity(rightV, PERCENT)

def setTorque(leftT,rightT):
    motor_FL.set_max_torque(leftT, PERCENT)
    motor_FR.set_max_torque(rightT, PERCENT)
    motor_BL.set_max_torque(leftT, PERCENT)
    motor_BR.set_max_torque(rightT, PERCENT)

"""
def intake():

    motor_10.set_velocity(100, PERCENT)
    motor_11.set_velocity(100, PERCENT)

    while(True):
        if controller_1.buttonR1.pressing():
            motor_10.spin(FORWARD)
            motor_11.spin(FORWARD)
            digital_out_d.set(False)
            motor_10.set_velocity(100, PERCENT)
            motor_11.set_velocity(100, PERCENT)
        
        elif controller_1.buttonA.pressing():
            motor_10.spin(FORWARD)
            motor_11.spin(FORWARD)
            digital_out_d.set(True)
            motor_10.set_velocity(60, PERCENT)
            motor_11.set_velocity(100, PERCENT)

        elif controller_1.buttonL1.pressing():
            motor_10.stop()
            motor_11.spin(FORWARD)
            digital_out_d.set(False)
            motor_10.set_velocity(100, PERCENT)
            motor_11.set_velocity(100, PERCENT)

        elif controller_1.buttonL2.pressing():
            motor_10.spin(REVERSE)
            motor_11.spin(REVERSE)
            digital_out_d.set(False)
            motor_10.set_velocity(-100, PERCENT)
            motor_11.set_velocity(-80, PERCENT)

        else:
            motor_10.stop()
            motor_11.stop()
"""       
def tipPrevention():
    """print(inertial_13.acceleration(AxisType.YAXIS))
    
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



def CIO():

    #prints the temeprature of the warmest motor 
    controller_1.screen.print(max(motor_FL.temperature(PERCENT),motor_FR.temperature(PERCENT),motor_BL.temperature(PERCENT),motor_BR.temperature(PERCENT)))    



def pre_autonomous():
    # actions to do when the program starts
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")
    wait(1, SECONDS)
    motor_FL.spin(FORWARD)
    motor_FR.spin(FORWARD)
    motor_BL.spin(FORWARD)
    motor_BR.spin(FORWARD)
    

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    # place driver control in this while loop
    
    thread = Thread(driveFunction)
    #thread = Thread(arm_descore)
    #thread = Thread(intake)

    while True:
        
        wait(20, MSEC)
        

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()




