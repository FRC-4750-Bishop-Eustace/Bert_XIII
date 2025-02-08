# Global Vars

import wpimath.geometry
import math
import robot

# SPEED CONFIG
kMaxSpeed = 4.6  # meters per second 4.5
kRMaxSpeed = 8.5 # 10
kTMaxSpeed = 4.6 #4.5
kMaxAngularSpeed = math.pi  # 1/2 rotation per second
frontLeftZero = 0
frontRightZero = 0
backLeftZero = 0
backRightZero = 0
zeroThreshold = wpimath.geometry.Rotation2d(0.3)

# PID and FeedForward
drivePID_P = 0.02
drivePID_I = 0
drivePID_D = 0
turnPID_P = 7.8 #12 # 3
turnPID_I = 0
turnPID_D = 0.055 #0.1 #0.4750
driveFF_1 = 0.06#1.8
driveFF_2 = 2.5#3
turnFF_1 = 0 #0.05
turnFF_2 = 0 #0.45
elevatorPID_P = 0
elevatorPID_I = 0
elevatorPID_D = 0 # Dampening for oscillation
elevatorFF_1 = 0.1 # kS: Static friction compensation
elevatorFF_2 = 0.05 # kG: Gravity compensation
elevatorFF_3 = 0.34527 # kV: Velocity feedforward
elevatorFF_4 = 0 # kA: Acceleration feedforward


TurnState = 0

# MODULE/CHASSIS CONFIG
chassisHalfLength = 0.32 # Chassis should be square

frontLeftDriveController = 1
frontLeftTurnController = 5
frontLeftDriveEncoder = 1
frontLeftTurnEncoder = 9

frontRightDriveController = 2
frontRightTurnController = 3
frontRightDriveEncoder = 2
frontRightTurnEncoder = 11

backRightDriveController = 8
backRightTurnController = 6
backRightDriveEncoder = 8
backRightTurnEncoder = 10

backLeftDriveController = 4
backLeftTurnController = 7
backLeftDriveEncoder = 4
backLeftTurnEncoder = 12

# CONTROLLERS/DEADBAND/SLEW
joystickPort1 = 2
joystickPort2 = 0
joystickPort3 = 3

x_slewrate = 3
y_slewrate = 3
rot_slewrate = 1

x_deadband = 0.1
y_deadband = 0.1
rot_deadband = 0.2

triangleButton = 4
squareButton = 1
crossButton = 2
circleButton = 3
L1Button = 5
R1Button = 6
L2Button = 7
R2Button = 8
bigPad = 14
PSbutton = 13
shareButton = 9
optionsButton = 10

# SHOOTER + SHOOTER SPEEDS
shooter_motor1ID = 16
shooter_motor2ID = 17
intake_motor1ID = 13
intake_motor2ID = 14
loader_motorID = 15
climber_motorID = 18

intakeSpeedMotor1 = 2
intakeSpeedMotor2 = -2
ampSpeedMotor1 = 1
ampSpeedMotor2 = 2
shootSpeedMotor1 = -200 #-50
shootSpeedMotor2 = -200 #-50
#shootSpeedMotor1 = -1
#shootSpeedMotor2 = -1
loaderSpeedMotor = 0.5

reverseShootMotor1 = 3
reverseShootMotor2 = 3
reverseLoaderMotor = -0.5
reverseIntakeMotor1 = -2
reverseIntakeMotor2 = 2

climbUpSpeed = 10
climbDownSpeed = -10
# NEED TO CHANGE THE SHOOTER FUNCTIONS 

def setTurnState(rot) -> None:
    global TurnState
    if abs(rot) > 0:
        TurnState = 1
    else:
        TurnState = 0
    #print(TurnState)
    #return TurnState