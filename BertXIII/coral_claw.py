import math
import wpilib
from rev import SparkFlex, SparkMax #joint motor is sparkmax, intake motor is sparkflex
from wpimath.controller import ProfiledPIDController, ElevatorFeedforward
import wpimath
from wpilib import SmartDashboard
import variables

#speed = 0.1

at_position_1 = False
at_position_2 = False
at_position_3 = False

kMaxVelocity = 2
kMaxAcceleration = 1

class Coral_claw:
    def __init__(
        self,
        jointMotorID: int,
        intakeMotorID: int,
    ) -> None:

        self.jointMotor = SparkMax(jointMotorID, SparkMax.MotorType.kBrushless)
        self.jointEncoder = self.jointMotor.getAbsoluteEncoder()
        #self.jointEncoder = self.jointMotor.AlternateEncoder
        self.intakeMotor = SparkFlex(intakeMotorID, SparkFlex.MotorType.kBrushless)

        self.clawPIDController = ProfiledPIDController(
            variables.clawPID_P,
            variables.clawPID_I,
            variables.clawPID_D,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kMaxVelocity,
                kMaxAcceleration
            )
        )

        self.clawFeedforward = ElevatorFeedforward(
            variables.clawFF_1,
            variables.clawFF_2,
            variables.clawFF_3,
            variables.clawFF_4
        )

        SmartDashboard.putData("clawPID", self.clawPIDController)
    '''
    def jointMotor_set(self, setpoint):
        #self.jointMotor.setVoltage(-speed)
        encoderRotation = self.jointEncoder.getPosition()

        output = self.clawPIDController.calculate(
            encoderRotation, setpoint
        )

        feedforward = self.clawFeedforward.calculate(
            self.clawPIDController.getSetpoint().velocity
        )
        #print(output + feedforward)
        self.jointMotor.setVoltage((output + feedforward))
    '''

    def jointMotor_set(self, speed):
        encoderRotation = self.jointEncoder.getPosition()

        s0 = 0.7
        z1 = 0.04 
        s1 = 0.2
        z2 = 0.33
        s2 = -0.6
        z3 = 0.35
        s3 = -1
        z4 = 0.6
        s4 = -1.8

        if encoderRotation <= z1:
            self.jointMotor.setVoltage(s0)
        elif z1 < encoderRotation <= z2:
            self.jointMotor.setVoltage(s1)
        elif z2 < encoderRotation <= z3 :
            self.jointMotor.setVoltage(s2)
        elif z3 < encoderRotation <= z4:
            self.jointMotor.setVoltage(s3)
        elif encoderRotation > z4:
            self.jointMotor.setVoltage(s4)
            

    def jointMotor_up(self, speed):
        self.jointMotor.setVoltage(-speed)

    def jointMotor_down(self, speed):
        self.jointMotor.setVoltage(speed)
    
    def jointMotor_hold(self):
        encoderRotation = self.jointEncoder.getPosition()
    
        z2 = 0.1
        s2 = -0.6#-0.5
        z3 = 0.2
        s3 = -1.6

        point3 = 0.9
        point4 = 1
        speed3 = 0.8

        if point3 > encoderRotation >= z2:
            self.jointMotor.setVoltage(s2)
        #elif z2 > encoderRotation >= 0:
        elif point4 >= encoderRotation > point3: # if claw moves to far back it resets to "1"
            self.jointMotor.setVoltage(speed3)
        else:
            self.jointMotor.setVoltage(0)

        '''
        if z2 < encoderRotation <= z3:
            self.jointMotor.setVoltage(s2)
        elif point3 >= encoderRotation > z3:
            self.jointMotor.setVoltage(s3)
        elif point4 >= encoderRotation > point3: # if claw moves to far back it resets to "1"
            self.jointMotor.setVoltage(speed3)
        elif encoderRotation <= z2:
            self.jointMotor.setVoltage(0)
        '''

    def jointMotor_stop(self):
        self.jointMotor.setVoltage(0)
    
    def jointMotor_stow(self):

        encoderRotation = self.jointEncoder.getPosition()

        point1 = 0.14
        point2 = 0.2
        point3 = 0.9
        point4 = 1
    
        speed = -0.3
        speed2 = -1.6
        speed3 = 1.6
        speed4 = 0.2

        if point1 < encoderRotation <= point2: # if at point
            self.jointMotor.setVoltage(speed)
        elif point3 >= encoderRotation > point2: # If lower
            self.jointMotor.setVoltage(speed2)
        elif point4 >= encoderRotation > point3: # if claw moves to far back it resets to "1"
            self.jointMotor.setVoltage(speed3)
        elif encoderRotation <= point1:
            self.jointMotor.setVoltage(speed4)

    def jointMotor_idle(self):
        encoderRotation = self.jointEncoder.getPosition()

        z1 = 0
        z2 = 1
        z3 = 0.9
        s0 = -1.2
        
        if z3 > encoderRotation >= z1:
            self.jointMotor.setVoltage(s0)
        elif z2 > encoderRotation >= z3:
            self.jointMotor.setVoltage(0)

    def intakeMotor_intaking(self, speed):
        self.intakeMotor.setVoltage(-speed)

    def intakeMotor_release(self, speed):
        self.intakeMotor.setVoltage(speed)

    def intakeMotor_stop(self):
        self.intakeMotor.setVoltage(0)

    def printEncoder(self):
        encoderRotation = self.jointEncoder.getPosition()
        SmartDashboard.putNumber("clawRotation", encoderRotation)
        SmartDashboard.putNumber("ClawSetpoint", self.clawPIDController.getSetpoint().position)
        #print(encoderRotation)
    
    def encoderValue(self):
        return self.jointEncoder.getPosition()

    def goto(self, n):  #1 = up  2 = forward  3 = down
        if n == 1 and at_position_1 is False:
            jointMotor_up()
        elif n == 2 and at_position_2 is False:
            if at_position_1:
                jointMotor_down()
            elif at_position_3:
                jointMotor_up()

        elif n == 3 and at_position_3 is False:
            jointMotor_down()
        else:
            jointMotor_stop()






