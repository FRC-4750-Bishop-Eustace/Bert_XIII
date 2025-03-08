import math
import wpilib
from rev import SparkFlex, SparkMax #joint motor is sparkmax, intake motor is sparkflex

#speed = 0.1

at_position_1 = False
at_position_2 = False
at_position_3 = False


class Coral_claw:
    def __init__(
        self,
        jointMotorID: int,
        intakeMotorID: int,
    ) -> None:

        self.jointMotor = SparkMax(jointMotorID, SparkMax.MotorType.kBrushless)
        #self.jointEncoder = self.jointMotor.AlternateEncoder
        self.intakeMotor = SparkFlex(intakeMotorID, SparkFlex.MotorType.kBrushless)

    def jointMotor_up(self, speed):
        self.jointMotor.setVoltage(-speed)

    def jointMotor_down(self, speed):
        self.jointMotor.setVoltage(speed)

    def jointMotor_stop(self):
        self.jointMotor.setVoltage(0)

    def intakeMotor_intaking(self, speed):
        self.intakeMotor.setVoltage(-speed)

    def intakeMotor_release(self, speed):
        self.intakeMotor.setVoltage(speed)

    def intakeMotor_stop(self):
        self.intakeMotor.setVoltage(0)


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






