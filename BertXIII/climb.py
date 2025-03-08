import wpilib
from rev import SparkBase, SparkFlex, SparkFlexConfig
from enum import Enum

class Climber:
    # Climb state enumerator
    class ClimbState(Enum):
        kStateUp = 0
        kStateDown = 1
        kStateStop = 2

    def __init__(self, leftMotorID: int, rightMotorID: int):
        # Left side climb
        self.leftMotor = SparkFlex(leftMotorID, SparkFlex.MotorType.kBrushless)
        #self.leftEncoder = self.motor.getEncoder()
        self.leftConfig = SparkFlexConfig()
        self.leftConfig.IdleMode(SparkFlex.IdleMode.kBrake)
        self.leftMotor.configure(self.leftConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        # Right side climb
        self.rightMotor = SparkFlex(rightMotorID, SparkFlex.MotorType.kBrushless)
        #self.rightEncoder = self.motor.getEncoder()
        self.rightConfig = SparkFlexConfig()
        self.rightConfig.IdleMode(SparkFlex.IdleMode.kBrake)
        self.rightMotor.configure(self.rightConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

    def set(self, state: ClimbState):
        states = {
            self.ClimbState.kStateUp: 30,
            self.ClimbState.kStateDown: -30,
            self.ClimbState.kStateStop: 0
        }
        self.move(states.get(state, 0))

    def move(self, voltage):
        self.leftMotor.setVoltage(voltage)
        self.rightMotor.setVoltage(-voltage)

    def stop(self):
        self.leftMotor.setVoltage(0)
        self.rightMotor.setVoltage(0)
