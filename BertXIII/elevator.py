import math
from rev import SparkFlex, SparkFlexConfig, SparkBase
from wpimath.controller import ProfiledPIDController, ElevatorFeedforward
import wpimath.kinematics
import variables
import wpimath.geometry
from wpilib import SmartDashboard

kMaxVelocity = 16
kMaxAcceleration = 48

class Elevator:
    def __init__(
        self,
        elevatorMotor1ID: int,
        elevatorMotor2ID: int,
        heights: list[int]
    ) -> None:

        self.elevatorMotor1 = SparkFlex(elevatorMotor1ID, SparkFlex.MotorType.kBrushless)
        self.elevatorEncoder1 = self.elevatorMotor1.getEncoder()
        self.elevatorConfig1 = SparkFlexConfig()
        self.elevatorConfig1.inverted(True)
        self.elevatorConfig1.encoder.positionConversionFactor(5.5 / 15)
        self.elevatorConfig1.encoder.velocityConversionFactor((5.5 / 15) / 60)
        self.elevatorMotor1.configure(self.elevatorConfig1, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.elevatorMotor2 = SparkFlex(elevatorMotor2ID, SparkFlex.MotorType.kBrushless)
        self.elevatorEncoder2 = self.elevatorMotor2.getEncoder()
        self.elevatorConfig2 = SparkFlexConfig()
        self.elevatorConfig2.encoder.positionConversionFactor(5.5 / 15)
        self.elevatorConfig2.encoder.velocityConversionFactor((5.5 / 15) / 60)
        self.elevatorMotor2.configure(self.elevatorConfig2, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        self.heights = heights

        self.elevatorPIDController = ProfiledPIDController(
            variables.elevatorPID_P,
            variables.elevatorPID_I,
            variables.elevatorPID_D,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kMaxVelocity,
                kMaxAcceleration
            )
        )

        self.elevatorFeedforward = ElevatorFeedforward(
            variables.elevatorFF_1,
            variables.elevatorFF_2,
            variables.elevatorFF_3,
            variables.elevatorFF_4
        )

        SmartDashboard.putData("ElevatorPID", self.elevatorPIDController)

    def findPoints(self):
        encoderRotation = self.elevatorEncoder1.getPosition()

        SmartDashboard.putNumber("encoderRotation", self.elevatorEncoder1.getPosition())
        SmartDashboard.putNumber("setpoint", self.elevatorPIDController.getSetpoint().position)

    def start_elevatorMotor(self, setpoint: float):
        encoderRotation = self.elevatorEncoder1.getPosition()

        SmartDashboard.putNumber("encoderRotation", self.elevatorEncoder1.getPosition())
        SmartDashboard.putNumber("setpoint", self.elevatorPIDController.getSetpoint().position)

        # self.elevatorPIDController.setConstraints(
        #     wpimath.trajectory.TrapezoidProfile.Constraints(
        #         velocityScale,
        #         accelerationScale
        #     )
        # )

        output = self.elevatorPIDController.calculate(
            encoderRotation, setpoint
        )

        feedforward = self.elevatorFeedforward.calculate(
            self.elevatorPIDController.getSetpoint().velocity
        )
            
        self.elevatorMotor1.setVoltage((output + feedforward))
        self.elevatorMotor2.setVoltage((output + feedforward))

    def stop_elevatorMotor(self):
        #print('motor stopped')
        self.elevatorMotor1.setVoltage(0)
        self.elevatorMotor2.setVoltage(0)
    
    def get_elevatorEncoder(self):
        print('encoder revolutions = ', self.elevatorEncoder1.getPosition())
        print('encoder velocity = ', self.elevatorEncoder2.getVelocity())

    def set_elevatorEncoder(self, x):
        print('Setting Encoder to: ', x)
        self.elevatorEncoder1.setPosition(x)
        self.elevatorEncoder2.setPosition(-x)
    
    def set_elevatorMode(self, mode: int):
        mode = max(4, min(mode, 0))
        match mode:
            case 0:
                while self.elevatorEncoder.getPosition() > 0:
                    self.start_elevatorMotor(0)
            case 1:
                self.start_elevatorMotor(self.heights[0])
            case 2:
                self.start_elevatorMotor(self.heights[1])
            case 3:
                self.start_elevatorMotor(self.heights[2])
            case 4:
                self.start_elevatorMotor(self.heights[3])