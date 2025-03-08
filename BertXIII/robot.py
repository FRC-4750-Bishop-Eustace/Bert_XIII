#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import wpimath.estimator
import drivetrain
import variables
import elevator
import elevator2
import navxGyro
import ntcore
#import limelight
#import limelightresults
import commands2
import LimelightHelpers
import json
import time
#import limelight
#import vision
#import camera
#import auto
import ntcore
from wpilib import SmartDashboard, Field2d
from cscore import CameraServer
from wpilib import SmartDashboard
import choreo
import coral_claw
import climb
from urcl import URCL

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        wpilib.DataLogManager.start()
        URCL.start()
        self.controller = wpilib.Joystick(variables.joystickPort1)
        self.controller2 = wpilib.Joystick(variables.joystickPort2)
        self.pad = wpilib.Joystick(variables.joystickPort3)

        self.swerve = drivetrain.Drivetrain()
        #self.odometry = 

        self.elevator = elevator.Elevator(16, 17, [5, 10, 20, 30])
        #self.elevator2 = elevator2.Elevator(16, 17, [5, 10, 20, 30])
        #self.limelight = limelight.PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, avgTagDist, avgTagArea, fiducials)
        self.claw = coral_claw.Coral_claw(20, 18)
        self.climb = climb.Climber(15, 19)

        # navxGyro is a file to test the navx Gyro. This can be ignored/commented out.
        self.navxGyro = navxGyro.Gyro()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        # Speed limiters

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(variables.x_slewrate)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(variables.y_slewrate)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(variables.rot_slewrate)

        self.timer = wpilib.Timer()
        self.fieldDrive = 1
        #CameraServer.startAutomaticCapture()

        self.setpoint = 0
        
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.lmtable = self.inst.getTable("limelight")

        self.field = Field2d()
        SmartDashboard.putData("field", self.field)
        #SmartDashboard.putData("Swerve", self.swerve)
        
        # Loads from deploy/choreo/myTrajectory.traj
        # ValueError is thrown if the file does not exist or is invalid
        
        try:
            self.trajectory = choreo.load_swerve_trajectory("myTraj_15") # 
        except ValueError:
        # If the trajectory is not found, ChoreoLib already prints to DriverStation
            pass

        wpilib.DataLogManager.start()
        #urcl.start()

        #urcl.start(wpilib.DataLogManager.getLog())

        SmartDashboard.putNumber("voltage", 1)
        

    def robotPeriodic(self):
        self.swerve.updateOdometry()

    #FUTURE
    def autonomousInit(self):
        self.swerve.resetGyro()

        follow1 = commands2.cmd.run(lambda: self.FollowChoreoPath(self.trajectory)).withTimeout(6.7)
        follow2 = commands2.cmd.run(lambda: self.FollowChoreoPath(self.trajectory2)).withTimeout(4.4)
        stop = commands2.cmd.run(lambda: self.StopPath())
        
        if self.trajectory:
            # Get the initial pose of the trajectory
            initial_pose = self.trajectory.get_initial_pose(self.is_red_alliance())

            self.swerve.resetRobotPose(initial_pose)

            if initial_pose:
                # Reset odometry to the start of the trajectory
                self.swerve.updateOdometry()

        self.path_command = commands2.SequentialCommandGroup([
            follow1,
            follow2,
            stop
        ])
        self.path_command.schedule()

        # Reset and start the timer when the autonomous period begins
        self.timer.restart()

        #self.autoSelected = self.chooser.getSelected()
        #print("Auto selected:" + self.autoSelected)

    def autonomousPeriodic(self) -> None:

        self.matchTimer = self.timer.getMatchTime()

        self.field.setRobotPose(self.swerve.odometry.getPose())

        commands2.CommandScheduler.getInstance().run()
                

    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def teleopInit(self) -> None:
        self.swerve.resetRobotPose(self.swerve.odometry.getPose())
        self.initPose = self.swerve.odometry.getPose()


    def teleopPeriodic(self) -> None:

        #self.swerve.updateOdometry()
        #self.field.setRobotPose(self.swerve.odometry.getPose())

        self.matchTimer = self.timer.getFPGATimestamp()

        self.tx = self.lmtable.getNumber('tx', None)
        self.ty = self.lmtable.getNumber('ty', None)
        self.ta = self.lmtable.getNumber('ta', None)
        self.ts = self.lmtable.getNumber('ts', None)
        self.tid = self.lmtable.getNumber('tid', None)
        self.hw = self.lmtable.getNumber('hw', None)

        self.botpose = self.lmtable.getEntry('botpose_wpiblue').getDoubleArray([])

        #self.elevator.findPoints()

        #self.tagpose = self.botpose.toPose2d()
        #self.field.setRobotPose(self.botpose[0], self.botpose[1], self.botpose[2])

        #if self.ta > 2.0:
        #    self.swerve.estimator.addVisionMeasurement(wpimath.geometry.Pose2d(self.botpose[0], self.botpose[1], self.botpose[5]-90), self.matchTimer)
        self.swerve.UpdateEstimator()
        #print(self.visionPose)
        self.visionPose = self.swerve.estimator.getEstimatedPosition()
        #rint(self.getPeriod())
        #print(wpimath.geometry.Pose2d(self.botpose[0], self.botpose[1], self.botpose[5]), self.matchTimer)
        #print(self.visionPose)
        self.field.setRobotPose(self.visionPose)
        

        # CHANGE TO FIELD DRIVE VS BOT RELETIVE
        if self.controller.getRawButton(variables.crossButton) == 1:
            self.fieldDrive = 2
        if self.controller.getRawButton(variables.circleButton) == 1:
            self.fieldDrive = 1

        if self.controller.getRawButton(variables.triangleButton) == 1:
            self.swerve.resetGyro()

        if self.fieldDrive == 2:
            self.driveWithJoystick(True)
        else:
            self.driveWithJoystick(False)
    
        self.navxGyro.getGyro()
        
        
        # MAX SET POINT IS AROUND 22.5
        if self.pad.getRawButton(6) == 1: #down
            self.setpoint = 3
            self.elevator.start_elevatorMotor(self.setpoint)
        elif self.pad.getRawButton(9) == 1: #up
            self.setpoint = 8
            self.elevator.start_elevatorMotor(self.setpoint)
        else:
            self.elevator.start_elevatorMotor(self.setpoint)
        

        self.voltage = SmartDashboard.getNumber('voltage', 1)

        # FOR CLAW
        if self.pad.getRawButton(5) == 1: # up
            self.claw.jointMotor_up(self.voltage)
        elif self.pad.getRawButton(3) == 1:
            self.claw.jointMotor_down(self.voltage)
        else:
            self.claw.jointMotor_stop()
        
        if self.pad.getRawButton(2) == 1:
            self.claw.intakeMotor_intaking(self.voltage)
        elif self.pad.getRawButton(1) == 1:
            self.claw.intakeMotor_release(self.voltage)
        else:
            self.claw.intakeMotor_stop()

        if self.pad.getRawButton(7):
            self.climb.move(self.voltage)
        elif self.pad.getRawButton(10):
            self.climb.move(-self.voltage)
        else:
            self.climb.stop()
        
        '''
        if self.pad.getRawButton(6) == 1: #down
            self.elevator2.start_elevatorMotor(-self.voltage)
        elif self.pad.getRawButton(9) == 1: #up
            self.elevator2.start_elevatorMotor(self.voltage)
        else:
            self.elevator2.stop_elevatorMotor()
        '''
        

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        # NOTE: Check if we need inversion here
        #if fieldRelative:
        #    self.swerve.updateOdometry()

        self.dPad = self.controller.getPOV()

        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), variables.x_deadband)
            )
            * variables.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        # NOTE: Check if we need inversion here
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(0), variables.y_deadband)
            )
            * variables.kTMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.

        '''
        rot = (
            (-self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed) +
            (self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed)
        )
        '''
        rot = (
            (self.rotLimiter.calculate(
                wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            ) +
                -wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
            )
            * variables.kRMaxSpeed)

        '''
        rot = (
            (self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(2), variables.rot_deadband)
            ) / 2)
            * variables.kRMaxSpeed
        )
        '''

        if self.dPad == 0:
            xSpeed = 0.2
        if self.dPad == 90:
            ySpeed = -0.2
        
        if self.dPad == 180:
            xSpeed = -0.2
        if self.dPad == 270:
            ySpeed = 0.2
        
        if self.controller.getRawButton(variables.L1Button) == 1:
            rot = 0.5
        if self.controller.getRawButton(variables.R1Button) == 1:
            rot = -0.5
    
        variables.setTurnState(rot)

        #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
    
    def FollowChoreoPath(self, trajectory):
        sample = trajectory.sample_at(self.timer.get(), self.is_red_alliance())

        if sample:
            self.swerve.follow_trajectory(sample)
    
    def StopPath(self):
        self.swerve.drive(0, 0, 0, True, self.getPeriod())
        