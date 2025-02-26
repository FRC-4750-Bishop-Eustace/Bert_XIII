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
# import urcl

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.Joystick(variables.joystickPort1)
        self.controller2 = wpilib.Joystick(variables.joystickPort2)
        self.pad = wpilib.Joystick(variables.joystickPort3)

        self.swerve = drivetrain.Drivetrain()
        #self.odometry = 

        self.elevator = elevator.Elevator(16, 17, [5, 10, 20, 30])
        #self.elevator2 = elevator2.Elevator(16, 17, [5, 10, 20, 30])
        #self.limelight = limelight.PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, avgTagDist, avgTagArea, fiducials)

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

        if self.trajectory:
            # Get the initial pose of the trajectory
            initial_pose = self.trajectory.get_initial_pose(self.is_red_alliance())

            if initial_pose:
                # Reset odometry to the start of the trajectory
                self.swerve.updateOdometry()

        # Reset and start the timer when the autonomous period begins
        self.timer.restart()

        #self.autoSelected = self.chooser.getSelected()
        #print("Auto selected:" + self.autoSelected)

    def autonomousPeriodic(self) -> None:

        self.field.setRobotPose(self.swerve.odometry.getPose())

        if self.trajectory:
            # Sample the trajectory at the current time into the autonomous period
            sample = self.trajectory.sample_at(self.timer.get(), self.is_red_alliance())

            if sample:
                self.swerve.follow_trajectory(sample)
                


    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def teleopInit(self) -> None:
        self.swerve.resetRobotPose(self.swerve.odometry.getPose())
        self.initPose = self.swerve.odometry.getPose()


    def teleopPeriodic(self) -> None:

        #self.swerve.updateOdometry()
        #self.field.setRobotPose(self.swerve.odometry.getPose())

        self.tx = self.lmtable.getNumber('tx', None)
        self.ty = self.lmtable.getNumber('ty', None)
        self.ta = self.lmtable.getNumber('ta', None)
        self.ts = self.lmtable.getNumber('ts', None)
        self.tid = self.lmtable.getNumber('tid', None)
        self.hw = self.lmtable.getNumber('hw', None)

        self.botpose = self.lmtable.getEntry('botpose_wpiblue').getDoubleArray([])

        #self.tagpose = self.botpose.toPose2d()
        #self.field.setRobotPose(self.botpose[0], self.botpose[1], self.botpose[2])

        self.swerve.UpdateEstimator()
        #print(self.visionPose)
        #self.swerve.estimator.addVisionMeasurement(wpimath.geometry.Pose2d(self.botpose[0], self.botpose[1], self.botpose[5]), self.getPeriod())
        self.visionPose = self.swerve.estimator.getEstimatedPosition()
        #self.field.setRobotPose(wpimath.geometry.Pose2d(self.botpose[0], self.botpose[1], self.botpose[5]))
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
        
        if self.pad.getRawButton(6) == 1: #down
            self.elevator.start_elevatorMotor(3)
        elif self.pad.getRawButton(9) == 1: #up
            self.elevator.start_elevatorMotor(8)
        else:
            self.elevator.stop_elevatorMotor()
        
        #self.voltage = SmartDashboard.getNumber('voltage', 1)
        '''
        if self.pad.getRawButton(2) == 1: #down
            self.elevator2.start_elevatorMotor(-self.voltage)
        elif self.pad.getRawButton(4) == 1: #up
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

        xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRawAxis(1), variables.x_deadband)
            )
            * variables.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        # NOTE: Check if we need inversion here
        ySpeed = (
            self.yspeedLimiter.calculate(
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
                -wpimath.applyDeadband((self.controller.getRawAxis(3) + 1) / 2, variables.rot_deadband)
            ) +
                wpimath.applyDeadband((self.controller.getRawAxis(4) + 1) / 2, variables.rot_deadband)
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
        variables.setTurnState(rot)

        #self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
    

        