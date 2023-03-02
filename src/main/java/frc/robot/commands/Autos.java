// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drive_Train;
import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
  public static CommandBase TestStraight(Drive_Train driveTrain)
  {
    PathPlannerTrajectory path = PathPlanner.loadPath("Test Stright", new PathConstraints(1, 1));
    PPRamseteCommand ramseteCommand =
        new PPRamseteCommand(
            path,
            driveTrain::getPose,
            new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DrivetrainConstants.ksVolts,
                DrivetrainConstants.kvVoltSecondsPerMeter,
                DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            DrivetrainConstants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
            new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
            driveTrain::tankDriveVolts,
            driveTrain);
          return ramseteCommand;
  }
}

