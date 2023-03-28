// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drive_Train;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static CommandBase TestStraight(Drive_Train driveTrain) {
    return GeneratePath(driveTrain, "Test Straight");

  }

  public static CommandBase TestCurve(Drive_Train driveTrain) {
    return GeneratePath(driveTrain, "Test Curve");
  }

  public static CommandBase DriveForward(Drive_Train driveTrain) {
    return new DriveBackward(driveTrain);
  }

  private static CommandBase GeneratePath(Drive_Train driveTrain, String pathName) {
    PathPlannerTrajectory path = PathPlanner.loadPath(pathName, new PathConstraints(0.5, 0.5));
    PPRamseteCommand ramseteCommand = new PPRamseteCommand(
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
    driveTrain.resetOdometry(path.getInitialPose());
    return ramseteCommand;
  }
}
