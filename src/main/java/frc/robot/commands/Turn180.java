// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;

public class Turn180 extends CommandBase {
  private final Drive_Train _driveTrain;
  private final AHRS _gyro;

  public Turn180(Drive_Train driveTrain, AHRS gyro) {
    _driveTrain = driveTrain;
    _gyro = gyro;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _driveTrain.drive(0.0, _driveTrain.getAutoTurnPower());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((180 - Math.abs(_gyro.getAngle())) < 5);
  }
}
