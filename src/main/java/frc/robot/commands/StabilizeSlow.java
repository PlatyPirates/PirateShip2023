// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;

public class StabilizeSlow extends CommandBase {

  private final Drive_Train _drive_Train;
  private final AHRS _gyro;

  /** Creates a new StabilizeSlow. */
  public StabilizeSlow(Drive_Train drive_Train, AHRS gyro) {
    _drive_Train = drive_Train;
    _gyro = gyro;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_drive_Train);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = 0.34;
    double angle = 10; 

    if (_gyro.getRoll() > angle) {
      _drive_Train.drive(-power, 0);
    } else if (_gyro.getRoll() < -angle) {
      _drive_Train.drive(power, 0);
    } else {
      _drive_Train.drive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
