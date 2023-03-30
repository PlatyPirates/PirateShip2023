// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FourBarArms;

public class ArmHold extends CommandBase {
  private final FourBarArms _fourBarArms;

  /** Creates a new ArmHold. */
  public ArmHold(FourBarArms fourBarArms) {
    _fourBarArms = fourBarArms;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_fourBarArms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = 0.20;
    double target = 9.5;

    if (_fourBarArms.getPosition() < target) {
      _fourBarArms.go(power);
    } else if (_fourBarArms.getPosition() > target) {
      _fourBarArms.go(-power);
    } else {
      _fourBarArms.stop();
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
