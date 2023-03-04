// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Booty_Intake;

public class IntakeOut extends CommandBase {
  private final Booty_Intake _bootyIntake;

  /** Creates a new IntakeOut. */
  public IntakeOut(Booty_Intake bootyIntake) {
    _bootyIntake = bootyIntake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_bootyIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // _bootyIntake.intakeOut();
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
