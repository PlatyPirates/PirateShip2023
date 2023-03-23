// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Booty_Intake;
import frc.robot.subsystems.Booty_Intake.BootyState;

public class AutoIntakeOut extends CommandBase {
  /** Creates a new AutoIntakeOut. */
  private final Booty_Intake _bootyIntake;
  private long _startTime;
  public AutoIntakeOut(Booty_Intake bootyIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    _bootyIntake = bootyIntake;
    addRequirements(_bootyIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _startTime = System.currentTimeMillis();
    _bootyIntake.setState(BootyState.ConeIntake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _bootyIntake.setState(BootyState.Off);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long elapsedTime = System.currentTimeMillis()-_startTime;

    return (elapsedTime > 3000);
  
  }
}
