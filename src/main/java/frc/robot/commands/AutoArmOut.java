// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FourBarArms;

public class AutoArmOut extends CommandBase {
  private final FourBarArms _fourBarArms;
  private PIDController _PID;
  private int _count = 0;

  /** Creates a new AutoArmOut. */
  public AutoArmOut(FourBarArms fourBarArms) {
    _fourBarArms = fourBarArms;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_fourBarArms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _PID = new PIDController(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double min_command = 0.1;
      double error = 0 - _fourBarArms.getPosition();
      double pidOut = _PID.calculate(error, 0); //replace 0 with arm out position

      if(Math.abs(pidOut) < min_command) pidOut = Math.copySign(min_command, pidOut);
      _fourBarArms.go(pidOut); //adjust for units (ticks vs percent for power)

      SmartDashboard.putNumber("Error Value", error);
      SmartDashboard.putNumber("PID Out", pidOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = 0 - _fourBarArms.getPosition();
    if(Math.abs(error) < 5) //change 5
      _count++;
    else
      _count = 0;

    return (_count >= 5);
  }
}
