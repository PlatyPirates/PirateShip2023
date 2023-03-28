// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FourBarArms;

public class AutoArmIn extends CommandBase {
  private final FourBarArms _fourBarArms;
  private PIDController _PID;
  private int _count = 0;

  /** Creates a new AutoArmOut. */
  public AutoArmIn(FourBarArms fourBarArms) {
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
      double error = 0.0 - _fourBarArms.getPosition();
      double pidIn = _PID.calculate(error, 0.0); //replace 0 with arm out position (from limit)

      if(Math.abs(pidIn) > min_command) pidIn = Math.copySign(min_command, pidIn);
      _fourBarArms.go(pidIn); //adjust for units (ticks vs percent for power)

      SmartDashboard.putNumber("Error Value", error);
      SmartDashboard.putNumber("PID In", pidIn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = 0.0 - _fourBarArms.getPosition(); // from limit
    if(Math.abs(error) > 5) //change 5
      _count--;
    else
      _count = 0;

    return (_count <= 5);
  }
}
