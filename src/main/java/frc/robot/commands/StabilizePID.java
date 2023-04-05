// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;

public class StabilizePID extends CommandBase {

  private Drive_Train _driveTrain;
  private AHRS _gyro;

  private PIDController _pid;
  private double _minPower;
  private double _maxPower;

  /** Creates a new Stabilize. */
  public StabilizePID(Drive_Train driveTrain, AHRS gyro) {
    _driveTrain = driveTrain;
    _gyro = gyro;
    addRequirements(_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _pid = new PIDController(
      _driveTrain.getBalancePID().getP(),
      _driveTrain.getBalancePID().getI(),
      _driveTrain.getBalancePID().getD()
    );

    _minPower = _driveTrain.getBalancePowerMin();
    _maxPower = _driveTrain.getBalancePowerMax(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = _gyro.getRoll();
    SmartDashboard.putNumber("Stabilize Err", error);
    double pidOut = _pid.calculate(error, 0);
    SmartDashboard.putNumber("Stabilize PIDOut", pidOut);

    double drivePower = pidOut / 15;
    SmartDashboard.putNumber("Stabilize Power", drivePower);
    if (Math.abs(drivePower) > _maxPower) {
      drivePower = Math.copySign(_maxPower, drivePower);
    }
    else if (Math.abs(drivePower) < _minPower) {
      drivePower = Math.copySign(_minPower, drivePower);
    }
    if (Math.abs(error) > 9) {
      _driveTrain.drive(drivePower, 0);
    } else {
      _driveTrain.drive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}