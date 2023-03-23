// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakePivot extends SubsystemBase {
  private final CANSparkMax _pivotMotor = new CANSparkMax(Constants.IntakeConstants.pivotMotor, MotorType.kBrushless);

  private double _pivotPower = Constants.IntakeConstants.pivotMotorPower;

  private RelativeEncoder _encoder;

  /** Creates a new IntakePivot. */
  public IntakePivot() {
    setDefaultCommand(new RunCommand(this::stop, this));

    _pivotMotor.restoreFactoryDefaults();
    _pivotMotor.setIdleMode(IdleMode.kBrake);
    _pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    _pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, -IntakeConstants.pivotLimitOut);
    _pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    _pivotMotor.setSoftLimit(SoftLimitDirection.kForward, IntakeConstants.pivotLimitIn);
    _pivotMotor.burnFlash();

    _encoder = _pivotMotor.getEncoder();
  }

  public void stop() {
    _pivotMotor.stopMotor();
    SmartDashboard.putString("Pivot action", "stop");
  }

  public void pivotUp() {
    _pivotMotor.set(-_pivotPower);
    SmartDashboard.putString("Pivot action", "up");
  }

  public void pivotDown() {
    _pivotMotor.set(_pivotPower); 
    SmartDashboard.putString("Pivot action", "down");
  }

  public void setArmPower (double pivotPower) {
    _pivotPower = pivotPower; 
  }

  public double getPosition (){
    return _encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Position", _pivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Pivot Current", _pivotMotor.getOutputCurrent());
  }
}
