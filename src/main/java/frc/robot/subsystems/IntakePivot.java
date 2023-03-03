// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
  private final CANSparkMax _pivotMotor = new CANSparkMax(Constants.IntakeConstants.pivotMotor, MotorType.kBrushless);

  private double _pivotPower = Constants.IntakeConstants.pivotMotorPower;

  /** Creates a new IntakeUpDown. */
  public IntakePivot() {
    setDefaultCommand(new RunCommand(this::stop, this));

    _pivotMotor.restoreFactoryDefaults();

    _pivotMotor.burnFlash();
  }

  public void stop() {
    _pivotMotor.stopMotor();
  }

  public void pivotUp() {
    _pivotMotor.set(-_pivotPower);
  }

  public void pivotDown() {
    _pivotMotor.set(_pivotPower); 
  }

  public void setArmPower (double pivotPower) {
    _pivotPower = pivotPower; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
