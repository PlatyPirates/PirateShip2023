// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Booty_Intake extends SubsystemBase {
  private final CANSparkMax _intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotor, MotorType.kBrushed);

  private double _power = Constants.IntakeConstants.intakeMotorPower;
  /** Creates a new Booty_Intake. */
  public Booty_Intake() {
    setDefaultCommand(new RunCommand(this::stop, this));

    _intakeMotor.restoreFactoryDefaults();

    _intakeMotor.burnFlash();
  }

  public void stop() {
    _intakeMotor.stopMotor();
  }
  
  public void intakeIn() {
    _intakeMotor.set(_power);
  }

  public void intakeOut() {
    _intakeMotor.set(-_power);
  }

  public void setPower (double power){
    _power = power;
  }

  public double getPower(){
    return _power;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
