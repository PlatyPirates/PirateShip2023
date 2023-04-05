// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Booty_Intake extends SubsystemBase {
  private CANSparkMax _intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotor, MotorType.kBrushless);

  public enum BootyState {
    Off,
    CubeIntake,
    CubeHold,
    ConeIntake,
    ConeHold
  }

  public BootyState _state = BootyState.Off;

  /** Creates a new Booty_Intake. */
  public Booty_Intake() {
    _intakeMotor.restoreFactoryDefaults(); 
    _intakeMotor.setIdleMode(IdleMode.kBrake);
    _intakeMotor.burnFlash();

  }

  public void setState(BootyState state) {
    _state = state;
  }
  

  @Override
  public void periodic() {
    double _power = 0.0;
    int currentLimit = 25;

    // This method will be called once per scheduler run

    if(_state == BootyState.Off) {
      _power = 0.0;
      currentLimit = 25;
    }else if(_state == BootyState.CubeIntake) {
      _power = 0.30;
      currentLimit = 25;
    } else if (_state == BootyState.ConeIntake) {
      _power = -0.60;
      currentLimit = 25;
    } else if (_state == BootyState.CubeHold) {
      _power = 0.0;
      currentLimit = 5;
    } else if (_state == BootyState.ConeHold) {
      _power = -0.07;
      currentLimit = 5;
    } 

    _intakeMotor.set(_power);
    _intakeMotor.setSmartCurrentLimit(currentLimit); 

    SmartDashboard.putString("Intake State", _state.toString()); 
  }
}
