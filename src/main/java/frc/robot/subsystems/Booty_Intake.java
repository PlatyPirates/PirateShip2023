// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Booty_Intake extends SubsystemBase {
  private CANSparkMax _intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotor, MotorType.kBrushed);

  public enum BootyState {
    Off,
    CubeIntake,
    CubeHold,
    ConeIntake,
    ConeHold
  }

  public BootyState _state = BootyState.Off;

  //public double _power = Constants.IntakeConstants.intakeMotorPower;
  /** Creates a new Booty_Intake. */
  public Booty_Intake() {
    //setDefaultCommand(new RunCommand(this::stop, this));

    _intakeMotor.restoreFactoryDefaults(); //this may be causing the motor controller to burn out

    //_intakeMotor.burnFlash();
  }

  public void stop() {
    //_intakeMotor.stopMotor();
  }
  
  /*
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
*/

  public void setState(BootyState state) {
    _state = state;
  }
  

  @Override
  public void periodic() {
    double _power = 0.0;
    int currentLimit = 40;

    // This method will be called once per scheduler run

    if(_state == BootyState.Off) {
      _power = 0.0;
      currentLimit = 55;
    }else if(_state == BootyState.CubeIntake) {
      _power =0.25;
      currentLimit = 55;
    } else if (_state == BootyState.ConeIntake) {
      _power = -0.60;
      currentLimit = 55;
    } else if (_state == BootyState.CubeHold) {
      _power = 0.10;
      currentLimit = 20;
    } else if (_state == BootyState.ConeHold) {
      _power = -0.12;
      currentLimit = 20;
    } 

    _intakeMotor.set(_power);
    _intakeMotor.setSmartCurrentLimit(currentLimit); 

    SmartDashboard.putNumber("Intake Power", _power);
    SmartDashboard.putString("Intake State", _state.toString()); 
    SmartDashboard.putNumber("Intake Current", _intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("current limit", currentLimit);
  }
}
