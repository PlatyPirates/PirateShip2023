// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.JoystickConstants;

public class Drive_Train extends SubsystemBase {
  
  private final CANSparkMax _fLMotor = new CANSparkMax(DrivetrainConstants.frontLeftMotor, MotorType.kBrushless);
  private final CANSparkMax _fRMotor = new CANSparkMax(DrivetrainConstants.frontRightMotor, MotorType.kBrushless);
  private final CANSparkMax _bLMotor = new CANSparkMax(DrivetrainConstants.backLeftMotor, MotorType.kBrushless);
  private final CANSparkMax _bRMotor = new CANSparkMax(DrivetrainConstants.backRightMotor, MotorType.kBrushless);

  private final DifferentialDrive _drive = new DifferentialDrive(_fLMotor, _fRMotor);

  public Drive_Train() {
    _fLMotor.restoreFactoryDefaults();
    _fRMotor.restoreFactoryDefaults();
    _bLMotor.restoreFactoryDefaults();
    _bRMotor.restoreFactoryDefaults();

    //modify this if the motors are spinning in different directions
    _fLMotor.setInverted(true);
    _bLMotor.setInverted(true);

    _bLMotor.follow(_fLMotor);
    _bRMotor.follow(_fRMotor);

    enableOpenLoopRampRate(false);
  }

  public void enableOpenLoopRampRate(boolean enable) {
    double rampRate = (enable ? Constants.DrivetrainConstants.rampRate : 0.0);

    _fLMotor.setOpenLoopRampRate(rampRate);
    _fRMotor.setOpenLoopRampRate(rampRate);
    _bLMotor.setOpenLoopRampRate(rampRate);
    _bRMotor.setOpenLoopRampRate(rampRate);
  }

  public void teleopDrive(Joystick driveControl) {
    double forward = applyDeadband(driveControl.getRawAxis(JoystickConstants.LEFT_STICK_Y));
    double turn = applyDeadband(driveControl.getRawAxis(JoystickConstants.RIGHT_STICK_X));

    //might need to be inverted
    _drive.arcadeDrive(forward, turn);
  }

  private double applyDeadband(double value) {
    if(Math.abs(value) < JoystickConstants.deadband)
      return 0.0;
    else
      return (value - Math.copySign(0.1, value)) /(1 - JoystickConstants.deadband);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //may need to be inverted
    _fLMotor.setVoltage(leftVolts);
    _fRMotor.setVoltage(rightVolts);
    _bLMotor.setVoltage(leftVolts);
    _bRMotor.setVoltage(rightVolts);
  }
}
