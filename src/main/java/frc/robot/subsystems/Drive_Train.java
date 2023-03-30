// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // change according to gyro
  private AHRS _gyro;

  private RelativeEncoder _leftEncoder;
  private RelativeEncoder _rightEncoder;

  private DifferentialDriveOdometry _odometry;

  private Pose2d _pose;

  public Drive_Train(AHRS gyro) {

    _gyro = gyro;
    _gyro.reset();

    _fLMotor.restoreFactoryDefaults();
    _fRMotor.restoreFactoryDefaults();
    _bLMotor.restoreFactoryDefaults();
    _bRMotor.restoreFactoryDefaults();

    // modify this if the motors are spinning in different directions
    _fLMotor.setInverted(true);
    _bLMotor.setInverted(true);

    _bLMotor.follow(_fLMotor);
    _bRMotor.follow(_fRMotor);

    enableOpenLoopRampRate(true);

    _leftEncoder = _fLMotor.getEncoder();
    _rightEncoder = _fRMotor.getEncoder();

    _leftEncoder.setPositionConversionFactor((
        DrivetrainConstants.kDistancePerWheelRevolutionMeters / DrivetrainConstants.kGearReduction)*1.26);
    _rightEncoder.setPositionConversionFactor((
        DrivetrainConstants.kDistancePerWheelRevolutionMeters / DrivetrainConstants.kGearReduction)*1.26);

    encoderReset();

    _odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(-_gyro.getAngle()), -_leftEncoder.getPosition(), -_rightEncoder.getPosition());

    _bLMotor.burnFlash();
    _bRMotor.burnFlash();
    _fLMotor.burnFlash();
    _fRMotor.burnFlash();
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

    // might need to be inverted
    _drive.arcadeDrive(forward, turn);
  }

  public void drive(double forward, double turn) {
    _drive.arcadeDrive(forward, turn);
  }

  private double applyDeadband(double value) {
    if (Math.abs(value) < JoystickConstants.deadband)
      return 0.0;
    else
      return (value - Math.copySign(0.1, value)) / (1 - JoystickConstants.deadband);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _pose = _odometry.update(Rotation2d.fromDegrees(-_gyro.getAngle()), -_leftEncoder.getPosition(), -_rightEncoder.getPosition());

    SmartDashboard.putNumber("Left Encoder", -_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", -_rightEncoder.getPosition());

    SmartDashboard.putNumber("Angle", -_gyro.getAngle());
    SmartDashboard.putNumber("Pitch", _gyro.getPitch());
    SmartDashboard.putNumber("Roll", _gyro.getRoll());
    SmartDashboard.putNumber("Yaw", _gyro.getYaw());
  }

  public void encoderReset() {
    _rightEncoder.setPosition(0.0);
    _leftEncoder.setPosition(0.0);
  }

  public void resetHeading() {
    _gyro.reset();
  }

  public Pose2d getPose() {
    return _pose;
  }

  public double getPosition() {
    return -_leftEncoder.getPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-_leftEncoder.getVelocity() / 60, -_rightEncoder.getVelocity() / 60);
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    _gyro.reset();
    _odometry.resetPosition(
        Rotation2d.fromDegrees(-_gyro.getAngle()), -_leftEncoder.getPosition(), -_rightEncoder.getPosition(), _pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // may need to be inverted
    _fLMotor.setVoltage(-leftVolts);
    _fRMotor.setVoltage(-rightVolts);
    _bLMotor.setVoltage(-leftVolts);
    _bRMotor.setVoltage(-rightVolts);
  }
}
