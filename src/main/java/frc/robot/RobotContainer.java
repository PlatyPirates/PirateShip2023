// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Booty_Intake.BootyState;
import frc.robot.subsystems.LEDStrip.LED_MODE;
import frc.robot.util.Settings;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //idk about this one
  private final AHRS _gyro = new AHRS();

  private final Drive_Train _drive_Train = new Drive_Train(_gyro);
  private final Joystick _driver = new Joystick(0);
  private final Joystick _operator = new Joystick(1);

  private final Booty_Intake _bootyIntake = new Booty_Intake();
  private final FourBarArms _fourBarArms = new FourBarArms();
  private final IntakePivot _intakePivot = new IntakePivot(); 

  public final LEDStrip LEDs = new LEDStrip(28, 0, LED_MODE.BounceAlliance);

  private SendableChooser<String> _chooser = new SendableChooser<String>();
  private String _autoSelected;
  private double _autoDelay = 0.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    _drive_Train.setDefaultCommand(new ArcadeDrive(_drive_Train, _driver));

    _chooser.setDefaultOption("Nothing", "do nothing");
    // _chooser.setDefaultOption("Test Straight", "Test Straight");
    // _chooser.addOption("Test Curve", "Test Curve");
    // _chooser.setDefaultOption("Drive forward", "drive forward");
    _chooser.addOption("Cube Only", "CubeOnly");
    _chooser.addOption("Move", "SpitAndMove");
    _chooser.addOption("Balance", "SpitAndBalanced");
    _chooser.addOption("Mobility & Balance", "MobilityAndBalance");

    SmartDashboard.putData("Auto choices", _chooser);

    SmartDashboard.putNumber("Auto Delay", _autoDelay);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(_operator, JoystickConstants.BUMPER_RIGHT)
      .onTrue(new InstantCommand(() -> _bootyIntake.setState(BootyState.CubeIntake)))
      .onFalse(new InstantCommand(() -> _bootyIntake.setState(BootyState.CubeHold))); 
    new JoystickButton(_operator, JoystickConstants.BUMPER_LEFT)
      .onTrue(new InstantCommand(() -> _bootyIntake.setState(BootyState.ConeIntake)))
      .onFalse(new InstantCommand(() -> _bootyIntake.setState(BootyState.ConeHold)));

    //new JoystickButton(_driver, JoystickConstants.Y).onTrue(new Extend(_fourBarArms, _intakePivot));
    //new JoystickButton(_driver, JoystickConstants.A).onTrue(new Retract(_fourBarArms, _intakePivot));

    new JoystickButton(_operator, JoystickConstants.Y).whileTrue(new RunCommand(_fourBarArms::armOut, _fourBarArms));
    new JoystickButton(_operator, JoystickConstants.A).whileTrue(new RunCommand(_fourBarArms::armIn, _fourBarArms));
    new JoystickButton(_operator, JoystickConstants.X).whileTrue(new RunCommand(_intakePivot::pivotUp, _intakePivot));
    new JoystickButton(_operator, JoystickConstants.B).whileTrue(new RunCommand(_intakePivot::pivotDown, _intakePivot));

    new JoystickButton(_operator, JoystickConstants.LOGO_LEFT)
      .onTrue(new InstantCommand(() -> LEDs.setColor(new Color(255, 120, 0))))
      .onFalse(new InstantCommand(() -> LEDs.setMode(LED_MODE.Alliance)));
    new JoystickButton(_operator, JoystickConstants.LOGO_RIGHT)
      .onTrue(new InstantCommand(() -> LEDs.setColor(new Color(100, 0, 127))))
      .onFalse(new InstantCommand(() -> LEDs.setMode(LED_MODE.Alliance)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    _autoSelected = _chooser.getSelected();
    System.out.println("Auto selected: " + _autoSelected);
    SmartDashboard.putString("Auto Choice", _autoSelected);
    _autoDelay = SmartDashboard.getNumber("Auto Delay", 0.0);

    _drive_Train.encoderReset();
    _drive_Train.resetHeading();

    if (_autoSelected == "drive forward") {
      return Autos.DriveForward(_drive_Train);
    } 
    else if (_autoSelected == "CubeOnly"){
      return new AutoPivotUp(_intakePivot)
        .andThen(new AutoPivotDown(_intakePivot));
    }
    else if(_autoSelected == "SpitAndMove"){
      return new WaitCommand(_autoDelay)
        .andThen(new AutoSpitAndMove(_drive_Train, _intakePivot));
    }
    else if(_autoSelected == "SpitAndBalanced"){
      return new WaitCommand(_autoDelay)
        .andThen(new AutoPivotUp(_intakePivot))
        .andThen(new AutoPivotDown(_intakePivot))
        .andThen(new DriveBackwardCharge(_drive_Train)
        .andThen((new StabilizePID(_drive_Train, _gyro).alongWith(new ArmIn(_fourBarArms)))));
    }
    else if(_autoSelected == "MobilityAndBalance") {
      return new WaitCommand(_autoDelay)
        .andThen(new AutoPivotUp(_intakePivot))
        .andThen(new AutoPivotDown(_intakePivot))
        .andThen(new DriveOverChargeStation(_drive_Train))
        .andThen(new Turn180(_drive_Train, _gyro))
        .andThen(new DriveBackwardChargeShort(_drive_Train))
        .andThen((new StabilizePID(_drive_Train, _gyro).alongWith(new ArmIn(_fourBarArms))));
  }
    // else if(_autoSelected == "Test Straight"){
    //   return Autos.TestStraight(_drive_Train);
    // }
    // else if(_autoSelected == "Test Curve"){
    //   return Autos.TestCurve(_drive_Train);
    // }
    else {
      return null;
    }
  }

  public void loadSettings() {
    _drive_Train.setBalancePID(
      Settings.loadDouble("Balance", "P", DrivetrainConstants.balancePID.getP()),
      Settings.loadDouble("Balance", "I", DrivetrainConstants.balancePID.getI()),
      Settings.loadDouble("Balance", "D", DrivetrainConstants.balancePID.getD())
    );

    _drive_Train.setBalancePowerMin(Settings.loadDouble("Balance", "Min", DrivetrainConstants.balancePowerMin));
    _drive_Train.setBalancePowerMax(Settings.loadDouble("Balance", "Max", DrivetrainConstants.balancePowerMax));
    _drive_Train.setBalanceDistance(Settings.loadDouble("Balance", "Distance", DrivetrainConstants.balanceDistance));
    _drive_Train.setBalanceDistance(Settings.loadDouble("Balance", "Mobility", DrivetrainConstants.mobilityDistance));
    _drive_Train.setBalanceDistance(Settings.loadDouble("Auto", "TurnPower", DrivetrainConstants.autoTurnPower));
    _drive_Train.setBalanceDistance(Settings.loadDouble("Balance", "DistanceShort", DrivetrainConstants.balanceDistanceShort));
  }  

  public void saveSettings(){
    Settings.saveDouble("Balance", "P", _drive_Train.getBalancePID().getP());
    Settings.saveDouble("Balance", "I", _drive_Train.getBalancePID().getI());
    Settings.saveDouble("Balance", "D", _drive_Train.getBalancePID().getD());
    Settings.saveDouble("Balance", "Min", _drive_Train.getBalancePowerMin());
    Settings.saveDouble("Balance", "Max", _drive_Train.getBalancePowerMax());
    Settings.saveDouble("Balance", "Distance", _drive_Train.getBalanceDistance());
    Settings.saveDouble("Balance", "Mobility", _drive_Train.getMobilityDistance());
    Settings.saveDouble("Auto", "TurnPower", _drive_Train.getAutoTurnPower());
    Settings.saveDouble("Balance", "DistanceShort", _drive_Train.getBalanceDistanceShort());
  }
}
