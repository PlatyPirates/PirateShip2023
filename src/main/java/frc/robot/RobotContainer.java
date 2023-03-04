// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Extend;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.Retract;
import frc.robot.subsystems.Booty_Intake;
import frc.robot.subsystems.Drive_Train;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FourBarArms;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Booty_Intake.BootyState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //idk about this one
  private final ADIS16470_IMU _gyro = new ADIS16470_IMU();

  private final Drive_Train _drive_Train = new Drive_Train(_gyro);
  private final Joystick _driver = new Joystick(0);

  private final Booty_Intake _bootyIntake = new Booty_Intake();
  private final FourBarArms _fourBarArms = new FourBarArms();
  private final IntakePivot _intakePivot = new IntakePivot(); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    _drive_Train.setDefaultCommand(new ArcadeDrive(_drive_Train, _driver));
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
    new JoystickButton(_driver, JoystickConstants.BUMPER_RIGHT)
      .onTrue(new InstantCommand(() -> _bootyIntake.setState(BootyState.CubeIntake)))
      .onFalse(new InstantCommand(() -> _bootyIntake.setState(BootyState.CubeHold))); 
    //new JoystickButton(_driver, JoystickConstants.BUMPER_LEFT)
      //.onTrue(new InstantCommand(() -> _bootyIntake.setState(BootyState.ConeIntake)));
      //.onFalse(new InstantCommand(() -> _bootyIntake.setState(BootyState.ConeHold)));

    //new JoystickButton(_driver, JoystickConstants.Y).onTrue(new Extend(_fourBarArms, _intakePivot));
    //new JoystickButton(_driver, JoystickConstants.A).onTrue(new Retract(_fourBarArms, _intakePivot));

    new JoystickButton(_driver, JoystickConstants.Y).whileTrue(new RunCommand(_fourBarArms::armOut, _fourBarArms));
    new JoystickButton(_driver, JoystickConstants.A).whileTrue(new RunCommand(_fourBarArms::armIn, _fourBarArms));
    new JoystickButton(_driver, JoystickConstants.X).whileTrue(new RunCommand(_intakePivot::pivotUp, _intakePivot));
    new JoystickButton(_driver, JoystickConstants.B).whileTrue(new RunCommand(_intakePivot::pivotDown, _intakePivot));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
