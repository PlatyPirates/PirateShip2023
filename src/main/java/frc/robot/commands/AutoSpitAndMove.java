// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive_Train;
import frc.robot.subsystems.IntakePivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSpitAndMove extends SequentialCommandGroup {
  /** Creates a new AutoSpitAndMove. */
  public AutoSpitAndMove(Drive_Train driveTrain, IntakePivot intakePivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoPivotUp(intakePivot), new DriveBackward(driveTrain));
  }
}
