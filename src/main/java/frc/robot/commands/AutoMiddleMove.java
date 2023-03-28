// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Booty_Intake;
import frc.robot.subsystems.Drive_Train;
import frc.robot.subsystems.IntakePivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMiddleMove extends SequentialCommandGroup {
  //start middle, spit cargo, drive out of community over charge station, drive back onto the charge station and stabilize
  
  public AutoMiddleMove(Drive_Train driveTrain, IntakePivot intakePivot, Booty_Intake bootyIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoPivotUp(intakePivot), new AutoIntakeOut(bootyIntake), new AutoPivotDown(intakePivot), new DriveBackwardCharge(driveTrain));
  }
}
