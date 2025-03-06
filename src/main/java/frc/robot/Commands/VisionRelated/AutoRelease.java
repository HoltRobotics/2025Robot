// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.VisionRelated;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Subsystems.AlgeaManipulator;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision.Limelight;

/**
 * @deprecated Please do not use this, it is unfinished
 */
public class AutoRelease extends SequentialCommandGroup {
  /** Creates a new AutoRelease. */
  public AutoRelease(CoralManipulator coral, Swerve swerve, Limelight liiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiight, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      /*new ParallelDeadlineGroup(
         )*/
    );
  }
}
