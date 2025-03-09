// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CoralWrist.SetWrist;
import frc.robot.Commands.Elevator.SetHeight;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LevelTwo extends ParallelCommandGroup {
  Wrist m_wrist;
  Elevator m_Elevator;

  /** Creates a new LevelTwo. */
  public LevelTwo(Wrist wrist, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // System.out.println("Getting through command group");
    addCommands(new SetWrist(CoralManipulatorConstants.kStage2Angle, wrist));
    addCommands(new WaitCommand(1));
    addCommands(new SetHeight(ElevatorConstants.kStageTwo, elevator));
  }
}
