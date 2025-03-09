// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.CoralWrist.SetWrist;
import frc.robot.Commands.Elevator.SetHeight;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LevelThree extends ParallelCommandGroup {
  Wrist m_wrist;
  Elevator m_elevator;

  /** Creates a new LevelThree. */
  public LevelThree(Wrist wrist, Elevator elevator) {
    m_wrist = wrist;
    m_elevator = elevator;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetWrist(CoralManipulatorConstants.kStage3Angle, wrist));
    addCommands(new SetHeight(ElevatorConstants.kStageThree, elevator));
  }
}
