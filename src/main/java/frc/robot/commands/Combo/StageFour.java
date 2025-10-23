// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Elevator.SetHeight;
import frc.robot.commands.Wrist.SetWrist;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StageFour extends ParallelCommandGroup {
  Elevator elevator;
  Wrist wrist;

  /** Creates a new StageFour. */
  public StageFour(Elevator elevator, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = elevator;
    this.wrist = wrist;
    addCommands(new SetHeight(elevator, ElevatorConstants.kStageFour));
    addCommands(new SetWrist(wrist, WristConstants.kStage4));
  }
}
