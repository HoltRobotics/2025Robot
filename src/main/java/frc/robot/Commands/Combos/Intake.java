// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Elevator.SetHeight;
import frc.robot.Commands.Wrist.SetWrist;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.Subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Intake extends ParallelCommandGroup {
  Elevator m_elevator;
  CoralManipulator m_Manipulator;
  /** Creates a new Intake. */
  public Intake(Elevator elevator, CoralManipulator manipulator) {
    m_elevator = elevator;
    m_Manipulator = manipulator;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new SetHeight(ElevatorConstants.kIntake, elevator));
    addCommands(new SetWrist(CoralManipulatorConstants.kIntakeAngle, manipulator));
  }
}
