// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.CoralWrist.SetWrist;
import frc.robot.Commands.Elevator.SetHeight;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.CoralIntake;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakePosition extends ParallelCommandGroup {
Wrist m_wrist;
Elevator m_elevator;

  /** Creates a new Intake. */
  public IntakePosition(Wrist m_wrist2, Elevator elevator) {
    m_wrist = m_wrist2;
    m_elevator = elevator;
    
    addCommands(new SetWrist(CoralManipulatorConstants.kIntakeAngle, m_wrist2));
    addCommands(new SetHeight(ElevatorConstants.kIntake, elevator));
  }
}
