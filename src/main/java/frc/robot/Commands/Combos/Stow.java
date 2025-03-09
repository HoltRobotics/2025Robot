// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Combos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stow extends InstantCommand {
  Wrist m_wrist;
  Elevator m_elevator;

  public Stow(Wrist wrist, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    m_elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setheight(ElevatorConstants.kMinHeight);
    m_wrist.setAngle(CoralManipulatorConstants.kStowAngle);
  }
}
