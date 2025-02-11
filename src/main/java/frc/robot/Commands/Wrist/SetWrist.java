// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.CoralManipulator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetWrist extends InstantCommand {
  private final CoralManipulator m_manipulator;
  private final double m_angle;

  public SetWrist(CoralManipulator manipulator, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulator = manipulator;
    m_angle = angle;
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulator.setAngle(m_angle);
  }
}
