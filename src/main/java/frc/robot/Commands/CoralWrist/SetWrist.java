// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CoralWrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Wrist;

public class SetWrist extends InstantCommand {
  Wrist m_wrist;
  private double m_angle;

  public SetWrist(double angle, Wrist wrist) {
    m_wrist = wrist;
    m_angle = angle;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setAngle(m_angle);
  }
}
