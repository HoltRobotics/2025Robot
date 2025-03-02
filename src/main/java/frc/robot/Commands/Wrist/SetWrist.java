// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.CoralManipulator;

public class SetWrist extends InstantCommand {
  private CoralManipulator m_manipulator;
  private double m_angle;

  public SetWrist(double angle, CoralManipulator manipulator) {
    m_manipulator = manipulator;
    m_angle = angle;
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SetWrist is running");
    m_manipulator.enablePID();
    m_manipulator.setAngle(m_angle);
  }
}
