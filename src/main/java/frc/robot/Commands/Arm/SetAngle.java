// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Arm;

/**Automatically moves the arm to the angle of your desire */
public class SetAngle extends InstantCommand {
  private double m_angle;
  private Arm m_arm;
  public SetAngle(double angle, Arm arm) {
    m_angle = angle;
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setInMotion(true);
    m_arm.setAngle(m_angle);
  }
}
