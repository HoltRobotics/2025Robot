// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Arm;

/**Manual Stop Arm command */
public class StopArm extends InstantCommand {
  private final Arm m_arm;
  
  public StopArm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.stopArm();
  }
}
