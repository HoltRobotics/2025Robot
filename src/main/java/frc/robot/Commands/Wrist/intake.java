// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralManipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intake extends Command {
  private final CoralManipulator m_Manipulator;
  /** Creates a new intake. */
  public intake(CoralManipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_Manipulator = manipulator;
  addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Manipulator.shooterintake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Manipulator.shootStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
