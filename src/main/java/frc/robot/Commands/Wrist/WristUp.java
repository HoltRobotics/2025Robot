// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Subsystems.CoralManipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristUp extends Command {
  private final CoralManipulator m_manipulator;

  /** Creates a new WristUp. */
  public WristUp(CoralManipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_manipulator = manipulator;
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulator.wristUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulator.wristStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_manipulator.getPosition() > CoralManipulatorConstants.kMaxAngle) {
    //   return true;
    // } else {
    //   return false;
    // }
    return false;
  }
}
