// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristUp extends Command {
  Wrist wrist;
  double limit = WristConstants.kMaxAngle;

  /** Creates a new WristUp. */
  public WristUp(Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.disablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.wristUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stop();
    wrist.setAngle(wrist.getPosition());
    wrist.enablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (wrist.getPosition() > limit) {
    //   return true;
    // } else {
    // return false;
    // }
    return false;
  }
}
