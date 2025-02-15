// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.naming.directory.SearchControls;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.CoralManipulator;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Swerve;
import frc.robot.Telemetry;
import frc.robot.Commands.Elevator.MoveDown;
import frc.robot.Commands.Elevator.MoveUp;
import frc.robot.Commands.Elevator.SetHeight;
import frc.robot.Commands.Elevator.StopElevator;
import frc.robot.Commands.Wrist.SetWrist;
import frc.robot.Commands.Wrist.WristDown;
import frc.robot.Commands.Wrist.WristStop;
import frc.robot.Commands.Wrist.WristUp;
import frc.robot.Commands.Wrist.intake;
import frc.robot.Commands.Wrist.shoot;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;

@SuppressWarnings("unused")
public class RobotContainer {
    // Setting up max speeds for driving and turning
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Controllers
    private final PS5Controller m_driver = new PS5Controller(Constants.OIConstants.kDriverPort);
    private final Joystick m_operator = new Joystick(Constants.OIConstants.kOperatorPort);

    // Subsystems
    public final Elevator m_elevator = new Elevator();
    public final CoralManipulator m_manipulator = new CoralManipulator();
    public final Swerve m_swerve = TunerConstants.createDrivetrain();
    public final Telemetry logger = new Telemetry(MaxSpeed);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public RobotContainer() {
    configureButtonBindings();
    configureSwerveBindings();

    m_swerve.registerTelemetry(logger::telemeterize);
  }

  private void configureButtonBindings() {

    // Combo Commands

    // Elevator Commands
      // PID Height Commands
    new JoystickButton(m_operator, 6).onTrue(new SetHeight(ElevatorConstants.kStageOne, m_elevator));
    new JoystickButton(m_operator, 7).onTrue(new SetHeight(ElevatorConstants.kStageTwo, m_elevator));
    new JoystickButton(m_operator, 8).onTrue(new SetHeight(ElevatorConstants.kStageThree, m_elevator));
    new JoystickButton(m_operator, 9).onTrue(new SetHeight(ElevatorConstants.kStageFour, m_elevator));
    new JoystickButton(m_operator, 10).onTrue(new SetHeight(ElevatorConstants.kMinHeight, m_elevator));

      // Manual Elevator Commands
    new JoystickButton(m_operator, 11).whileTrue(new MoveUp(m_elevator));
    new JoystickButton(m_operator, 12).whileTrue(new MoveDown(m_elevator));
    new JoystickButton(m_operator, 22).onTrue(new StopElevator(m_elevator));

    // Wrist Commands
      // PID Wrist Commands
    new JoystickButton(m_operator, 17).onTrue(new SetWrist(m_manipulator, CoralManipulatorConstants.kStage1Angle));
    new JoystickButton(m_operator, 18).onTrue(new SetWrist(m_manipulator, CoralManipulatorConstants.kIntakeAngle));
    new JoystickButton(m_operator, 19).onTrue(new SetWrist(m_manipulator, CoralManipulatorConstants.kStowAngle));

      // Manual Wrist Commands
    new JoystickButton(m_operator, 13).whileTrue(new WristUp(m_manipulator));
    new JoystickButton(m_operator, 14).whileTrue(new WristDown(m_manipulator));
    new JoystickButton(m_operator, 20).onTrue(new WristStop(m_manipulator));
    new JoystickButton(m_operator, 15).whileTrue(new shoot(m_manipulator));
    new JoystickButton(m_operator, 16).whileTrue(new intake(m_manipulator));

    // Climber Commands
      // PID Climber Commands

      // Manual Climber Commands
  }

  private void configureSwerveBindings() {
    m_swerve.setDefaultCommand(
      m_swerve.applyRequest(() ->
        drive.withVelocityX(m_driver.getLeftY() * MaxSpeed)
             .withVelocityY(m_driver.getLeftX() * MaxSpeed)
             .withRotationalRate(-m_driver.getRightX() * MaxAngularRate)
      )
    );

    new JoystickButton(m_driver, PS5Controller.Button.kOptions.value).onTrue(m_swerve.runOnce(() -> m_swerve.seedFieldCentric()));
    new JoystickButton(m_driver, PS5Controller.Button.kR3.value).whileTrue(m_swerve.applyRequest(() -> brake));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
