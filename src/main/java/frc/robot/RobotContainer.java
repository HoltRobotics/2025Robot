// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    // Setting up max speeds for driving and turning
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Controllers
    private final PS5Controller m_driver = new PS5Controller(Constants.OIConstants.kDriverPort);
    private final Joystick m_operator = new Joystick(Constants.OIConstants.kOperatorPort);

    // Subsystems
    public final Swerve m_swerve = TunerConstants.createDrivetrain();
    public final Telemetry logger = new Telemetry(MaxSpeed);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // Setting up the Auton Chooser box and Shuffleboard
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
    private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser("Tests");
    m_tab.add("Auto Chooser", m_autoChooser);
    
    configureButtonBindings();
    configureSwerveBindings();

    m_swerve.registerTelemetry(logger::telemeterize);
  }

  private void configureButtonBindings() {

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