// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.naming.directory.SearchControls;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SysIdSwerveTranslation;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Telemetry;
import frc.robot.Commands.Climber.ClimberAngle;
import frc.robot.Commands.Climber.ClimberIn;
import frc.robot.Commands.Climber.ClimberOut;
import frc.robot.Commands.Climber.EnableClimb;
import frc.robot.Commands.Climber.ResetClimber;
import frc.robot.Commands.Combos.IntakePosition;
import frc.robot.Commands.Combos.LevelFour;
import frc.robot.Commands.Combos.LevelOne;
import frc.robot.Commands.Combos.LevelThree;
import frc.robot.Commands.Combos.LevelTwo;
import frc.robot.Commands.Combos.Stow;
import frc.robot.Commands.Elevator.MoveDown;
import frc.robot.Commands.Elevator.MoveUp;
import frc.robot.Commands.Elevator.SetHeight;
import frc.robot.Commands.Elevator.StopElevator;
import frc.robot.Commands.Swerve.RegDrive;
import frc.robot.Commands.Swerve.SlowDrive;
import frc.robot.Commands.CoralIntake.Intake;
import frc.robot.Commands.CoralIntake.Shoot;
import frc.robot.Commands.CoralIntake.SlowShoot;
import frc.robot.Commands.CoralWrist.ResetWrist;
import frc.robot.Commands.CoralWrist.WristDown;
import frc.robot.Commands.CoralWrist.WristStop;
import frc.robot.Commands.CoralWrist.WristUp;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CoralIntake;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Wrist;

@SuppressWarnings("unused")
public class RobotContainer {
    // Setting up max speeds for driving and turning
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // The value of magnitude adjusts how fast the robot turns 
    private double slowDrive = 1.0;

    // Controllers
    private final PS5Controller m_driver = new PS5Controller(Constants.OIConstants.kDriverPort);
    private final Joystick m_operator = new Joystick(Constants.OIConstants.kOperatorPort);

    // Subsystems
    public final Elevator m_elevator = new Elevator();
    public final Wrist m_wrist = new Wrist();
    public final CoralIntake m_intake = new CoralIntake();
    public final Swerve m_swerve = TunerConstants.createDrivetrain();
    public final Climber m_climber = new Climber();
    public final Telemetry logger = new Telemetry(MaxSpeed);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband (0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    
    // Shuffleboard and PathPlanner
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
    private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    /* PathPlanner Commands */
    NamedCommands.registerCommand("Level 1", new LevelOne(m_wrist, m_elevator));
    NamedCommands.registerCommand("Level 2", new LevelTwo(m_wrist, m_elevator));
    NamedCommands.registerCommand("Level 3", new LevelThree(m_wrist, m_elevator));
    NamedCommands.registerCommand("Level 4", new LevelFour(m_wrist, m_elevator));
    // NamedCommands.registerCommand("Shoot", new Shoot(m_intake));
    NamedCommands.registerCommand("Shoot", new Shoot(m_intake));
    NamedCommands.registerCommand("Slow Shoot", new SlowShoot(m_intake));
    NamedCommands.registerCommand("Intake", new IntakePosition(m_wrist, m_elevator));
    NamedCommands.registerCommand("Stow", new Stow(m_wrist, m_elevator));
    NamedCommands.registerCommand("Intake", new Intake(m_intake));

    configureButtonBindings();
    configureSwerveBindings();

    m_swerve.registerTelemetry(logger::telemeterize);

    m_autoChooser = AutoBuilder.buildAutoChooser();
    m_tab.add("Autochooser", m_autoChooser);
  }

  private void configureButtonBindings() {

    // Elevator Commands
      // PID Height Commands
        // new JoystickButton(m_operator, 6).onTrue(new SetHeight(ElevatorConstants.kStageOne, m_elevator));
        // new JoystickButton(m_operator, 7).onTrue(new SetHeight(ElevatorConstants.kStageTwo, m_elevator));
        // new JoystickButton(m_operator, 8).onTrue(new SetHeight(ElevatorConstants.kStageThree, m_elevator));
        // new JoystickButton(m_operator, 9).onTrue(new SetHeight(ElevatorConstants.kStageFour, m_elevator));
        // new JoystickButton(m_operator, 10).onTrue(new SetHeight(ElevatorConstants.kMinHeight, m_elevator));

      // Manual Elevator Commands
      new JoystickButton(m_operator, 11).whileTrue(new MoveUp(m_elevator));
      new JoystickButton(m_operator, 12).whileTrue(new MoveDown(m_elevator));
      new JoystickButton(m_operator, 22).onTrue(new StopElevator(m_elevator));

    // Intake Commands
      new JoystickButton(m_operator, 16).whileTrue(new Intake(m_intake));
      new JoystickButton(m_operator, 15).whileTrue(new Shoot(m_intake));
      new JoystickButton(m_operator, 6).whileTrue(new SlowShoot(m_intake));

    // Wrist Commands
      // Manual Wrist Commands
      new JoystickButton(m_operator, 13).whileTrue(new WristUp(m_wrist));
      new JoystickButton(m_operator, 14).whileTrue(new WristDown(m_wrist));
      new JoystickButton(m_operator, 19).onTrue(new WristStop(m_wrist));
      new JoystickButton(m_operator, 24).onTrue(new ResetWrist(m_wrist));

    // Combo Commands
      new JoystickButton(m_operator, 10).onTrue(new Stow(m_wrist, m_elevator));
      new JoystickButton(m_operator, 5).onTrue(new Intake(m_intake));
      new JoystickButton(m_operator, 1).onTrue(new LevelOne(m_wrist, m_elevator));
      new JoystickButton(m_operator, 2).onTrue(new LevelTwo(m_wrist, m_elevator));
      new JoystickButton(m_operator, 3).onTrue(new LevelThree(m_wrist, m_elevator));
      new JoystickButton(m_operator, 4).onTrue(new LevelFour(m_wrist, m_elevator));

    // Climber Commands
      // PID Climber Commands
      new JoystickButton(m_operator, 20).onTrue(new ClimberAngle(m_climber, 20));
      new JoystickButton(m_operator, 23).onTrue(new ResetClimber(m_climber));

      // Manual Climber Commands
      new JoystickButton(m_operator, 17).whileTrue( new ClimberOut(m_climber));
      new JoystickButton(m_operator, 18).whileTrue(new ClimberIn(m_climber));
      new JoystickButton(m_operator, 21).whileTrue(new EnableClimb(m_climber));
  }

  private void configureSwerveBindings() {
    m_swerve.setDefaultCommand(
      m_swerve.applyRequest(() ->
        drive.withVelocityX(-m_driver.getLeftY() * MaxSpeed * 0.6)
             .withVelocityY(-m_driver.getLeftX() * MaxSpeed * 0.6)
             .withRotationalRate(m_driver.getRightX() * MaxAngularRate * 0.6)
      )
    );

    new JoystickButton(m_driver, PS5Controller.Button.kOptions.value).onTrue(m_swerve.runOnce(() -> m_swerve.seedFieldCentric()));
    new JoystickButton(m_driver, PS5Controller.Button.kR3.value).whileTrue(m_swerve.applyRequest(() -> brake));

    // Driver Intake Controls
      new JoystickButton(m_driver, PS5Controller.Button.kL1.value).whileTrue(new Intake(m_intake));
      new JoystickButton(m_driver, PS5Controller.Button.kCross.value).whileTrue(new Shoot(m_intake));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}