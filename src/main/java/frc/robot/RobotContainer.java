// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Climber.In;
import frc.robot.commands.Climber.Out;
import frc.robot.commands.Climber.StopClimber;
import frc.robot.commands.Combo.SetIntake;
import frc.robot.commands.Combo.StageFour;
import frc.robot.commands.Combo.StageOne;
import frc.robot.commands.Combo.StageThree;
import frc.robot.commands.Combo.StageTwo;
import frc.robot.commands.Combo.Stow;
import frc.robot.commands.Elevator.ElevatorDown;
import frc.robot.commands.Elevator.ElevatorUp;
import frc.robot.commands.Elevator.SetHeight;
import frc.robot.commands.Elevator.StopElevator;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.RunOuttake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Wrist.ResetWrist;
import frc.robot.commands.Wrist.StopWrist;
import frc.robot.commands.Wrist.WristDown;
import frc.robot.commands.Wrist.WristUp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Climber climber;
  private final Drive drive;
  private final Elevator elevator;
  private final Intake intake;
  private final Wrist wrist;

  private SwerveDriveSimulation driveSimulation = null;

  // Controllers
  private final CommandPS5Controller ps5 = new CommandPS5Controller(Constants.kPS5);
  private final CommandXboxController xbox = new CommandXboxController(Constants.kXBOX);
  private final CommandJoystick translation = new CommandJoystick(Constants.kFlight1);
  private final CommandJoystick rotation = new CommandJoystick(Constants.kFlight2);
  private final CommandJoystick panel = new CommandJoystick(Constants.kPanel);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        climber = new Climber();
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (pose) -> {});
        elevator = new Elevator();
        intake = new Intake();
        wrist = new Wrist();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        climber = new Climber();
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        elevator = new Elevator();
        intake = new Intake();
        wrist = new Wrist();
        break;

      default:
        // Replayed robot, disable IO implementations
        climber = new Climber();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        elevator = new Elevator();
        intake = new Intake();
        wrist = new Wrist();
        break;
    }

    // Set up auto routines
    
    // Sut up auto commands
    NamedCommands.registerCommand("Stage 4", new StageFour(elevator, wrist));
    NamedCommands.registerCommand("EUP", new ElevatorUp(elevator));
    NamedCommands.registerCommand("Stow", new Stow(elevator, wrist));
    NamedCommands.registerCommand("Outtake", new RunOuttake(intake));

    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Initializes the resetGyro command
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    // Swtich case to change the buttons and presets based on the drivers and scenarios
    switch (Constants.currentDriver) {
      case HENRY:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> ps5.getLeftY() * -0.7,
                () -> ps5.getLeftX() * -0.7,
                () -> -ps5.getRightX() * 0.6,
                0.3,
                ps5.R1()));

        ps5.R3().onTrue(Commands.runOnce(drive::stopWithX, drive));

        ps5.options().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));


        // Operator Commands
          // Elevator Commands
            // PID Height Control
            panel.button(6).onTrue(new SetHeight(elevator, ElevatorConstants.kStageOne));
            panel.button(7).onTrue(new SetHeight(elevator, ElevatorConstants.kStageTwo));
            panel.button(8).onTrue(new SetHeight(elevator, ElevatorConstants.kStageThree));
            panel.button(9).onTrue(new SetHeight(elevator, ElevatorConstants.kStageFour));

            // Manual Control
            panel.button(11).whileTrue(new ElevatorUp(elevator)).onFalse(new StopElevator(elevator));
            panel.button(12).whileTrue(new ElevatorDown(elevator)).onFalse(new StopElevator(elevator));
            panel.button(22).onTrue(new StopElevator(elevator));


          // Wrist Commands
            // PID Angle Control
            panel.button(24).onTrue(new ResetWrist(wrist));
            ps5.triangle().whileTrue(new WristUp(wrist)).onFalse(new StopWrist(wrist));
            ps5.square().onTrue(new WristDown(wrist)).onFalse(new StopWrist(wrist));

            // Manual Control
            panel.button(13).whileTrue(new WristUp(wrist)).onFalse(new StopWrist(wrist));
            panel.button(14).whileTrue(new WristDown(wrist)).onFalse(new StopWrist(wrist));
            panel.button(20).onTrue(new StopWrist(wrist));


          // Intake Commands
            panel.button(16).whileTrue(new RunIntake(intake)).onFalse(new StopIntake(intake));
            ps5.L1().whileTrue(new RunIntake(intake)).onFalse(new StopIntake(intake));
            panel.button(15).whileTrue(new RunOuttake(intake)).onFalse(new StopIntake(intake));
            ps5.cross().whileTrue(new RunOuttake(intake)).onFalse(new StopIntake(intake));

          
          // Climber Commands
            panel.button(17).whileTrue(new Out(climber)).onFalse(new StopClimber(climber));
            panel.button(18).whileTrue(new In(climber)).onFalse(new StopClimber(climber));


          // Combo Commands
            panel.button(1).onTrue(new StageOne());
            panel.button(2).onTrue(new StageTwo(elevator, wrist));
            panel.button(3).onTrue(new StageThree(elevator, wrist));
            panel.button(4).onTrue(new StageFour(elevator, wrist));
            panel.button(5).onTrue(new SetIntake(elevator, wrist));
            panel.button(10).onTrue(new Stow(elevator, wrist));
        break;

      case PROGRAMMING:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> xbox.getLeftY() * 0.7,
                () -> xbox.getLeftX() * 0.7,
                () -> -xbox.getRightX() * 0.6,
                0.15,
                xbox.rightBumper()));

        xbox.rightStick().onTrue(Commands.runOnce(drive::stopWithX, drive));

        xbox.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        

        // Operator Commands
          // Elevator Commands
            // PID Height Control
            panel.button(6).onTrue(new SetHeight(elevator, ElevatorConstants.kStageOne));
            panel.button(7).onTrue(new SetHeight(elevator, ElevatorConstants.kStageTwo));
            panel.button(8).onTrue(new SetHeight(elevator, ElevatorConstants.kStageThree));
            panel.button(9).onTrue(new SetHeight(elevator, ElevatorConstants.kStageFour));

            // Manual Control
            panel.button(11).whileTrue(new ElevatorUp(elevator)).onFalse(new StopElevator(elevator));
            panel.button(12).whileTrue(new ElevatorDown(elevator)).onFalse(new StopElevator(elevator));
            panel.button(22).onTrue(new StopElevator(elevator));


          // Wrist Commands
            // PID Angle Control
            panel.button(24).onTrue(new ResetWrist(wrist));
            xbox.y().whileTrue(new WristUp(wrist)).onFalse(new StopWrist(wrist));
            xbox.x().whileTrue(new WristDown(wrist)).onFalse(new StopWrist(wrist));

            // Manual Control
            panel.button(13).whileTrue(new WristUp(wrist)).onFalse(new StopWrist(wrist));
            panel.button(14).whileTrue(new WristDown(wrist)).onFalse(new StopWrist(wrist));
            panel.button(20).onTrue(new StopWrist(wrist));


          // Intake Commands
            panel.button(16).whileTrue(new RunIntake(intake)).onFalse(new StopIntake(intake));
            xbox.leftBumper().whileTrue(new RunIntake(intake)).onFalse(new StopIntake(intake));
            panel.button(15).whileTrue(new RunOuttake(intake)).onFalse(new StopIntake(intake));
            xbox.a().whileTrue(new RunOuttake(intake)).onFalse(new StopIntake(intake));

          
          // Climber Commands
            panel.button(18).whileTrue(new Out(climber)).onFalse(new StopClimber(climber));
            panel.button(19).whileTrue(new In(climber)).onFalse(new StopClimber(climber));


          // Combo Commands
            panel.button(1).onTrue(new StageOne());
            panel.button(2).onTrue(new StageTwo(elevator, wrist));
            panel.button(3).onTrue(new StageThree(elevator, wrist));
            panel.button(4).onTrue(new StageFour(elevator, wrist));
            panel.button(5).onTrue(new SetIntake(elevator, wrist));
            panel.button(10).onTrue(new Stow(elevator, wrist));
        break;

        

      case DEMO:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> ps5.getLeftY() * -0.25,
                () -> ps5.getLeftX() * -0.25,
                () -> ps5.getRightX() * 0.2,
                0.1,
                ps5.R1()));

        ps5.R3().onTrue(Commands.runOnce(drive::stopWithX, drive));

        ps5.options().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        break;

      case PRAVAKAR:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -translation.getY() * 0.9,
                () -> -translation.getX() * 0.9,
                () -> -rotation.getX() * 0.6,
                0.3,
                translation.button(4)));

        rotation.button(3).whileTrue(Commands.runOnce(drive::stopWithX, drive));

        translation.button(3).onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        

        // Operator Commands
          // Elevator Commands
            // PID Height Control
            panel.button(6).onTrue(new SetHeight(elevator, ElevatorConstants.kStageOne));
            panel.button(7).onTrue(new SetHeight(elevator, ElevatorConstants.kStageTwo));
            panel.button(8).onTrue(new SetHeight(elevator, ElevatorConstants.kStageThree));
            panel.button(9).onTrue(new SetHeight(elevator, ElevatorConstants.kStageFour));

            // Manual Control
            panel.button(11).whileTrue(new ElevatorUp(elevator)).onFalse(new StopElevator(elevator));
            panel.button(12).whileTrue(new ElevatorDown(elevator)).onFalse(new StopElevator(elevator));
            panel.button(22).onTrue(new StopElevator(elevator));


          // Wrist Commands
            // PID Angle Control
            panel.button(24).onTrue(new ResetWrist(wrist));
            translation.button(2).whileTrue(new WristUp(wrist)).onFalse(new StopWrist(wrist));
            rotation.button(2).whileTrue(new WristDown(wrist)).onFalse(new StopWrist(wrist));

            // Manual Control
            panel.button(13).whileTrue(new WristUp(wrist)).onFalse(new StopWrist(wrist));
            panel.button(14).whileTrue(new WristDown(wrist)).onFalse(new StopWrist(wrist));
            panel.button(20).onTrue(new StopWrist(wrist));


          // Intake Commands
            panel.button(16).whileTrue(new RunIntake(intake)).onFalse(new StopIntake(intake));
            rotation.button(1).whileTrue(new RunIntake(intake)).onFalse(new StopIntake(intake));
            panel.button(15).whileTrue(new RunOuttake(intake)).onFalse(new StopIntake(intake));
            translation.button(1).whileTrue(new RunOuttake(intake)).onFalse(new StopIntake(intake));

          
          // Climber Commands
            panel.button(18).whileTrue(new Out(climber)).onFalse(new StopClimber(climber));
            panel.button(19).whileTrue(new In(climber)).onFalse(new StopClimber(climber));


          // Combo Commands
            panel.button(1).onTrue(new StageOne());
            panel.button(2).onTrue(new StageTwo(elevator, wrist));
            panel.button(3).onTrue(new StageThree(elevator, wrist));
            panel.button(4).onTrue(new StageFour(elevator, wrist));
            panel.button(5).onTrue(new SetIntake(elevator, wrist));
            panel.button(10).onTrue(new Stow(elevator, wrist));

        break;

      default:
        break;
    }

    // // Lock to 0° when A button is held
    // ps5
    //     .cross()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> ps5.getLeftY(),
    //             () -> ps5.getLeftX(),
    //             () -> new Rotation2d()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return autoChooser.getSelected();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
