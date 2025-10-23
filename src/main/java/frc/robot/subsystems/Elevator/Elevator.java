// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  SparkMax elevator = new SparkMax(ElevatorConstants.kElevatorID, MotorType.kBrushless);
  SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  SparkClosedLoopController elevatorPID = elevator.getClosedLoopController();

  ShuffleboardTab tab = Shuffleboard.getTab("Main");

  double setPoint;
  double position = elevator.getEncoder().getPosition();
  boolean isEnabled = false;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorConfig.inverted(true);
    elevatorConfig.idleMode(IdleMode.kBrake);

    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    elevatorConfig.closedLoop.pid(
      ElevatorConstants.kP,
      ElevatorConstants.kI,
      ElevatorConstants.kD);

    elevatorConfig.closedLoop.maxMotion
      .maxAcceleration(ElevatorConstants.kMaxAcceleration)
      .maxVelocity(ElevatorConstants.kMaxVelocity)
      .allowedClosedLoopError(ElevatorConstants.kMaxError)
      .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    elevatorConfig.encoder.positionConversionFactor(ElevatorConstants.kConversionFactor);

    elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevator.getEncoder().setPosition(0);
  }

  public void setHeight(double height) {
    setPoint = height;
    elevatorPID.setReference(height, ControlType.kMAXMotionPositionControl);
  }

  public void moveUp() {
    elevator.set(0.35);
  }

  public void moveDown() {
    elevator.set(-0.35);
  }

  public void enablePID() {
    isEnabled = true;
  }

  public void disablePID() {
    isEnabled = false;
  }

  public void stop() {
    elevator.stopMotor();
  }

  public double getPosition() {
    return position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isEnabled) {
      elevatorPID.setReference(setPoint, ControlType.kPosition);
    }

    position = elevator.getEncoder().getPosition();
    SmartDashboard.putNumber("Elevator Height", position);
  }
}
