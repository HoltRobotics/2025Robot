// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Elevator extends SubsystemBase {

  SparkMax m_elevator = new SparkMax(ElevatorConstants.kElevatorID, MotorType.kBrushless);

  SparkMaxConfig m_elevatorConfig = new SparkMaxConfig();

  SparkClosedLoopController m_elevatorPID = m_elevator.getClosedLoopController();

  double m_setPoint = 0;

  double m_position = m_elevator.getEncoder().getPosition();

  boolean m_isEnabled = false;

  /** Creates a new Elevator. */
  public Elevator() {

    m_elevatorConfig.inverted(false);
    m_elevatorConfig.idleMode(IdleMode.kBrake);
    
    m_elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_elevatorConfig.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

    m_elevatorConfig.closedLoop.maxMotion.maxAcceleration(ElevatorConstants.kMaxAcceleration);
    m_elevatorConfig.closedLoop.maxMotion.maxVelocity(ElevatorConstants.kMaxVelocity);
    m_elevatorConfig.closedLoop.maxMotion.allowedClosedLoopError(ElevatorConstants.kMaxError);

    m_elevatorConfig.encoder.positionConversionFactor(ElevatorConstants.kConvertionFactor);

    m_elevator.configure(m_elevatorConfig, null, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    if (m_isEnabled) {
      m_elevatorPID.setReference(m_setPoint, ControlType.kMAXMotionPositionControl);
    }
    m_position = m_elevator.getEncoder().getPosition();
    // This method will be called once per scheduler run
  }

  public void setheight(double height) {
    m_elevatorPID.setReference(m_setPoint, ControlType.kMAXMotionPositionControl);
  }

  public void MoveUp() {
    m_elevator.set(0.3);
  }

  public void MoveDown() {
    m_elevator.set(-0.3);
  }

  public void EnablePID() {
    m_isEnabled = true;
  }

  public void DisablePID() {
    m_isEnabled = false;
  }

  public void ElevatorStop() {
    m_elevator.stopMotor();
  }

  public double getPosition() {
    return m_position;
  }

}
