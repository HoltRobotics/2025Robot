// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
/**
 * Holds data for the climber
 */
public class Climber extends SubsystemBase {

  /*leave comments of what these lines of code do */
  SparkMax m_climberMotor = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless); // m_climberMotor is a SparkMax
  SparkMaxConfig m_climberConfig = new SparkMaxConfig(); // m_climberConfig is a SparkMaxConfig
  SparkClosedLoopController m_climberPID = m_climberMotor.getClosedLoopController(); // m_climberPID is the controller for the motor

  double m_climberPosition = m_climberMotor.getEncoder().getPosition();
  double m_setpoint = 0;

  boolean m_isEnabled = false;

  /** Creates a new Climber. */
  public Climber() {
    m_climberConfig.inverted(true);
    m_climberConfig.idleMode(IdleMode.kBrake); // idleMode for the climberConfig equals doing nothing (braking)

    m_climberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_climberConfig.closedLoop.pid(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);

    m_climberConfig.closedLoop.maxMotion.maxAcceleration(ClimberConstants.kMaxAcceleration);
    m_climberConfig.closedLoop.maxMotion.maxVelocity(ClimberConstants.kMaxVelocity);
    m_climberConfig.closedLoop.maxMotion.allowedClosedLoopError(ClimberConstants.kMaxError);
    m_climberConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    m_climberMotor.configure(m_climberConfig, null, null); // tells climberMotor that it uses climberConfig for configuration (MUST PUT LAST)
  }
  /**
   * Called periodically (Almost all the time)
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_isEnabled) {
      m_climberPID.setReference(m_setpoint, ControlType.kPosition); // tells motor where it's going
    }
  }
  /**Makes the climber fold out */
  public void climberFoldOut() {
    /*makes climber fold out */
    m_climberMotor.set(-1);
  }
  /**Makes the climber fold in */
  public void climberFoldIn() {
    /*Makes climber fold in */
    m_climberMotor.set(1);
  }
  /**Makes the Climber Stop */
  public void climberStop() {
    /*makes climber stop */
    m_climberMotor.stopMotor();
  }
  /**Sets a point
   * @param newSetpoint sets a new setpoint
   */
  public void setSetpoint(double newSetpoint) {
    /*makes m_Setpoint equal the current setpoint */
    m_setpoint = newSetpoint;
  }
  /**Obtains the setpoint
   * @return returns the current setpoint
   */
  public double getSetpoint() {
    /*obtains setpoint from m_setpoint */
    return m_setpoint;
  }
  /**
   * Gets the position of the climber
   * @return returns the climber
   */
  public double getPosition() {
    /*obtains position from m_climberPosition */
    return m_climberPosition;
  }
}

