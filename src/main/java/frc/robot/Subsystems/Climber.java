// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  SparkMax m_ClimberMotorOne = new SparkMax(ClimberConstants.kClimberMotorOneID, MotorType.kBrushless);
  SparkMax m_ClimberMotorTwo = new SparkMax(ClimberConstants.kClimberMotorTwoID, MotorType.kBrushless);

  RelativeEncoder m_encoderOne = m_ClimberMotorOne.getEncoder();
  RelativeEncoder m_encoderTwo = m_ClimberMotorTwo.getEncoder();

  SparkClosedLoopController m_controllerOne = m_ClimberMotorOne.getClosedLoopController();
  SparkClosedLoopController m_controllerTwo = m_ClimberMotorTwo.getClosedLoopController();

  SparkMaxConfig m_ClimberConfigOne = new SparkMaxConfig();
  SparkMaxConfig m_ClimberConfigTwo = new SparkMaxConfig();

  double m_ClimberOnePosition = m_ClimberMotorTwo.getEncoder().getPosition();
  double m_ClimberTwoPosition = m_ClimberMotorTwo.getEncoder().getPosition();
  double m_setpoint;

  boolean m_climbEnabled = false;

  /** Creates a new Climber. */
  public Climber() {
    m_ClimberConfigOne
      .idleMode(IdleMode.kBrake)
      .inverted(true);

    m_ClimberConfigTwo
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    m_ClimberConfigOne.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.1, 0, 0);

    m_ClimberConfigTwo.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.1, 0, 0);

    m_ClimberMotorOne.configure(m_ClimberConfigOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // telling ClimberMotor that it uses ClimberConfig
    m_ClimberMotorTwo.configure(m_ClimberConfigTwo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // telling ClimberMotor that it uses ClimberConfig
 
    m_ClimberMotorOne.getEncoder().setPosition(0);
    m_ClimberMotorTwo.getEncoder().setPosition(0);

    m_setpoint = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ClimberOnePosition = m_ClimberMotorOne.getEncoder().getPosition();
    m_ClimberTwoPosition = m_ClimberMotorTwo.getEncoder().getPosition();

    SmartDashboard.putNumber("Climber Goal Angle", m_setpoint);
    SmartDashboard.putNumber("Climber Actual Angle", ((m_ClimberOnePosition * m_ClimberTwoPosition) / 2));
    SmartDashboard.putBoolean("Climber Enabled", m_climbEnabled);
  }

  public void in() {
    if (m_climbEnabled) {
      m_ClimberMotorOne.set(-0.5);
      m_ClimberMotorTwo.set(-0.5);
    } else {
      m_ClimberMotorOne.set(0);
      m_ClimberMotorTwo.set(0);
    }
  }

  public void out() {
    m_ClimberMotorOne.set(0.5);
    m_ClimberMotorTwo.set(0.5);
  }

  public void stop() {
    m_ClimberMotorOne.set(0);
    m_ClimberMotorTwo.set(0);
  }

  public void setClimberAngle(double angle) {
    m_setpoint = angle;
    m_ClimberMotorOne.getClosedLoopController().setReference(angle, ControlType.kPosition);
    m_ClimberMotorTwo.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  public void resetClimber() {
    m_ClimberMotorOne.getEncoder().setPosition(0);
    m_ClimberMotorTwo.getEncoder().setPosition(0);
  }

  public void climbEnable() {
    m_climbEnabled = true;
  }
}
