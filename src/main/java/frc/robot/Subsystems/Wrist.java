// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralManipulatorConstants;

public class Wrist extends SubsystemBase {
  SparkMax m_wrist = new SparkMax(Constants.CoralManipulatorConstants.kWristMotorID, MotorType.kBrushless);

  RelativeEncoder m_encoder = m_wrist.getEncoder();

  SparkClosedLoopController m_controller = m_wrist.getClosedLoopController();

  SparkMaxConfig m_config = new SparkMaxConfig();

  double m_setpoint = 0;
  double m_position = m_wrist.getEncoder().getPosition();
  boolean m_running = false;

  /** Creates a new Wrist. */
  public Wrist() {
    m_config
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    m_config.encoder
      .positionConversionFactor(CoralManipulatorConstants.kConvertionFactor);

    m_config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(CoralManipulatorConstants.kP, CoralManipulatorConstants.kI, CoralManipulatorConstants.kD)
      .maxMotion
        .maxAcceleration(CoralManipulatorConstants.kMaxAcceleration)
        .maxVelocity(CoralManipulatorConstants.kMaxVelocity)
        .allowedClosedLoopError(CoralManipulatorConstants.kMaxAllowedError)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    m_wrist.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_wrist.getEncoder().setPosition(0);  
  }

  public void up() {
    m_running = true;
    m_wrist.set(0.2);
  }

  public void down() {
    m_running = true;
    m_wrist.set(-0.2);
  }

  public void stop() {
    m_running = false;
    m_wrist.set(0);
  }

  public void setAngle(double angle) {
    m_controller.setReference(angle, ControlType.kMAXMotionPositionControl);
    while (m_wrist.getEncoder().getVelocity() > 0) {
      m_running = true;
    }
  }

  public void resetWrist() {
    m_wrist.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_position = m_wrist.getEncoder().getPosition();

    SmartDashboard.putNumber("Wrist Actual Angle", m_position);
    SmartDashboard.putNumber("Wrist Goal Angle", m_setpoint);
    SmartDashboard.putBoolean("Wrist Running", m_running);

    if (m_position > CoralManipulatorConstants.kMaxAngle) {
      setAngle(CoralManipulatorConstants.kMaxAngle);
    } else if (m_position < CoralManipulatorConstants.kMinAngle) {
      setAngle(CoralManipulatorConstants.kMinAngle);
    }

    while (m_wrist.getEncoder().getVelocity() > 0) {
      m_running = true;
    }
  }
}
