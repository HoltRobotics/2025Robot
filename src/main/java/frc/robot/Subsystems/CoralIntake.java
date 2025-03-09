// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralIntake extends SubsystemBase {
  SparkMax m_shooter = new SparkMax(CoralManipulatorConstants.kIntakeMotorID, MotorType.kBrushless);

  SparkMaxConfig m_shooterConfig = new SparkMaxConfig();

  boolean m_intakeRunning = false;

  /** Creates a new shooter. */
  public CoralIntake() {
    m_shooterConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    m_shooter.configure(m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Running?", m_intakeRunning);
    }


  public void intake() {
    m_shooter.set(-.3);
    m_intakeRunning = true;
    //set angle
  }

public void stop() {
  m_shooter.stopMotor();
  m_intakeRunning = false;
}

  public void shoot() {
    m_shooter.set(1);
    //set angle
  }

  public void slowShoot() {
    m_shooter.set(0.38);
  }
}
