// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  SparkMax m_ClimberMotorOne = new SparkMax(ClimberConstants.kClimberMotorOneID, MotorType.kBrushless);
  SparkMax m_ClimberMotorTwo = new SparkMax(ClimberConstants.kClimberMotorTwoID, MotorType.kBrushless);

  SparkMaxConfig m_ClimberConfigOne = new SparkMaxConfig();
  SparkMaxConfig m_ClimberConfigTwo = new SparkMaxConfig();

  double m_ClimberOnePosition = m_ClimberMotorTwo.getEncoder().getPosition();
  double m_ClimberTwoPosition = m_ClimberMotorTwo.getEncoder().getPosition();

  /** Creates a new Climber. */
  public Climber() {
    m_ClimberMotorOne.configure(m_ClimberConfigOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // telling ClimberMotor that it uses ClimberConfig
    m_ClimberMotorTwo.configure(m_ClimberConfigTwo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // telling ClimberMotor that it uses ClimberConfig

    m_ClimberConfigOne.inverted(true);
    m_ClimberConfigTwo.inverted(false);

    m_ClimberConfigOne.idleMode(IdleMode.kBrake);
    m_ClimberConfigTwo.idleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void out() {
    m_ClimberMotorOne.set(0.3);
    m_ClimberMotorTwo.set(0.3);
  }

  public void in() {
    m_ClimberMotorOne.set(-0.3);
    m_ClimberMotorTwo.set(-0.3);
  }

  public void stop() {
    m_ClimberMotorOne.set(0);
    m_ClimberMotorTwo.set(0);
  }
}
