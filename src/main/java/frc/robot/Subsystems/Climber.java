// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
/**
 * @deprecated Use {@link frc.robot.subsystems.Arm} instead.
 * At least, I assume this file isn't being used
 */
public class Climber extends SubsystemBase {

  SparkMax m_ClimberMotor = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
  SparkMaxConfig m_ClimberConfig = new SparkMaxConfig();

  EncoderConfig m_ClimberEncoder = m_ClimberConfig.encoder.positionConversionFactor(0);

  double m_ClimberPosition = m_ClimberMotor.configAccessor.encoder.getPositionConversionFactor();

  /** Creates a new Climber. */
  public Climber() {
    m_ClimberMotor.configure(m_ClimberConfig, null, null); // telling ClimberMotor that it uses ClimberConfig
    m_ClimberConfig.inverted(true);
    m_ClimberConfig.idleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
