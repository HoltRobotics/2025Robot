// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgeaManipulatorConstants;

public class AlgeaManipulator extends SubsystemBase {
  SparkMax m_shooterOne = new SparkMax(AlgeaManipulatorConstants.kShooterOne, MotorType.kBrushless);
  SparkMax m_shooterTwo = new SparkMax(AlgeaManipulatorConstants.kShooterTwo, MotorType.kBrushless);
  SparkMax m_articulator = new SparkMax(AlgeaManipulatorConstants.kArticulator, MotorType.kBrushless);

  SparkMaxConfig m_shooterOneConfig = new SparkMaxConfig();
  SparkMaxConfig m_shooterTwoConfig = new SparkMaxConfig();
  SparkMaxConfig m_articConfig = new SparkMaxConfig();

  double m_articPosition = m_articulator.getEncoder().getPosition();
  double m_setPoint;

  SparkClosedLoopController m_articPID = m_articulator.getClosedLoopController();



  /** Creates a new Funnel. */
  public AlgeaManipulator() {

    m_shooterOneConfig.inverted(false);
    m_shooterOneConfig.idleMode(IdleMode.kBrake);

    m_shooterTwoConfig.inverted(false);
    m_articConfig.inverted(false);

  }

  @Override
  public void periodic() {
    m_articPosition = m_articulator.getEncoder().getPosition();
    // This method will be called once per scheduler run
  }
}
