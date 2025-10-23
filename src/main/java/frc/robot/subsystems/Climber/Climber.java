// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  SparkMax climberOne = new SparkMax(ClimberConstants.kClimberOneID, MotorType.kBrushless);
  SparkMax climberTwo = new SparkMax(ClimberConstants.kClimberTwoID, MotorType.kBrushless);
  SparkMaxConfig climberOneConfig = new SparkMaxConfig();
  SparkMaxConfig climberTwoConfig = new SparkMaxConfig();

  double climberOnePosition = climberOne.getEncoder().getPosition();
  double climberTwoPosition = climberOne.getEncoder().getPosition();

  /** Creates a new Climber. */
  public Climber() {
    climberOneConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    climberTwoConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    climberOne.configure(climberOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberTwo.configure(climberTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void in() {
    climberOne.set(0.5);
    climberTwo.set(0.5);
  }

  public void out() {
    climberOne.set(-0.5);
    climberTwo.set(-0.5);
  }

  public void stop() {
    climberOne.stopMotor();
    climberTwo.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
