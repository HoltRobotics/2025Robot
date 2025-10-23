// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  SparkMax intake = new SparkMax(IntakeConstants.kIntakeID, MotorType.kBrushless);
  SparkMaxConfig intakeConfig = new SparkMaxConfig();

  boolean intakeRunning = false;
  /** Creates a new Intake. */
  public Intake() {
    intakeConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake() {
    intake.set(-0.5);
    intakeRunning = true;
  }

  public void outtake() {
    intake.set(0.5);
    intakeRunning = false;
  }

  public void spit() {
    intake.set(-0.5);
    intakeRunning = false;
  }

  public void stop() {
    intake.stopMotor();
    intakeRunning = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Running", intakeRunning);
  }
}
