// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralManipulator extends SubsystemBase {


  /*leave comments to what these do */
  SparkMax m_shooter = new SparkMax(CoralManipulatorConstants.kIntakeMotorID, MotorType.kBrushless);
  SparkMax m_wrist = new SparkMax(CoralManipulatorConstants.kWristMotorID, MotorType.kBrushless);

  SparkMaxConfig m_shooterConfig = new SparkMaxConfig();
  SparkMaxConfig m_wristConfig = new SparkMaxConfig();

  double m_wristPosition = m_wrist.getEncoder().getPosition();
  double m_artSetPoint = 0;

  SparkClosedLoopController m_wristPid = m_wrist.getClosedLoopController();

  /** Creates a new shooter. */
  public CoralManipulator() {
    m_wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_wristConfig.closedLoop.pid(.001, 0, 0);

  m_shooterConfig.inverted(false);
  m_shooterConfig.idleMode(IdleMode.kBrake);

  m_wristConfig.inverted(false);
  m_wristConfig.idleMode(IdleMode.kBrake);

  m_shooter.configure(m_shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  m_wrist.configure(m_wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    //m_artPid.setReference(m_artSetPoint, ControlType.kPosition);
    // This method will be called once per scheduler run
  }


  public void shooterintake() {
    m_shooter.set(-.3);
    //set angle
  }

public void shootStop() {
  m_shooter.stopMotor();

}

  public void shootL1() {
    m_shooter.set(1);
    //set angle
  }

  public void shootL2() {
    m_shooter.set(1);
    //set angle
  }

  public void shootL3 () {
    m_shooter.set(1);
    //set angle
  }

  public void shootL4 () {
    m_shooter.set(1);
  
  }
 public void setsetpoint (double newsetpoint) {
m_artSetPoint = newsetpoint;
 }

public double getPosition () {
  return m_wristPosition;
}

public double getsetpoint () {
  return m_artSetPoint;
}

public void wristUp () {
  m_wrist.set(.1);
}

public void wristDown () {
  m_wrist.set(-.1);
}

public void wristStop () {
  m_wrist.stopMotor();
}
public void setAngle (double angle) {
  m_wristPid.setReference(angle, ControlType.kPosition);
}
}
