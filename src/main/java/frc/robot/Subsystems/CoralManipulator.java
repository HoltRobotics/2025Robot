// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralManipulator extends SubsystemBase {


  /*leave comments to what these do */
  SparkMax m_shooter = new SparkMax(20, MotorType.kBrushless);
  SparkMax m_articulation = new SparkMax(21, MotorType.kBrushless);

  SparkMaxConfig m_shooterConfig = new SparkMaxConfig();
  SparkMaxConfig m_articulationConfig = new SparkMaxConfig();

  double m_artPosition = m_articulation.getEncoder().getPosition();
  double m_artSetPoint = 0;

  SparkClosedLoopController m_artPid = m_articulation.getClosedLoopController();

  /** Creates a new shooter. */
  public CoralManipulator() {
    m_articulationConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_articulationConfig.closedLoop.pid(.001, 0, 0);

  m_shooterConfig.inverted(false);
  m_shooterConfig.idleMode(IdleMode.kBrake);

  m_articulationConfig.inverted(false);
  m_articulationConfig.idleMode(IdleMode.kBrake);

  m_shooter.configure(m_shooterConfig, null, null);
  m_articulation.configure(m_articulationConfig, null, null);
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
  return m_artPosition;
}

public double getsetpoint () {
  return m_artSetPoint;
}

public void elbowUp () {
  m_articulation.set(.1);
}

public void elbowDown () {
  m_articulation.set(-.1);
}

public void elbowStop () {
  m_articulation.stopMotor();
}
public void setAngle (double angle) {
  m_artPid.setReference(angle, ControlType.kPosition);
}
}
