// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

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

public class Wrist extends SubsystemBase {

  SparkMax wrist = new SparkMax(WristConstants.kWristID, MotorType.kBrushless);
  SparkMaxConfig wristConfig = new SparkMaxConfig();

  SparkClosedLoopController wristPID = wrist.getClosedLoopController();

  double position = wrist.getEncoder().getPosition();
  double setPoint = 0;
  boolean isEnabled = true;

  /** Creates a new Wrist. */
  public Wrist() {
    wristConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)

      .encoder
        .positionConversionFactor(WristConstants.kConversionFactor);

    wristConfig  
      .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0625, 0, 0.01)

      .maxMotion
        .maxAcceleration(WristConstants.kMaxAcceleration)
        .maxVelocity(WristConstants.kMaxVelocity)
        .allowedClosedLoopError(WristConstants.kMaxError)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wrist.getEncoder().setPosition(0);
  }

  public void setAngle(double angle) {
    setPoint = angle;
    wristPID.setReference(angle, ControlType.kMAXMotionPositionControl);
  }

  public double getPosition() {
    return position;
  }

  public void wristUp() {
    wrist.set(0.5);
  }

  public void wristDown() {
    wrist.set(-0.5);
  }

  public void stop() {
    wrist.stopMotor();
  }

  public void enablePID() {
    isEnabled = true;
  }

  public void disablePID() {
    isEnabled = false;
  }

  public void resetWrist() {
    wrist.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isEnabled) {
      wristPID.setReference(setPoint, ControlType.kMAXMotionPositionControl);
    }

    position = wrist.getEncoder().getPosition();

    SmartDashboard.putNumber("Wrist Angle", position);
     //System.out.println(position);
  }
}
