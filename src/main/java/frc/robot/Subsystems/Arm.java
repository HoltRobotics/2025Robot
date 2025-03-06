// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final SparkMax m_armMotor = new SparkMax(ArmConstants.armMotorId, MotorType.kBrushless);
  private final SparkMax m_followingMotor = new SparkMax(ArmConstants.followingMotorId, MotorType.kBrushless);

  private final RelativeEncoder m_encoder;
  private final RelativeEncoder m_followingEncoder;
  
  SparkMaxConfig m_armConfig = new SparkMaxConfig();
  SparkMaxConfig m_followerConfig = new SparkMaxConfig();


  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final GenericEntry m_angleDisplay;

  private final SparkClosedLoopController m_climberPid = m_armMotor.getClosedLoopController();
  private final SparkClosedLoopController m_followingClimberPID = m_followingMotor.getClosedLoopController();

  public boolean m_inMotion = false;
  private double m_setPoint = 0;

  /** Creates a new Arm. */
  public Arm() {
    m_encoder = m_armMotor.getEncoder();
    m_followingEncoder = m_followingMotor.getEncoder();

    m_armConfig.encoder
    .positionConversionFactor(1/125);

    m_armConfig.closedLoop
    .pid(0.100, 0, 0)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    
    m_followerConfig
    .follow(m_armMotor, true);

    m_followerConfig.encoder
    .positionConversionFactor(1/125);

    m_followerConfig.closedLoop
    .pid(0.1, 0, 0)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    m_followingMotor.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_armMotor.configure(m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_angleDisplay = m_tab.add("Arm Angle", getAngle()).getEntry();

    m_encoder.setPosition(0);
    m_followingEncoder.setPosition(0);
  }

  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_armMotor.setVoltage(output);
  }

  /**
   * Gets the current angle of the arm.
   * @return Angle of the arm.
   */
  public double getAngle() {
    return m_encoder.getPosition(); // Returns the angle of the arm.
  }

  /**
   * Sets the PID setpoint.
   * @param angle The desired angle of the arm.
   */
  public void setAngle(double angle) {
    m_setPoint = angle;
  }

  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle();
  }

  /**
   * Method for forcing the arm to move up.
   */
  public void up() {
    m_armMotor.set(-0.4);
    m_followingMotor.set(-0.4); // Sets the speed of the motor to -1/4.
  }

  /**
   * Method for forcing the arm to move down.
   */
  public void down() {
    m_armMotor.set(0.4); // Sets the speed of the motor to 1/4.
    m_followingMotor.set(0.4);
  }

  public boolean getInMotion() {
    return m_inMotion;
  }

  public void setInMotion(boolean inMotion) {
    m_inMotion = inMotion;
  }


  public void setSpeed(double speed) {
    m_armMotor.set(speed);
    m_followingMotor.set(speed);
  }

  @Override
  public void periodic() {
    super.periodic();
    m_angleDisplay.setDouble(getAngle());
    if(getAngle() < -5) { // Checks to see if the arm is past the min limit.
      setAngle(0); // If it is set the PID to 0.
    }
    if(getAngle() > ArmConstants.kMaxAngle) { // Checks to see if the arm is past the max limit.
      setAngle(ArmConstants.kMaxAngle); // If is is set the PID to 180.
    }
    if(getAngle() > m_setPoint - 0.1 && getAngle() < m_setPoint + 0.1) {
      m_inMotion = false;
    }

    m_climberPid.setReference(m_setPoint, ControlType.kPosition);
    m_followingClimberPID.setReference(m_setPoint, ControlType.kPosition);
  }

  public void stopArm(){
    m_armMotor.set(0);
    m_followingMotor.set(0);
  }

  public void armAngle(double angle){
    m_climberPid.setReference(angle, ControlType.kPosition);
    m_followingClimberPID.setReference(angle, ControlType.kPosition);
  }
}