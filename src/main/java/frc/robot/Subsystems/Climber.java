// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  SparkMax m_ClimberMotorOne = new SparkMax(ClimberConstants.kClimberMotorOneID, MotorType.kBrushless);
  SparkMax m_ClimberMotorTwo = new SparkMax(ClimberConstants.kClimberMotorTwoID, MotorType.kBrushless);

  RelativeEncoder m_encoderOne = m_ClimberMotorOne.getEncoder();
  RelativeEncoder m_encoderTwo = m_ClimberMotorTwo.getEncoder();

  SparkClosedLoopController m_controllerOne = m_ClimberMotorOne.getClosedLoopController();
  SparkClosedLoopController m_controllerTwo = m_ClimberMotorTwo.getClosedLoopController();

  SparkMaxConfig m_ClimberConfigOne = new SparkMaxConfig();
  SparkMaxConfig m_ClimberConfigTwo = new SparkMaxConfig();

  /*climberposition variables are set to the respective motor's position */
  double m_ClimberOnePosition = m_ClimberMotorTwo.getEncoder().getPosition();
  double m_ClimberTwoPosition = m_ClimberMotorTwo.getEncoder().getPosition();
  double m_setpoint;

  boolean m_climbEnabled = false; // climber starts as disabled

  /** Creates a new Climber. */
  public Climber() {
    /*m_ClimberConfigOne is set to stop when not being moved and is also inverted */
    m_ClimberConfigOne
      .idleMode(IdleMode.kBrake)
      .inverted(true);

    /*m_ClimberConfigTwo is set to stop when not being moved and isn't inverted */
    m_ClimberConfigTwo
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    m_ClimberConfigOne.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.1, 0, 0);

    m_ClimberConfigTwo.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.1, 0, 0);

    m_ClimberMotorOne.configure(m_ClimberConfigOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // telling ClimberMotor that it uses ClimberConfig
    m_ClimberMotorTwo.configure(m_ClimberConfigTwo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // telling ClimberMotor that it uses ClimberConfig
 
    /*climber motors start at position 0 */
    m_ClimberMotorOne.getEncoder().setPosition(0);
    m_ClimberMotorTwo.getEncoder().setPosition(0);

    m_setpoint = 0; // climber setpoint starts at 0
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*position variables are constantly updated to the actual motor positions */
    m_ClimberOnePosition = m_ClimberMotorOne.getEncoder().getPosition();
    m_ClimberTwoPosition = m_ClimberMotorTwo.getEncoder().getPosition();

    /*code for the SmartDashboard statistics */
    SmartDashboard.putNumber("Climber Goal Angle", m_setpoint); // puts the climber setpoint as "Climber Goal Angle" number
    SmartDashboard.putNumber("Climber Actual Angle", ((m_ClimberOnePosition * m_ClimberTwoPosition) / 2)); // puts the climber's current angle as "Climber Actual Angle" number
    SmartDashboard.putBoolean("Climber Enabled", m_climbEnabled); // toggles true/false if the climber is enabled or not
  }

  public void in() {
    if (m_climbEnabled) {
      /*function in folds in the climber if the climber is enabled */
      m_ClimberMotorOne.set(-0.5);
      m_ClimberMotorTwo.set(-0.5);
    } else {
      /*if the climber is not enabled, the climber motors don't move */
      m_ClimberMotorOne.set(0);
      m_ClimberMotorTwo.set(0);
    }
  }

  public void out() {
    /*function out folds out the climber */
    m_ClimberMotorOne.set(0.5);
    m_ClimberMotorTwo.set(0.5);
  }

  public void stop() {
    /*function stop stops the climber motors */
    m_ClimberMotorOne.set(0);
    m_ClimberMotorTwo.set(0);
  }

  public void setClimberAngle(double angle) {
    m_setpoint = angle;
    m_ClimberMotorOne.getClosedLoopController().setReference(angle, ControlType.kPosition);
    m_ClimberMotorTwo.getClosedLoopController().setReference(angle, ControlType.kPosition);
  }

  public void resetClimber() {
    /*function resetClimber sets climber's position to 0*/
    m_ClimberMotorOne.getEncoder().setPosition(0);
    m_ClimberMotorTwo.getEncoder().setPosition(0);
  }

  public void climbEnable() {
    /*function climbEnable enables the climber */
    m_climbEnabled = true;
  }
}
