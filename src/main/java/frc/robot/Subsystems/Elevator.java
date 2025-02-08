// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final SparkMax m_elevator = new SparkMax(ElevatorConstants.kElevatorMotorID, MotorType.kBrushless);

  private final RelativeEncoder m_encoder;
  
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final SuppliedValueWidget<Double> m_heightDisplay;
  private final SimpleWidget m_moving;

  public Boolean m_inMotion;
  private double m_setPoint = 0;

  private final SparkMaxConfig m_elevatorConfig = new SparkMaxConfig();

  private final ElevatorFeedforward ff = 
    new ElevatorFeedforward(
      ElevatorConstants.kSVolts, 
      ElevatorConstants.kGVolts, 
      ElevatorConstants.kVVolts, 
      ElevatorConstants.kAVoltSecondSquared
    );

  /** Creates a new Elevator. */
  public Elevator() {
    m_encoder = m_elevator.getEncoder();
    m_encoder.setPosition(0);

    m_heightDisplay = m_tab.addDouble("Elevator Height", () -> getHeight());
    m_moving = m_tab.add("Elevator in motion?", m_inMotion);

    m_elevatorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    m_elevatorConfig.encoder
      .positionConversionFactor(Units.inchesToMeters(Math.PI * 1.44) / 16);

    m_elevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.001, 0, 0.00001);

    m_elevator.configure(m_elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setHeight(double height) {
    m_setPoint = height;
    m_elevator.getClosedLoopController().setReference(height, ControlType.kMAXMotionPositionControl);
    m_inMotion = true;
  }

  public void hold(TrapezoidProfile.State setpoint) {
    double feedforward = ff.calculate(setpoint.position * 2 * Math.PI, setpoint.velocity);
    m_elevator.getClosedLoopController().setReference(m_setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
  }

  public double getHeight() {
    return m_encoder.getPosition();
  }

  public void up() {
    m_elevator.set(0.6);
    m_inMotion = true;
  }

  public void down() {
    m_elevator.set(-0.6);
    m_inMotion = true;
  }

  public void stop() {
    m_elevator.stopMotor();
    m_inMotion = false;
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getHeight() > ElevatorConstants.kMaxHeight) {
      setHeight(ElevatorConstants.kMaxHeight);
    }

    if (getHeight() > m_setPoint - 0.005 && getHeight() < m_setPoint + 0.005) {
      m_inMotion = false;
    }
  }
}
