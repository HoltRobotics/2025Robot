// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  Spark m_led = new Spark(LEDConstants.kBlinkInPort);
  
  /** Creates a new LED. */
  public LED() {
    setAlliance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLight(double setLED) {
    m_led.set(setLED);
  }

  public void setAlliance() {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    setLight(0.87);
  }else {
    setLight(0.61);
  }
  }
}
