// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.LimelightConstants;


public class Limelight extends SubsystemBase {
  public GenericEntry m_distance;
  private ShuffleboardTab m_tab;

  public double m_goodDistance = getDistance();
  /** Creates a new Limelight. */
  public Limelight() {
    setCorrectTarget();
    m_tab = Shuffleboard.getTab("Main");
    m_distance = m_tab.add("Distance", getDistance()).withWidget(BuiltInWidgets.kTextView).getEntry();
  }



  public double getDistance() {
    return (LimelightConstants.kGoalHeightMeters - LimelightConstants.kLimelightLensHeightMeters) / Math.tan(LimelightConstants.kMountAngleRadians + Units.degreesToRadians(LimelightHelpers.getTY("getName()")));
  }

  public double getTargetArmAngle(){
    return (15.2 + (16.7 * getDistance()) + (-1.68 * Math.pow(getDistance(), 2)));
  }

  public double getTargetRPM(){
    return (65 + (5 * getDistance()));
  }

  public double getTX(String Limelight){
    return LimelightHelpers.getTX(Limelight);
  }

  public double getTY(String limelight){
    return LimelightHelpers.getTY(limelight);
  }

  private void setCorrectTarget(){
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      LimelightHelpers.setPriorityTagID("getName()", 21);
    } else if (DriverStation.getAlliance().get() == Alliance.Red){
      LimelightHelpers.setPriorityTagID("getName()", 10);
    } else {
      System.out.println("Did not get alliance to set up the limelight");
    }
  }



  @Override
  public void periodic() {
    m_distance.setDouble(getDistance());
  }

}
