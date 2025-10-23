// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class JoystickUtils {

  public static double curveInput(double input, double deadband) {
    if (MathUtil.applyDeadband(input, deadband) == 0) return 0;

    double correctedValue = input;

    // Does math to force a linear output between deadband and 1;
    correctedValue = (correctedValue - (deadband * Math.signum(correctedValue))) / (1 - deadband);

    // Raises the input to a higher power (2 in this case) for a smoother feel
    correctedValue = Math.copySign(Math.pow(correctedValue, 2), input);

    return Math.min(correctedValue, 1.0);
  }
}
