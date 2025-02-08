// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public class OIConstants {
        public static int kDriverPort = 0;
        public static int kOperatorPort = 1;
    }

    public class ElevatorConstants {
        // Motor ID's
        public static int kElevatorMotorID = 20;

        // Manual Speed
        public static double kManualSpeed = 0.6;
        
        // Height Limits
        public static double kMaxHeight = 100;
        public static double kMinHeight = 0.05;

        // Heights
        public static double kLoadHeight = 20;
        public static double kL2Height = 10;
        public static double kL3Height = 40;
        public static double kL4Height = 90;

        // Feed Forward Controls
        public static double kSVolts = 0;
        public static double kGVolts = 0.1;
        public static double kVVolts = 6;
        public static double kAVoltSecondSquared = 0.05;

    }

    public class FunnelConstants {

    }

    public class ClimberConstants {

    }

    public class CoralManipulatorConstants {

    }
}
