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
        public static int kElevatorID = 15;

        public static final double kP = 0.0025;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kMaxVelocity = 0;
        public static final double kMaxAcceleration = 0;
        public static final double kMaxError = 0;

        public static final double kConvertionFactor = (( 16 / 1 ) / 6 + (3 / 4));
        public static final double kStageFour = 165.5;
        public static final double kStageThree = 90;
        public static final double kStageTwo = 32.7675;
        public static final double kStageOne = 5;
        public static final double kMaxHeight = 171;
        public static final double kMinHeight = 0;


    }

    public class ClimberConstants {
        public static final int kClimberMotorID = 0;
    }

    public class CoralManipulatorConstants {
        public static final int kWristMotorID = 16;
        public static final int kIntakeMotorID = 17;

        public static final double kStowAngle = 0; // TO-DO: Tune this
        public static final double kStage1Angle = -135; // TO-DO: Tune this
        public static final double kStage2Angle = -135;
        public static final double kStage3Angle = -135;
        public static final double kStage4Angle = -145;
        public static final double kMaxAngle = 0;
        public static final double kMinAngle = -90;
        public static final double kIntakeAngle = -45;
    }

    public class ArmConstants{
        public static final int armMotorId = 20;

        public static final double kStowAngle = 0;
        public static final double kStage1Angle = 170;
        public static final double kStage2Angle = 32.8;
        public static final double kStage3Angle = 80;
        public static final double kHumanStageAngle = 22;
        public static final double kTippedCoralAngle = 159.52;
        public static final double kMaxAngle = 180; // Change if we need a different max angle
        public static final double kClawHeightOffset = 0.3;
        public static final double kClawLengthOffset = 0.25;
        public static final double kArmLength = 0.5;

        public static final double kP = 0.15;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kMaxRadsPerSecond = 0;
        public static final double kMaxRadsPerSecondPerSecond = 0;
    }
}
