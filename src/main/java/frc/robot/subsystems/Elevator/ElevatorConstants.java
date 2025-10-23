package frc.robot.subsystems.Elevator;

public class ElevatorConstants {
    public static int kElevatorID = 15;

    public static final double kP = 0.02;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kMaxVelocity = 100;
    public static final double kMaxAcceleration = 100;
    public static final double kMaxError = 0.5;
    public static final double kConversionFactor = ((16 / 1) / 6 + (3 / 4));

    public static final double kMinHeight = 0;
    public static final double kStageOne = 5.71;
    public static final double kIntake = 20;
    public static final double kStageTwo = 36;
    public static final double kStageThree = 86;
    public static final double kStageFour = 170;
    public static final double kMaxHeight = 171;
}
