package frc.examples.templates.manipulators.arm;

public class Constants {
    
    public static class ArmConstants {
    public static final int kLeftID = 1;
    public static final int kRightID = 2;

    public static final double kStartPos = 0;
    public static final double kIntakePos = 0; 
    public static final double kShootSpePos = 0;
    public static final double kDefaultPos = 0; 
    public static final double kFrontAmpPos = 0;
    
    public static final double kOverrunLimit = 0;

    //PID values must be tuned, FeedForward values can be calculated to be an estimate in reCalc.
    public static final double kP = 6; 
    public static final double kI = 0;
    public static final double kD = 0.7; 
    public static final double kIz = 0; 
    public static final double kFF = 0;
    
    public static final double kMaxOutput = 0.7; 
    public static final double kMinOutput = -0.7;
    public static final double kMaxAccel = 0.18;
    public static final double kMaxVel = 0.85;

    public static final double kTolerance = 0.002;
    
    public static final double kManualSpeed = 0.3;

    public static final double kSVolts = 0; // kS
    public static final double kGVolts = 0.87; // kG
    public static final double kVVoltSecondPerRad = 3.90; // kV
    public static final double kAVoltSecondSquaredPerRad = 0.07; // kA
    }
}
