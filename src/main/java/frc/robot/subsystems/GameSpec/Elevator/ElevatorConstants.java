package frc.robot.subsystems.GameSpec.Elevator;

public class ElevatorConstants {
    
    public static final int elevatorMotorID = 9;
    public static final int elevatorMotor2ID = 10;
    public static final int elevatorEncoderID = 52;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains elevatorGains = new MMGains(1000, 1500, 4500, 2.5 , 0, 0.0, 0.025, 0.3);
    
    public static final double PackageHeight = 17.9;

    public static final double L4Height = 57.0;
    public static final double L4LongHeight = 53.0;
    public static final double L3Height = 28.35;
    public static final double L2Height = 13.2;
    public static final double L1Height = 18.79;
    public static final double FeederHeight = 1.25;
    public static final double L4ScoreHeight = 40.0;
    public static final double L3ScoreHeight = 22.8;
    public static final double L2ScoreHeight = 9.2;
    public static final double HPHeight = 17.0;
    public static final double climbHeight = 5.0;


    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
}