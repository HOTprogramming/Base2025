package frc.robot.subsystems.GameSpec.Elevator;

public class ElevatorConstants {
    
    public static final int elevatorMotorID = 9;
    public static final int elevatorMotor2ID = 10;
    public static final int elevatorEncoderID = 52;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains elevatorGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
    
    public static final double PackageHeight = 0;

    public static final double L4Height = 2;
    public static final double L3Height = 1.5;
    public static final double L2Height = 1;
    public static final double L1Height = .5;
    public static final double FeederHeight = 1.25;

    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
}