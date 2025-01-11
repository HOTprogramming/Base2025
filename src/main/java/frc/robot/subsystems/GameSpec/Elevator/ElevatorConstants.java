package frc.robot.subsystems.GameSpec.Elevator;

public class ElevatorConstants {
    
    public static final int elevatorMotorID = 32;
    public static final int elevatorEncoderID = 51;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains elevatorGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
    
    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
}