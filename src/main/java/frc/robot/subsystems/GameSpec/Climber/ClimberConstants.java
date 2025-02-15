package frc.robot.subsystems.GameSpec.Climber;

public class ClimberConstants {
    
    public static final int climberMotorID = 11;
    public static final int climberMotor2ID = 12;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains climberGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
    public static final int ServoPort = 3;
    public static final int ServoPort2 = 11;
    public static final double UnspoolDistance = -2;
    public static final double SpoolDistance = 3;
    public static final double ServoClampDistance = 0.5;
    public static final double climberServoLockPos = 1.0;
    public static final double climberServoOpenPos = 0.0;

    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
}