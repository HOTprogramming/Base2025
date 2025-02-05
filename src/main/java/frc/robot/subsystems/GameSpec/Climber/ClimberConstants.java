package frc.robot.subsystems.GameSpec.Climber;

public class ClimberConstants {
    
    public static final int climberMotorID = 11;
    public static final int climberMotor2ID = 12;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains climberGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
    
    public static final double l4Height = 2;
    public static final double l3Height = 1.5;
    public static final double l2Height = 1;
    public static final double l1Height = .5;
    public static final double netHeight = 3;
    public static final double intakeCoralHeight = 1.25;
    public static final double intakeAlgaeHeight = .75;

    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
}