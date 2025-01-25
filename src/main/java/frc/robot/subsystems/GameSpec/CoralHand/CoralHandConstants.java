package frc.robot.subsystems.GameSpec.CoralHand;

import frc.robot.subsystems.GameSpec.Arm.ArmConstants.MMGains;

public class CoralHandConstants {



    
    public static final int coralHandMotorID = 32;
    public static final int coralHandEncoderID = 51;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains coralHandGains = new MMGains(200, 100, 200, 15, 0.0, 0.0, 0, 0);
    
    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
}