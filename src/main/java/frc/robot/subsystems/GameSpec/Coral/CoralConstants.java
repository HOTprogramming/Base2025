package frc.robot.subsystems.GameSpec.Coral;

import frc.robot.subsystems.GameSpec.Arm.ArmConstants.MMGains;

public class CoralConstants {



    
    public static final int coralMotorID = 16;
    public static final int coralEncoderID = 45;
    public static final int coralCandiID = 53;//sweetbox 6
    public static final int coralWristID = 13;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains coralWristGains = new MMGains(200, 100, 200, 1, 0.0, 0.0, 0, 0);
    public static final VVGains coralSpinGains = new VVGains(10.0, 0.0, 0.0, 0, 0);

    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
    public record VVGains(double kP, double kI, double kD, double kV, double kS) {} 

}