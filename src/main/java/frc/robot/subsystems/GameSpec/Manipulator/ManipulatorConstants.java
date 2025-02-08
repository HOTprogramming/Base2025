package frc.robot.subsystems.GameSpec.Manipulator;

public class ManipulatorConstants {

    // CORAL AND ALGEA IDS ARE WRONG
    public static final int coralMotorID = 16;//flipped
    public static final int coralEncoderID = 45;
    public static final int coralCandiID = 53;//sweetbox 6
    public static final int coralWristID = 13;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains coralWristGains = new MMGains(3000, 4000, 14000, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final VVGains coralSpinGains = new VVGains(10.0, 0.0, 0.0, 0, 1);

    public static final int algaeArmID = 17;
    public static final int algaeRollerID = 25;
    public static final int algaeEncoderID = 46;
    public static final MMGains algaeGains = new MMGains(100, 100, 200, 1.0 , 0.0, 0.0, 0, 0);
    
    public static final double coralWristHP = 0;
    public static final double coralWristScore = -90;

    public static final double algaeExtend = 0;
    public static final double algaePackage = 0;

    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
    public record VVGains(double kP, double kI, double kD, double kV, double kS) {} 

}