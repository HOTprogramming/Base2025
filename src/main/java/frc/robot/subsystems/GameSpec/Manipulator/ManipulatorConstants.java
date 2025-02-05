package frc.robot.subsystems.GameSpec.Manipulator;

public class ManipulatorConstants {

    // CORAL AND ALGEA IDS ARE WRONG
    public static final int coralMotorID = 15;//flipped
    public static final int coralEncoderID = 45;
    public static final int coralCandiID = 53;//sweetbox 6
    public static final int coralWristID = 13;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains coralWristGains = new MMGains(200, 100, 200, 1, 0.0, 0.0, 0, 0);
    public static final VVGains coralSpinGains = new VVGains(10.0, 0.0, 0.0, 0, 1);

    public static final int algaeArmID = 16;
    public static final int algaeRollerID = 25;
    public static final int algaeEncoderID = 52;
    public static final MMGains algaeGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
    
    public static final double l4Height = 2;
    public static final double l3Height = 1.5;
    public static final double l2Height = 1;
    public static final double l1Height = .5;
    public static final double netHeight = 3;
    public static final double intakeCoralHeight = 1.25;
    public static final double intakeAlgaeHeight = .75;

    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
    public record VVGains(double kP, double kI, double kD, double kV, double kS) {} 

}