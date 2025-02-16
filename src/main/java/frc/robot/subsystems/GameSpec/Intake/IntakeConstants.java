package frc.robot.subsystems.GameSpec.Intake;

import frc.robot.Constants;

public class IntakeConstants {

    public static int intakeRollerID;
    public static int intakeRotationID;
    public static int intakeEncoderID;
    public static double kReduction;
    public static double kMaxAccelerationRpmPerSec;
    public static MMGains intakeGains;
    
    public static double l4Height;
    public static double l3Height;
    public static double l2Height;
    public static double l1Height;
    public static double netHeight;
    public static double intakeCoralHeight;
    public static double intakeAlgaeHeight;


    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 

    static {
        switch (Constants.getRobot()) {
        case COMPBOT -> {
          compBotConstants();
        }
        case DEVBOT -> {
          practiceBotConstants();
        }
        case SIMBOT -> {    
          simBotConstants();
        }
      }
    }

    private static void compBotConstants(){

        intakeRollerID = 18;
        intakeRotationID = 19;
        intakeEncoderID = 46;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        intakeGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
        
        l4Height = 2;
        l3Height = 1.5;
        l2Height = 1;
        l1Height = .5;
        netHeight = 3;
        intakeCoralHeight = 1.25;
        intakeAlgaeHeight = .75;

    }

    private static void practiceBotConstants(){

        intakeRollerID = 18;
        intakeRotationID = 19;
        intakeEncoderID = 46;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        intakeGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
        
        l4Height = 2;
        l3Height = 1.5;
        l2Height = 1;
        l1Height = .5;
        netHeight = 3;
        intakeCoralHeight = 1.25;
        intakeAlgaeHeight = .75;

    }

    private static void simBotConstants(){

        intakeRollerID = 18;
        intakeRotationID = 19;
        intakeEncoderID = 46;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        intakeGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
        
        l4Height = 2;
        l3Height = 1.5;
        l2Height = 1;
        l1Height = .5;
        netHeight = 3;
        intakeCoralHeight = 1.25;
        intakeAlgaeHeight = .75;

    }
}