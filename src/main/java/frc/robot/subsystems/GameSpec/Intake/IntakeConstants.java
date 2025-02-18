package frc.robot.subsystems.GameSpec.Intake;

import frc.robot.Constants;

public class IntakeConstants {

    public static int intakeRollerID;
    public static int intakeRotationID;
    public static int intakeEncoderID;
    public static double kReduction;
    public static double kMaxAccelerationRpmPerSec;
    public static MMGains intakeGains;

    public static double intakePackage;
    public static double intakeGround;

    public static double intakeEncoderOffset;

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
        intakeGains = new MMGains(0, 0, 0, 0.1 , 0.0, 0.0, 0.05, 0);
        
        intakePackage = -5.0;
        intakeGround = -158.0;

        intakeEncoderOffset = 0.39404296875;

    }

    private static void practiceBotConstants(){

        intakeRollerID = 18;
        intakeRotationID = 19;
        intakeEncoderID = 46;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        intakeGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
    }

    private static void simBotConstants(){

        intakeRollerID = 18;
        intakeRotationID = 19;
        intakeEncoderID = 46;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        intakeGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);
        

    }
}