package frc.robot.subsystems.GameSpec.Intake;

import frc.robot.Constants;

public class IntakeConstants {

    public static int blackWheelsID;
    public static int orangeWheelsID;
    public static int intakeRotationID;
    public static int intakeEncoderID;
    public static double kReduction;
    public static double kMaxAccelerationRpmPerSec;
    public static MMGains intakeGains;

    public static double intakePackage;
    public static double intakeGround;
    public static double rollerIntakeVoltage;
    public static double rollerExpelVoltage;
    public static double climberOut;
    public static double vertical;

    public static double intakeEncoderOffset;

    public record MMGains(double kP, double kI, double kD) {} 

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

        blackWheelsID = 17;
        orangeWheelsID = 18;
        intakeRotationID = 19;
        intakeEncoderID = 46;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        intakeGains = new MMGains(0,0,0);
        
        intakePackage = -5.0;
        intakeGround = -150.0;
        rollerIntakeVoltage = 10;
        rollerExpelVoltage = 1.5;
        climberOut = -120;
        vertical = -90;

        intakeEncoderOffset = 0.313232421875;

    }

    private static void practiceBotConstants(){
      
    }

    private static void simBotConstants(){

    }
}