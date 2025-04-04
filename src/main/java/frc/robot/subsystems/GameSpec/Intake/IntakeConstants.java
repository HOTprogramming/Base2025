package frc.robot.subsystems.GameSpec.Intake;

import frc.robot.Constants;

public class IntakeConstants {

    public static int blackWheelsID;
    public static int orangeWheelsID;
    public static int intakeRotationID;
    public static int intakeEncoderID;
    //change to add Ks,Kv to Gains and allow 2 motors (KRM 3/29) 
    public static MMGains intakeGains;
    public static MMGains intakeOrangeGains;
    public static MMGains intakeBlackGains;

    public static double intakeHandoff;
    public static double intakeGround;
    public static double intakeClearance;
    public static double intakeBump;
    public static double rollerIntakeVoltage;
    public static double rollerExpelVoltage;
    public static double climberOut;
    public static double vertical;
    public static double intakeClimb;

    public static double intakeEncoderOffset;

    //Add Ks, Kv to Gains (KRM 03/29)
    public record MMGains(double kP, double kI, double kD, double kS, double kV) {} 

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
        //change to allow seperate gains for 2 motors (KRM 03/29)
        intakeGains = new MMGains(0.19,0.0,0.0,0.0,0.0);
        intakeOrangeGains = new MMGains(0.55,0.2,0.0,0.4,0.07);
        intakeBlackGains = new MMGains(0.1,0.1,0.0,0.4,0.95);

        intakeHandoff = 145.0;//147
        intakeClearance = 120;
        intakeBump = 75;
        intakeGround = 5;
        rollerIntakeVoltage = 10;
        rollerExpelVoltage = 1.5;
        climberOut = -120;
        vertical = -90;
        intakeClimb = 6.5;
        
        intakeEncoderOffset = 0.15283203125;

    }

    private static void practiceBotConstants(){
      blackWheelsID = 17;
      orangeWheelsID = 18;
      intakeRotationID = 19;
      intakeEncoderID = 46;
      // kReduction = (1.0 / 2.0);
      // kMaxAccelerationRpmPerSec = 9000.0; 
      
      //change to allow seperate gains for 2 motors (KRM 03/29)
      intakeGains = new MMGains(0.19,0.0,0.0,0.0,0.0);
      intakeOrangeGains = new MMGains(0.55,0.2,0.0,0.4,0.07);
      intakeBlackGains = new MMGains(0.1,0.1,0.0,0.4,0.95);

      
      intakeHandoff = 145.0;
      intakeClearance = 120;
      intakeBump = 75;
      intakeGround = 4;
      rollerIntakeVoltage = 10;
      rollerExpelVoltage = 1.5;
      climberOut = -120;
      vertical = -90;

      intakeEncoderOffset = -0.104;

    }

    private static void simBotConstants(){

    }
}