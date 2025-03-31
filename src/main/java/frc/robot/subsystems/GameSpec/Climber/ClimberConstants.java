package frc.robot.subsystems.GameSpec.Climber;

import frc.robot.Constants;

public class ClimberConstants {
    public static int climberMotorID;
    public static int climberMotor2ID;
    public static int ServoPort;
    public static int ServoPort2;
    public static int ServoPort3;
    public static double climberServoLockPos;
    public static double climberServoOpenPos;
    public static double targetClicks;
    public static double packageClicks;
    public static double softStopClicks;


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

        climberMotorID = 11;
        climberMotor2ID = 12;
        ServoPort = 8;
        ServoPort2 = 9;
        ServoPort3 = 7;
        climberServoLockPos = 0.50;
        climberServoOpenPos = 0.16;
        targetClicks = 100.0;
        packageClicks = 5.0;
        softStopClicks = 30.0;

    }

    private static void practiceBotConstants(){

        climberMotorID = 11;
        climberMotor2ID = 12;
        ServoPort = 8;
        ServoPort2 = 9;
        climberServoLockPos = 0.50;
        climberServoOpenPos = 0.16;
        targetClicks = 80.0;
        softStopClicks = 40.0;

    }

    private static void simBotConstants(){

        climberMotorID = 11;
        climberMotor2ID = 12;
        ServoPort = 8;
        ServoPort2 = 9;
        climberServoLockPos = 0.50;
        climberServoOpenPos = 0.16;
        targetClicks = 80.0;

    }

}