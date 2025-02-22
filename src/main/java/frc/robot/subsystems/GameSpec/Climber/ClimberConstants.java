package frc.robot.subsystems.GameSpec.Climber;

import frc.robot.Constants;

public class ClimberConstants {
    public static int climberMotorID;
    public static int climberMotor2ID;
    public static double kReduction;
    public static double kMaxAccelerationRpmPerSec;
    public static MMGains climberGains;
    public static int ServoPort;
    public static int ServoPort2;
    public static double UnspoolDistance;
    public static double SpoolDistance;
    public static double ServoClampDistance;
    public static double climberServoLockPos;
    public static double climberServoOpenPos;
    public static double targetClicks;
    public static double packageClicks;

    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 

    //public static final double climberServoLockPos2 = 0.33;
    //public static final double climberServoOpenPos2 = 0.34;

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
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0;
        UnspoolDistance = -2;
        SpoolDistance = 3;
        ServoClampDistance = 0.5;
        climberServoLockPos = 0.50;
        climberServoOpenPos = 0.16;
        targetClicks = 80.0;
        packageClicks = 5.0;
        climberGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);

    }

    private static void practiceBotConstants(){

        climberMotorID = 11;
        climberMotor2ID = 12;
        ServoPort = 8;
        ServoPort2 = 9;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0;
        UnspoolDistance = -2;
        SpoolDistance = 3;
        ServoClampDistance = 0.5;
        climberServoLockPos = 0.50;
        climberServoOpenPos = 0.16;
        targetClicks = 80.0;
        climberGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);

    }

    private static void simBotConstants(){

        climberMotorID = 11;
        climberMotor2ID = 12;
        ServoPort = 8;
        ServoPort2 = 9;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0;
        UnspoolDistance = -2;
        SpoolDistance = 3;
        ServoClampDistance = 0.5;
        climberServoLockPos = 0.50;
        climberServoOpenPos = 0.16;
        targetClicks = 80.0;
        climberGains = new MMGains(200, 100, 200, 20 , 2.5, 0.0, 0, 0);

    }

}