package frc.robot.subsystems.GameSpec.Elevator;

import frc.robot.Constants;

public class ElevatorConstants {

    public static int elevatorMotorID;
    public static int elevatorMotor2ID;
    public static int elevatorEncoderID;
    public static double kReduction;
    public static double kMaxAccelerationRpmPerSec;
    public static MMGains elevatorGains;
    public static double elevatorEncoderOffset;
    
    public static double PackageHeight;
    public static double AutoL4Height;
    public static double L4Height;
    public static double L4LongHeight;
    public static double L3Height;
    public static double L3LongHeight;
    public static double L2Height;
    public static double L2LongHeight;
    public static double L1Height;
    public static double FeederHeight;
    public static double L4ReturnScoreHeight;
    public static double L4MiniScoreHeight;
    public static double L3ScoreHeight;
    public static double L2ScoreHeight;
    public static double HPHeight;
    public static double climbHeight;
    public static double highAlgae;
    public static double lowAlgae;
    public static double BargeHeight;
    public static double ProcessorHeight;
    public static double initialClimbHeight;
    public static double intakeCoralHeight;
    public static double autonHalfHeight;


    public record MMGains(double CruiseVelocity, double Acceleration, double ExpoKV, double ExpoKA, double kP, double kI, double kD, double kV, double kS, double kG) {} 

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

        elevatorMotorID = 9;
        elevatorMotor2ID = 10;
        elevatorEncoderID = 52;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        elevatorGains = new MMGains(0, 0, 0.0, 0.001, 
        20.0 , 0, 1.3, 0.0, 0.0, 8.0);
        //elevatorGains = new MMGains(0, 0, 0, 45.0 , 0, 2.25, 30.0, 1.0, 0.0);//torque position pid

        elevatorEncoderOffset = 0.2546;
        PackageHeight = 15.0; 
        AutoL4Height = 58.0;
        L4Height = 53.0;
        L4LongHeight = 52.0;
        L3Height = 23.2;
        L3LongHeight = 20.8;
        L2Height = 10.4;
        L2LongHeight = 9.73;
        L1Height = 17.5;
        FeederHeight = 9.1;
        L4ReturnScoreHeight = 50.0;
        L3ScoreHeight = 18.0;
        L2ScoreHeight = 9.8; 
        HPHeight = 17.0;
        climbHeight = 0.25;
        highAlgae = 35;
        lowAlgae = 17;
        BargeHeight = 68.75;
        ProcessorHeight = 14.0;
        initialClimbHeight = 12.0;
        L4MiniScoreHeight = 50.0;
        intakeCoralHeight = 9.6;
        autonHalfHeight = 25.0;

        //elevator:15.7
        //arm: -45.7
        //elevator: 24.2 

    }

    private static void practiceBotConstants(){

        elevatorMotorID = 9;
        elevatorMotor2ID = 10;
        elevatorEncoderID = 52;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        //expo does not use the acceleration or jerk configs, cruise velocity is optional. 
        //The lower the KV and KA, the faster the mechanism moves.
        elevatorGains = new MMGains(0, 0, 0.0, 0.001, 
        20.0 , 0, 1.3, 0.0, 0.0, 8.0);
        elevatorEncoderOffset = 0.4953 - .0168;

        PackageHeight = 15.0; 
        AutoL4Height = 58.0;
        L4Height = 56.0;
        L4LongHeight = 52.0;
        L3Height = 25.55;
        L3LongHeight = 22.35;
        L2Height = 10.6;
        L2LongHeight = 9.73;
        L1Height = 17.0;
        FeederHeight = 12.0;
        L4ReturnScoreHeight = 50.0;
        L3ScoreHeight = 21.2;//was 19.8
        L2ScoreHeight = 11.0; //was 9.8
        HPHeight = 17.0;
        climbHeight = 0.25;
        highAlgae = 35;
        lowAlgae = 17;
        BargeHeight = 68.75;
        ProcessorHeight = 14.0;
        initialClimbHeight = 20.25;
        L4MiniScoreHeight = 50.0;
        autonHalfHeight = 25.0;
    }

    private static void simBotConstants(){

        elevatorMotorID = 9;
        elevatorMotor2ID = 10;
        elevatorEncoderID = 52;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        elevatorGains = new MMGains(1000, 1500, 4500, 2.5,0, 0, 0.0, 0.025, 0.3, 0.0);
        
        PackageHeight = 17.9;
    
        L4Height = 59.0;
        L4LongHeight = 55.0;
        L3Height = 28.35;
        L2Height = 13.2;
        L1Height = 18.79;
        FeederHeight = 1.25;
        //L4ScoreHeight = 40.0;
        L3ScoreHeight = 22.8;
        L2ScoreHeight = 9.2;
        HPHeight = 17.0;
        climbHeight = 1.4;

    }

}