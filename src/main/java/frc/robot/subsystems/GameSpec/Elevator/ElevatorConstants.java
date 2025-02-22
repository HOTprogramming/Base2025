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
    public static double L4Height;
    public static double L4LongHeight;
    public static double L3Height;
    public static double L2Height;
    public static double L1Height;
    public static double FeederHeight;
    public static double L4ScoreHeight;
    public static double L4MiniScoreHeight;
    public static double L3ScoreHeight;
    public static double L2ScoreHeight;
    public static double HPHeight;
    public static double climbHeight;
    public static double highAlgae;
    public static double lowAlgae;
    public static double BargeHeight;
    public static double ProcessorHeight;
    public static double FloorIntakeHeight;
    public static double FloorIntakeGrabHeight;


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

        elevatorMotorID = 9;
        elevatorMotor2ID = 10;
        elevatorEncoderID = 52;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        elevatorGains = new MMGains(1000, 1500, 4500, 2.5 , 0, 0.0, 0.025, 0.3);
        elevatorEncoderOffset = 0.2546;

        PackageHeight = 15.0; 
        L4Height = 57.0;
        L4LongHeight = 53.0;
        L3Height = 25.35;
        L2Height = 13.9;
        L1Height = 14.79;
        FeederHeight = 13.2;
        L4ScoreHeight = 40.0;
        L3ScoreHeight = 19.8;
        L2ScoreHeight = 9.8;
        HPHeight = 17.0;
        climbHeight = 1.4;
        highAlgae = 35;
        lowAlgae = 17;
        BargeHeight = 68.5;
        ProcessorHeight = 14.0;
        FloorIntakeHeight = 20.0;
        FloorIntakeGrabHeight = 15.0;
        L4MiniScoreHeight = 50.0;

    }

    private static void practiceBotConstants(){

        elevatorMotorID = 9;
        elevatorMotor2ID = 10;
        elevatorEncoderID = 52;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        elevatorGains = new MMGains(1000, 1500, 4500, 2.5 , 0, 0.0, 0.025, 0.3);
        elevatorEncoderOffset = 0.4953;

        PackageHeight = 17.9; 
        L4Height = 57.0;
        L4LongHeight = 53.0;
        L3Height = 28.35;
        L2Height = 13.2;
        L1Height = 18.79;
        FeederHeight = 17.9;
        L4ScoreHeight = 40.0;
        L3ScoreHeight = 22.8;
        L2ScoreHeight = 9.2;
        HPHeight = 17.0;
        climbHeight = 1.4;
        L4MiniScoreHeight = 40.0;
    }

    private static void simBotConstants(){

        elevatorMotorID = 9;
        elevatorMotor2ID = 10;
        elevatorEncoderID = 52;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        elevatorGains = new MMGains(1000, 1500, 4500, 2.5 , 0, 0.0, 0.025, 0.3);
        
        PackageHeight = 17.9;
    
        L4Height = 57.0;
        L4LongHeight = 53.0;
        L3Height = 28.35;
        L2Height = 13.2;
        L1Height = 18.79;
        FeederHeight = 1.25;
        L4ScoreHeight = 40.0;
        L3ScoreHeight = 22.8;
        L2ScoreHeight = 9.2;
        HPHeight = 17.0;
        climbHeight = 1.4;

    }

}