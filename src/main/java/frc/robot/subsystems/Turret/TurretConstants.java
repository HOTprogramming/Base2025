package frc.robot.subsystems.Turret;

import frc.robot.Constants;

public class TurretConstants {
    public static int turretMotorID;
    public static int turretCanCoderID;
    public static double kReduction;
    public static double kMaxAccelerationRpmPerSec;
    public static Gains turretGains;
    public static double elevatorEncoderOffset;
    public static double encoderClicksPerTurretRevolution;
    public static double encoderClicksPerDegree;
    public static double leftLimit; 
    public static double rightLimit;
    public static double tolerance; 

    public record Gains(double CruiseVelocity, double Acceleration, double ExpoKV, double ExpoKA, double kP, double kI, double kD, double kV, double kS, double kG, double kA) {} 


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
            case CAMERABOT -> {
                cameraBotConstants();
            }
        }
    }

    private static void compBotConstants(){
    }

    private static void practiceBotConstants(){
    }

    private static void simBotConstants(){
    }

    private static void cameraBotConstants(){
        turretMotorID = 32;
        turretCanCoderID = 33;
        kReduction = (1.0/2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        turretGains = new Gains(10, 10, 10.0, 0.001, 
                20.0 , 0, 1.3, 0.0, 0.0, 8.0,0.0);
        
        encoderClicksPerTurretRevolution = 1024.0; 
        encoderClicksPerDegree = encoderClicksPerTurretRevolution/360.0;
        //Allow 540 degrees of travel 
        leftLimit = 540 * encoderClicksPerDegree;  
        //Assume zero is 90 degrees right 
        rightLimit = 0.0;  
        //Set tolerance for 5 clicks 
        tolerance = 5.0;
    }

}