package frc.robot.subsystems.Turret;

import frc.robot.Constants;

public class TurretConstants {
    public static int turretLeftMotorID;
    public static int turretLeftEncoderID;
    public static int turretRightMotorID;
    public static int turretRightEncoderID;

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
        turretLeftMotorID = 1;
        turretRightMotorID = 2;
    }

}