package frc.robot.subsystems.GameSpec.Algae;
import frc.robot.Constants;

public class AlgaeConstants {

      public static int canRangeID;
      public static int algaeRollerID;

      public static double algaeIntakeVoltage;
      public static double algaeExpelVoltage;
      public static double algaeHoldVoltage;

      public static double algaeTriggerDistance;
    

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
      
      canRangeID = 55;
  
      algaeRollerID = 15;

      algaeIntakeVoltage = -8.0;
      algaeExpelVoltage = 16.0;
      algaeHoldVoltage = -0.8;

      algaeTriggerDistance = 0.07;
    }

    private static void practiceBotConstants(){

    }

    private static void simBotConstants(){

    }

}