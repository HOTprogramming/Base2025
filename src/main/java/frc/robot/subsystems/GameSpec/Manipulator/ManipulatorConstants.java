package frc.robot.subsystems.GameSpec.Manipulator;
import frc.robot.Constants;
import frc.robot.subsystems.GameSpec.Arm.Arm;
import frc.robot.subsystems.GameSpec.Arm.ArmIOReal;
import frc.robot.subsystems.GameSpec.Arm.ArmIOSim;
import frc.robot.subsystems.GameSpec.Elevator.Elevator;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOReal;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOSim;

public class ManipulatorConstants {

      public static int coralMotorID;
      public static int coralEncoderID;
      public static int coralCandiID;
      public static int coralWristID;
      public static int canRangeID;
      public static MMGains coralWristGains;
      public static VVGains coralSpinGains;
      public static double coralWristEncoderOffset;
      
      public static double coralWristHP;
      public static double coralWristScore;

      public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
      public record VVGains(double kP, double kI, double kD, double kV, double kS) {}
    

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
      
      coralMotorID = 16;
      coralEncoderID = 45;
      coralCandiID = 53;
      coralWristID = 13;
      canRangeID = 55;
      coralWristGains = new MMGains(0, 0, 0, 0.725, 0.0, 0.01, 0.0, 0.0);
      coralWristEncoderOffset = 0.429688;
  
      
      coralWristHP = -90;
      coralWristScore = 0;

    }

    private static void practiceBotConstants(){

      coralMotorID = 16;
      coralEncoderID = 45;
      coralCandiID = 53;
      coralWristID = 13;
      canRangeID = 55;
      //coralWristGains = new MMGains(3000, 4000, 14000, 0.2, 0.0, 0.002, 0.1, 0.0);
      coralWristGains = new MMGains(0, 0, 0, 0.725, 0.0, 0.01, 0.0, 0.0);
      coralWristEncoderOffset = -0.180176;
      
      coralWristHP = -99;
      coralWristScore = 0;

    }

    private static void simBotConstants(){

    }

}