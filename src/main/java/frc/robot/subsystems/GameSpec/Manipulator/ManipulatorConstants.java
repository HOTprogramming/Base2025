package frc.robot.subsystems.GameSpec.Manipulator;
import frc.robot.Constants;
import frc.robot.subsystems.GameSpec.Arm.Arm;
import frc.robot.subsystems.GameSpec.Arm.ArmIOReal;
import frc.robot.subsystems.GameSpec.Arm.ArmIOSim;
import frc.robot.subsystems.GameSpec.Elevator.Elevator;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOReal;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOSim;

public class ManipulatorConstants {

      // CORAL AND ALGEA IDS ARE WRONG
      public static int coralMotorID;
      public static int coralEncoderID;
      public static int coralCandiID;
      public static int coralWristID;
      public static double kReduction;
      public static double kMaxAccelerationRpmPerSec;
      public static MMGains coralWristGains;
      public static VVGains coralSpinGains;
      public static double coralWristEncoderOffset;
  
      public static int algaeArmID;
      public static int algaeRollerID;
      public static int algaeEncoderID;
      public static MMGains algaeGains;
      
      public static double coralWristHP;
      public static double coralWristScore;
  
      public static double algaeExtend;
      public static double algaePackage;

      public static double algaeIntakeVoltage;

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
      kReduction = (1.0 / 2.0);
      kMaxAccelerationRpmPerSec = 9000.0; 
      coralWristGains = new MMGains(0, 0, 0, 0.2, 0.0, 0.002, 0.1, 0.0);
      coralSpinGains = new VVGains(10.0, 0.0, 0.0, 0, 1);
      coralWristEncoderOffset = 0.163086;
  
      algaeArmID = 17;
      algaeRollerID = 25;
      algaeEncoderID = 46;
      algaeGains = new MMGains(100, 100, 200, 1.0 , 0.0, 0.0, 0, 0);
      
      coralWristHP = -90;
      coralWristScore = 0;
  
      algaeExtend = 0;
      algaePackage = 0;

      algaeIntakeVoltage = 1.0;
    }

    private static void practiceBotConstants(){

      // CORAL AND ALGEA IDS ARE WRONG
      coralMotorID = 16;//flipped
      coralEncoderID = 54;
      coralCandiID = 53;//sweetbox 6
      coralWristID = 13;
      kReduction = (1.0 / 2.0);
      kMaxAccelerationRpmPerSec = 9000.0; 
      coralWristGains = new MMGains(3000, 4000, 14000, 0.2, 0.0, 0.002, 0.1, 0.0);
      coralSpinGains = new VVGains(10.0, 0.0, 0.0, 0, 1);
      coralWristEncoderOffset = -0.472;
  
      algaeArmID = 17;
      algaeRollerID = 25;
      algaeEncoderID = 46;
      algaeGains = new MMGains(100, 100, 200, 1.0 , 0.0, 0.0, 0, 0);
      
      coralWristHP = -90;
      coralWristScore = 0;
  
      algaeExtend = 0;
      algaePackage = 0;

    }

    private static void simBotConstants(){

      coralMotorID = 16;
      coralEncoderID = 54;
      coralCandiID = 53;
      coralWristID = 13;
      kReduction = (1.0 / 2.0);
      kMaxAccelerationRpmPerSec = 9000.0; 
      coralWristGains = new MMGains(3000, 4000, 14000, 0.2, 0.0, 0.002, 0.1, 0.0);
      coralSpinGains = new VVGains(10.0, 0.0, 0.0, 0, 1);
  
      algaeArmID = 17;
      algaeRollerID = 25;
      algaeEncoderID = 46;
      algaeGains = new MMGains(100, 100, 200, 1.0 , 0.0, 0.0, 0, 0);
      
      coralWristHP = -90;
      coralWristScore = 0;
  
      algaeExtend = 0;
      algaePackage = 0;
    }

}