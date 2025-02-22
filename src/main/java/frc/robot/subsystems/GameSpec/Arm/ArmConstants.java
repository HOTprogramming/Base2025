package frc.robot.subsystems.GameSpec.Arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmConstants {

    public static int armMotorID;
    public static int armEncoderID;
    public static double kReduction;
    public static double kMaxAccelerationRpmPerSec;
    public static double simGearing;
    public static double simInertia;
    public static double simArmLength;
    public static double simMinAngle;
    public static double simMaxAngle;
    public static double startingAngle;
    public static double measurementSTDDEVS;
    public static MMGains armGains;
    public static DCMotor simGearBox;
    public static boolean gravity;
    public static double armEncoderOffset;

    public static double PackageAngle;
    public static double FeederAngle;
    public static double L1Angle;
    public static double L2Angle;
    public static double L3Angle;
    public static double L4Angle;
    public static double L4Score;
    public static double L3Score;
    public static double L2Score;
    public static double Horizontal;
    public static double Intermediate;
    public static double CurrentFail;
    public static double L3short;
    public static double L2Short;
    public static double L4Short;
    public static double IntakeAlgae;
    public static double Processor;
    public static double GetAlgaeFromReef;
    public static double Barge;
    
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

        armMotorID = 14;
        armEncoderID = 44;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        simGearing = 200;
        simInertia = SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 8.0);
        simArmLength = Units.inchesToMeters(60);
        simMinAngle = Units.degreesToRadians(-90);
        simMaxAngle = Units.degreesToRadians(90);
        startingAngle = Units.degreesToRadians(0);
        measurementSTDDEVS = 2.0 * Math.PI / 4096.0;
        armGains = new MMGains(0, 0, 0, 1.0, 0.0, 0.14, 0.0, 0.0);//Voltage
        simGearBox = DCMotor.getKrakenX60Foc(1);
        gravity = false;
        armEncoderOffset = 0.08618178125;
    
        PackageAngle = 0;
        FeederAngle = 50.8;
        L1Angle = -104.0;
        L2Angle = -52.2;
        L3Angle = -31.6;
        L4Angle = -36;
        L4Score = -65;
        L3Score = -81.5;
        L2Score = -87.8;
        Horizontal = 90;
        Intermediate = -49.0;
        CurrentFail = 60.0;
        L3short = -20.6;
        L2Short = -20.6;
        L4Short = -22.5;
        IntakeAlgae = -61.0;
        Processor = 14.3;
        GetAlgaeFromReef = 109;
        Barge = 128;

    }

    private static void practiceBotConstants(){

        armMotorID = 14;
        armEncoderID = 44;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        simGearing = 200;
        simInertia = SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 8.0);
        simArmLength = Units.inchesToMeters(60);
        simMinAngle = Units.degreesToRadians(-90);
        simMaxAngle = Units.degreesToRadians(90);
        startingAngle = Units.degreesToRadians(0);
        measurementSTDDEVS = 2.0 * Math.PI / 4096.0;
        armGains = new MMGains(3000, 3000, 6500, 0.2, 0.0, 0.01, 0.025, 0.3);//Voltage
        simGearBox = DCMotor.getKrakenX60Foc(1);
        gravity = false;
        armEncoderOffset = -0.1977;
    
        PackageAngle = 0;
        FeederAngle = 52.4;
        L1Angle = -104.0;
        L2Angle = -31.6;
        L3Angle = -31.6;
        L4Angle = -36;
        L4Score = -65;
        L3Score = -81.5;
        L2Score = -71.5;
        Horizontal = 90;
        Intermediate = -20.0;
        CurrentFail = 17.0;
        L3short = -20.6;
        L2Short = -20.6;
        L4Short = -22.5;

    }

    private static void simBotConstants(){

        armMotorID = 14;
        armEncoderID = 44;
        kReduction = (1.0 / 2.0);
        kMaxAccelerationRpmPerSec = 9000.0; 
        simGearing = 200;
        simInertia = SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 8.0);
        simArmLength = Units.inchesToMeters(60);
        simMinAngle = Units.degreesToRadians(-90);
        simMaxAngle = Units.degreesToRadians(90);
        startingAngle = Units.degreesToRadians(0);
        measurementSTDDEVS = 2.0 * Math.PI / 4096.0;
        armGains = new MMGains(3000, 3000, 6500, 0.2, 0.0, 0.01, 0.025, 0.3);//Voltage
        simGearBox = DCMotor.getKrakenX60Foc(1);
        gravity = false;
    
        PackageAngle = 0;
        FeederAngle = 52.4;
        L1Angle = -104.0;
        L2Angle = -31.6;
        L3Angle = -31.6;
        L4Angle = -36;
        L4Score = -65;
        L3Score = -81.5;
        L2Score = -71.5;
        Horizontal = 90;
        Intermediate = -20.0;
        CurrentFail = 17.0;
        L3short = -20.6;
        L2Short = -20.6;
        L4Short = -22.5;

    }
    
}