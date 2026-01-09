package frc.robot.subsystems.Turret;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;


public class TurretConstants {
    
    public static int kTurretMotorID = 32;
    public static int kTurretEncoderID = 33;
    public static String kBusName = "robot";
    public static double kEncoderOffset = -0.072266;     //Must be tuned -- check sensor direction 
    public static double kRotationPerDegree = (1.0/540.0);   //was 2.0 
    public static double kDegreePerRotation = 540.0; 
    public static double kLeftSoftLimit = -1.0;
    public static double kRightSoftLimit = 1.0;   
    public static double kFullRotation = kRotationPerDegree*360.0;

    // Define configuration for Turret Motor 
    public static final TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(30)
                .withSupplyCurrentLowerTime(1)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(120)
                .withPeakReverseTorqueCurrent(-120)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)    
                .withNeutralMode(NeutralModeValue.Coast)   
        ).withSlot0(
            new Slot0Configs()            /* Voltage-based position */
                .withKP(2.4)        // An error of 1 rotation results in 2.4v output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.1)        // A velocity of 1 rps results in 0.1v output 
        ).withSlot1(
            new Slot1Configs()            /* Torque-based position  */
                .withKP(60.0)       // An error of 1 rotation results in 60 A output
                .withKI(0.0)        // No output for integrated error     
                .withKD(6.0)        // A velocity of 1 rps results in 6 A output 
        ).withSlot2(                  
            new Slot2Configs()            /* Motion Magic Expo  */  
                .withKS(0.25)       // Add 0.25 V output to overcome static friction
                .withKV(.12)        // A velocity target of 1 rps results in 0.12 V output
                .withKA(0.01)       // An acceleration of 1 rps/s requires 0.01 V output
                .withKP(4.8)        // A position error of 2.5 rotations results in 12 V output
                .withKI(0)          // no output for integrated error
                .withKD(0.1)        // A velocity error of 1 rps results in 0.1 V output
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0.0)   // Unlimited cruise velocity
                .withMotionMagicExpo_kV(0.12)          // kV is around 0.12 V/rps
                .withMotionMagicExpo_kA(0.1)          // Use a slower kA of 0.1 V/(rps/s)
        ).withFeedback(
            new FeedbackConfigs() 
                .withFeedbackRemoteSensorID(kTurretEncoderID)        
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder) 
                .withSensorToMechanismRatio(1.0)  //The ratio of sensor rotations to the mechanism's output, where a ratio greater than 1 is a reduction.
                .withRotorToSensorRatio(1.0)   //The ratio of motor rotor rotations to remote sensor rotations, where a ratio greater than 1 is a reduction.
        );

    // Define configuration for Turret Motor Can Coder Sensor  
    public static final CANcoderConfiguration encoderConfigs = new CANcoderConfiguration() 
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withMagnetOffset(kEncoderOffset)    //The offset applied to the absolute integrated rotor sensor.
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        );


    /********************************************************
    * Based on the robot selected, load Intake configs with
    *   the selected robot configuration  
    ********************************************************/
    public static final TurretConfigs TurretConfigs =
        switch (Constants.getRobot()) {
            case COMPBOT -> new TurretConfigs(
                motorConfigs,
                encoderConfigs 
            );
            case SIMBOT -> new TurretConfigs(
                motorConfigs,
                encoderConfigs 
            );
            case DEVBOT -> new TurretConfigs(
                motorConfigs,
                encoderConfigs  
            );
            case CAMERABOT -> new TurretConfigs( 
                motorConfigs,
                encoderConfigs 
            );
            case COMP24 -> new TurretConfigs(
                motorConfigs,
                encoderConfigs 
            );
          
            case COMP25 -> new TurretConfigs(
                motorConfigs,
                encoderConfigs 
            );
      
            case MINIBOT -> new TurretConfigs(
                motorConfigs,
                encoderConfigs 
            );
          
        };


    /*****************
     * Define the Turret Configuration Class to be passed to TurretKraken  
     *****************/
    public record TurretConfigs(
        TalonFXConfiguration turretMotorConfigs,
        CANcoderConfiguration turretEncoderConfigs 
    ) {}

}
