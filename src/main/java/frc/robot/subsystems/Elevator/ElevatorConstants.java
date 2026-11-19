package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;

/**
 * This file should hold all of the constants for the entire subsystem.
 * If there is a value that doesnt change, it should be in here!
 * <p>
 * try to label units on values if possible with comments, 
 * and keep values with similar functions together in clearly marked code blocks
 * <p>
 * <b>anyone from the programming team should be able to easily make changes at a competition</b>
 * <p> 
 * do not forget to add current limits for both torqueFOC and voltage control modes,
 * regardless of the current intended control strat
 */
public final class ElevatorConstants {
    // CAN identification
    public static final int kelevatorMotorID = 9;
    public static final int kelevatorMotor2ID = 10;
    public static final int kelevatorEncoderID = 52;
    public static final CANBus canbus = new CANBus(Constants.canName);

    // Setpoints
    public static final double kSetpoint1 = 10.4;
    public static final double kSetpoint2 = 20.0;

    // Positional Limits
    public static final double kMinPosition = 10.0;
    public static final double kMaxPosition = 24.7;
    public static final double kPositionTolerance = 1.0;

    // Sensor Zeroing
    public static final double kelevatorEncoderOffset = 0.2546;
    public static final double kSensorToMechanismRatio = 0.208 / 24.75;

    // Motion Magic Tuning
    public static final double kExpoKV = 0.0;
    public static final double kExpoKA = 0.001;
    public static final double kCruiseVelocityRPS = 0.0; // 0.0 Is unlimited velocity (based on KV calculations)

    public static final boolean kPresetsCalibrated = true;

    // Motor data update rate (slow down if CAN util is > 70%)
    public static final double kStatusSignalUpdateRate = 50.0; // hz

    public static final CANcoderConfiguration EncoderConfigs = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(kelevatorEncoderOffset)
        );


    public static final TalonFXConfiguration ElevatorConfigs = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true)
        )
        .withTorqueCurrent(new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(60)
            .withPeakReverseTorqueCurrent(-60)
        )
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withFeedback(new FeedbackConfigs()
            .withFeedbackRemoteSensorID(kelevatorEncoderID)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withSensorToMechanismRatio(kSensorToMechanismRatio)
            .withRotorToSensorRatio(1.0)
        )
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kCruiseVelocityRPS)
            .withMotionMagicExpo_kV(kExpoKV)
            .withMotionMagicExpo_kA(kExpoKA)
        )
        .withSlot0(new Slot0Configs() // PID tuning
            .withKP(20.0).withKI(0.0).withKD(1.3)
            .withKV(0.0).withKS(0.0).withKG(8.0)
            .withGravityType(GravityTypeValue.Elevator_Static)
        )
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false)
        );
}
