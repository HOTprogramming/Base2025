package frc.robot.subsystems.Arm;

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
public final class ArmConstants {
    // CAN identification
    public static final int kArmMotorID = 14;
    public static final int kArmEncoderID = 44;
    public static final CANBus canbus = new CANBus(Constants.canName);

    // Setpoints (Degrees)
    public static final double kPackage = -90;
    public static final double kHorizontal = 0;

    // Positional Limits (Degrees)
    public static final double kMinPosition = -360;
    public static final double kMaxPosition = 1;
    public static final double kPositionTolerance = 1.0;

    // Sensor Zeroing
    public static final double kArmEncoderOffset = 0.149658203125;
    public static final double kSensorToMechanismRatio = 1.0 / 360.0;

    // Motion Magic Tuning
    public static final double kExpoKV = 0.0;
    public static final double kExpoKA = 0.001;
    public static final double kCruiseVelocityRPS = 0.0; // 0.0 Is unlimited velocity (based on KV calculations)

    public static final boolean kPresetsCalibrated = true;

    // Motor data update rate (slow down if CAN util is > 70%)
    public static final double kStatusSignalUpdateRate = 100.0; // hz

    public static final CANcoderConfiguration EncoderConfigs = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(kArmEncoderOffset)
        );


    public static final TalonFXConfiguration ArmConfigs = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(90)
            .withSupplyCurrentLimitEnable(true)
        )
        .withTorqueCurrent(new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(65)
            .withPeakReverseTorqueCurrent(-65)
        )
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withFeedback(new FeedbackConfigs()
            .withFeedbackRemoteSensorID(kArmEncoderID)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withSensorToMechanismRatio(kSensorToMechanismRatio)
            .withRotorToSensorRatio(1.0)
        )
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicExpo_kA(kExpoKA)
            .withMotionMagicExpo_kV(kExpoKV)
        )
        .withSlot0(new Slot0Configs() // PID tuning
            .withKP(1.82).withKI(0.0).withKD(0.13)
            .withKV(0.0).withKS(0.0).withKG(-2.0)
        )
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(1.0)
            .withReverseSoftLimitThreshold(1.0)
        );
}
