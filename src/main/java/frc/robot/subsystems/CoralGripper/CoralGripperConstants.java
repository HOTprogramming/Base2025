package frc.robot.subsystems.CoralGripper;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
public final class CoralGripperConstants {
    // CAN identification
    public static final int kWristMotorID = 13;
    public static final int kWristEncoderID = 45;
    public static final int kGripperMotorID = 16;
    public static final int kCandiID = 53;

    public static final CANBus canbus = new CANBus(Constants.canName);

    // Setpoints 
    // Wrist (Degrees)
    public static final double kPackage = -1;
    public static final double kFlat = -91;
    // Gripper (Volts)
    public static final double kIntake = 8.0;
    public static final double kSlowSpit = -2.0;
    public static final double kSpit = -5.0;
    public static final double kShoot = -10.0;




    // Positional Limits (Degrees)
    public static final double kMinAngle = -110;
    public static final double kMaxAngle = 20;
    public static final double kPositionTolerance = 1.0;

    // Intake Limits (Volts)
    public static final double kMinVolts = -12;
    public static final double kMaxVolts = 12;

    // Sensor Zeroing
    public static final double kWristEncoderOffset = 0.07934570312;
    public static final double kSensorToMechanismRatio = 1.0 / 360.0;

    public static final boolean kPresetsCalibrated = true;

    // Motor data update rate (slow down if CAN util is > 70%)
    public static final double kStatusSignalUpdateRate = 50.0; // hz

    public static final CANcoderConfiguration WristEncoderConfigs = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withMagnetOffset(kWristEncoderOffset)
        );


    public static final TalonFXSConfiguration WristConfigs = new TalonFXSConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(90)
            .withSupplyCurrentLimitEnable(true)
        )
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withExternalFeedback(new ExternalFeedbackConfigs()
            .withFeedbackRemoteSensorID(kWristEncoderID)
            .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.RemoteCANcoder)
            .withSensorToMechanismRatio(kSensorToMechanismRatio)
            .withRotorToSensorRatio(1.0)
        )
        .withSlot0(new Slot0Configs() // PID tuning
            .withKP(0.725).withKI(0.0).withKD(0.01)
            .withKV(0.0).withKS(0.0).withKG(0.0)
        )
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(1.0)
            .withReverseSoftLimitThreshold(1.0)
        )
        .withCommutation(new CommutationConfigs()
            .withMotorArrangement(MotorArrangementValue.Brushed_DC)
        );

    public static final TalonFXConfiguration GripperConfigs = new TalonFXConfiguration()
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
        .withSlot0(new Slot0Configs() // PID tuning
            .withKP(0.0).withKI(0.0).withKD(0.0)
            .withKV(0.0).withKS(0.0).withKG(0.0)
        )
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false)
        );
}
