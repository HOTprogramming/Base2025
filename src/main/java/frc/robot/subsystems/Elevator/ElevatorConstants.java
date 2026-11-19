package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;

public final class ElevatorConstants {
    private ElevatorConstants() {}

    public static final int kelevatorMotorID = 9;
    public static final int kelevatorMotor2ID = 10;
    public static final int kelevatorEncoderID = 52;
    public static final CANBus canbus = new CANBus(Constants.canName);
    public static final double kSetpoint1 = 10.4;
    public static final double kSetpoint2 = 20.0;
    public static final boolean kPresetsCalibrated = true;

    public static final double kMinPosition = 10.0;
    public static final double kMaxPosition = 24.7;
    public static final double kPositionTolerance = 1.0;

    public static final double kelevatorEncoderOffset = 0.2546;
    public static final double kSensorToMechanismRatio = 0.208 / 24.75;
    public static final double kExpoKV = 0.0;
    public static final double kExpoKA = 0.001;
    public static final double kCruiseVelocityRPS = 0.0;

    public static final CANcoderConfiguration EncoderConfigs = new CANcoderConfiguration();
    static {
        EncoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        EncoderConfigs.MagnetSensor.MagnetOffset = kelevatorEncoderOffset;
    }

    public static final TalonFXConfiguration ElevatorConfigs = new TalonFXConfiguration();
    static {
        ElevatorConfigs.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true);
        ElevatorConfigs.TorqueCurrent = new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(60)
            .withPeakReverseTorqueCurrent(-60);
        ElevatorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ElevatorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        ElevatorConfigs.Feedback.FeedbackRemoteSensorID = kelevatorEncoderID;
        ElevatorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        ElevatorConfigs.Feedback.SensorToMechanismRatio = kSensorToMechanismRatio;
        ElevatorConfigs.Feedback.RotorToSensorRatio = 1.0;

        ElevatorConfigs.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kCruiseVelocityRPS)
            .withMotionMagicExpo_kV(kExpoKV)
            .withMotionMagicExpo_kA(kExpoKA);

        ElevatorConfigs.Slot0 = new Slot0Configs()
            .withKP(20.0).withKI(0.0).withKD(1.3)
            .withKV(0.0).withKS(0.0).withKG(8.0);
        ElevatorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        ElevatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        ElevatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    }
}
