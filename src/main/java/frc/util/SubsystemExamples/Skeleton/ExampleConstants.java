package frc.util.SubsystemExamples.Skeleton;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public final class ExampleConstants {
    // CAN identification
    public static final int kExampleMotorID = 0;
    public static final CANBus canbus = new CANBus(Constants.canName);

    // Setpoints
    public static final double kSetpoint1 = 1;

    // Positional Limits
    public static final double kMinPosition = 0;
    public static final double kMaxPosition = 2;
    public static final double kPositionTolerance = 1.0;

    public static final boolean kPresetsCalibrated = true;

    // Motor data update rate (slow down if CAN util is > 70%)
    public static final double kStatusSignalUpdateRate = 50.0; // hz


    public static final TalonFXConfiguration ExampleConfigs = new TalonFXConfiguration()
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
        .withSlot0(new Slot0Configs() // PID tuning
            .withKP(0.0).withKI(0.0).withKD(0.0)
            .withKV(0.0).withKS(0.0).withKG(0.0)
        )
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false)
        );
}
