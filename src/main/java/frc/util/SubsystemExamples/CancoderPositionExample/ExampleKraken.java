package frc.util.SubsystemExamples.CancoderPositionExample;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.SubsystemExamples.ExampleMotorInterface;;

public class ExampleKraken implements ExampleMotorInterface {
    public final CANcoder encoder;
    public final TalonFX motor;

    private double target;
    private boolean targetActive;
    private boolean configOK;

    // status signals
    private final StatusSignal<Angle> ExamplePosition;
    private final StatusSignal<AngularVelocity> ExampleVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;

    private final StatusSignal<Angle> ExampleCancoderPosition;

    public ExampleKraken() {
        encoder = new CANcoder(
            ExampleConstants.kExampleEncoderID, ExampleConstants.canbus);
        motor = new TalonFX(
            ExampleConstants.kExampleMotorID, ExampleConstants.canbus);

        target = 0;
        targetActive = false;
        configOK = false;

        ExamplePosition = motor.getPosition();
        ExampleVelocity = motor.getVelocity();
        SupplyCurrent = motor.getSupplyCurrent();
        TorqueCurrent = motor.getTorqueCurrent();

        ExampleCancoderPosition = encoder.getPosition();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            ExampleConstants.kStatusSignalUpdateRate,
            ExamplePosition,
            ExampleVelocity,
            SupplyCurrent,
            TorqueCurrent,
            ExampleCancoderPosition
          );

        // Configure motors
        configureIO(ExampleConstants.EncoderConfigs, ExampleConstants.ExampleConfigs);
        stop();
    }

    /**
     * This exists to have different configs depending on different situations, 
     * such as different current limits for auto and teleop, or a possible 'limp mode' if excessive errors are detected
     */
    private void configureIO(CANcoderConfiguration cancoderConfig, TalonFXConfiguration kracken) {
        StatusCode encoderStatus = encoder.getConfigurator().apply(ExampleConstants.EncoderConfigs);
        StatusCode motorStatus = motor.getConfigurator().apply(ExampleConstants.ExampleConfigs);
        configOK = encoderStatus.isOK() && motorStatus.isOK();
        if (!configOK) {
            System.err.println("Example config error. Encoder=" + encoderStatus
                + " motor=" + motorStatus);
        }
    }

	@Override
    public void update() {
    }

	@Override
    public void goToSetpoint(double position) {
        // do not execute if any major errors are present
        if (!ExampleConstants.kPresetsCalibrated 
                || !configOK
                || !Double.isFinite(position)
                || position < ExampleConstants.kMinPosition
                || position > ExampleConstants.kMaxPosition
                || !ExampleCancoderPosition.refresh().getStatus().isOK()) {

            // Publish Errors
            SmartDashboard.putBoolean("Example/Command Too Low", position < ExampleConstants.kMinPosition);
            SmartDashboard.putBoolean("Example/Command Too High", position > ExampleConstants.kMaxPosition);
            SmartDashboard.putBoolean("Example/position isnt finite", !Double.isFinite(position));
            SmartDashboard.putBoolean("Example/Config Bad", !configOK);
            SmartDashboard.putBoolean("Example/Encoder Bad", !ExampleCancoderPosition.refresh().getStatus().isOK());
            stop();
            return;
        }

        target = position;
        targetActive = true;
        motor.setControl(new NeutralOut());
    }

	@Override
    public double getState() {
        return ExamplePosition.getValueAsDouble();
    }

    public double getTarget() { return target; }

	@Override
    public boolean isActive() { return targetActive; }

	@Override
    public boolean isConfigured() { return configOK; }

	@Override
    public boolean atTarget() {
        return targetActive && Math.abs(getState() - target) <= ExampleConstants.kPositionTolerance;
    }

	@Override
    public void stop() {
        targetActive = false;
        motor.stopMotor();
    }
}
