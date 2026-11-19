package frc.util.SubsystemExamples.Skeleton;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.SubsystemExamples.ExampleMotorInterface;

public class ExampleKraken implements ExampleMotorInterface {
    // declare IO
    public final TalonFX motor;

    private double target;
    private boolean targetActive;
    private boolean configOK;

    // status signals
    private final StatusSignal<Angle> ExamplePosition;
    private final StatusSignal<AngularVelocity> ExampleVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;

    public ExampleKraken() {
        motor = new TalonFX(
            ExampleConstants.kExampleMotorID, ExampleConstants.canbus);

        target = 0;
        targetActive = false;
        configOK = false;

        ExamplePosition = motor.getPosition();
        ExampleVelocity = motor.getVelocity();
        SupplyCurrent = motor.getSupplyCurrent();
        TorqueCurrent = motor.getTorqueCurrent();

    
        BaseStatusSignal.setUpdateFrequencyForAll(
            ExampleConstants.kStatusSignalUpdateRate,
            ExamplePosition,
            ExampleVelocity,
            SupplyCurrent,
            TorqueCurrent
          );

        // Configure motors
        configureIO(ExampleConstants.ExampleConfigs);
        stop();
    }

    /**
     * This exists to have different configs depending on different situations, 
     * such as different current limits for auto and teleop, or a possible 'limp mode' if excessive errors are detected
     */
    private void configureIO(TalonFXConfiguration kracken) {
        StatusCode motorStatus = motor.getConfigurator().apply(ExampleConstants.ExampleConfigs);
        configOK = motorStatus.isOK();
        if (!configOK) {
            System.err.println("Example config error. " + "motor=" + motorStatus);
        }
    }

	@Override
    public void update() {}

	@Override
    public void goToSetpoint(double position) {
        // do not execute if any major errors are present
        if (!ExampleConstants.kPresetsCalibrated 
                || !configOK
                || !Double.isFinite(position)
                || position < ExampleConstants.kMinPosition
                || position > ExampleConstants.kMaxPosition) {

            // Publish Errors (true = bad)
            SmartDashboard.putBoolean("Example/Command Too Low", position < ExampleConstants.kMinPosition);
            SmartDashboard.putBoolean("Example/Command Too High", position > ExampleConstants.kMaxPosition);
            SmartDashboard.putBoolean("Example/setpoint isnt finite", !Double.isFinite(position));
            SmartDashboard.putBoolean("Example/Config Bad", !configOK);
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
