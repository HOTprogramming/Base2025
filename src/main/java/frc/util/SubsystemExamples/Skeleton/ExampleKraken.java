package frc.util.SubsystemExamples.Skeleton;

import java.util.concurrent.PriorityBlockingQueue;

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

/**
 * This "component" example has one motor and zero encoders. This component will work best 
 * for shooters and simple intakes that take in <b>velocity</b> setpoints
 * <p>
 * This skeleton is also good as a starting point for more complex/unique systems
 */
public class ExampleKraken implements ExampleMotorInterface {
    // declare IO
    private final TalonFX motor;

    private double target;
    private boolean targetActive;
    private boolean configOK;
    private StatusCode subsystemFault;

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
        subsystemFault = StatusCode.StatusCodeNotInitialized;

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
    public void update() {
        this.subsystemFault = BaseStatusSignal.refreshAll(
            ExamplePosition,
            ExampleVelocity,
            SupplyCurrent,
            TorqueCurrent
        );
    }

	@Override
    public void goToSetpoint(double setpoint) {
        // do not execute if any major errors are present
        if (!ExampleConstants.kPresetsCalibrated 
                || !configOK
                || !Double.isFinite(setpoint)
                || setpoint < ExampleConstants.kMinSetpoint
                || setpoint > ExampleConstants.kMaxSetpoint
                || subsystemFault.isError()) {

            // Publish Errors (true = bad)
            SmartDashboard.putBoolean("Example/Command Too Low", setpoint < ExampleConstants.kMinSetpoint);
            SmartDashboard.putBoolean("Example/Command Too High", setpoint > ExampleConstants.kMaxSetpoint);
            SmartDashboard.putBoolean("Example/setpoint isnt finite", !Double.isFinite(setpoint));
            SmartDashboard.putBoolean("Example/Config Bad", !configOK);
            stop();
            return;
        }

        target = setpoint;
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
        return targetActive && Math.abs(getState() - target) <= ExampleConstants.kSetpointTolerance;
    }

	@Override
    public void stop() {
        targetActive = false;
        motor.stopMotor();
    }
}
