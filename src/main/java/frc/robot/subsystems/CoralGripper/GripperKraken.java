package frc.robot.subsystems.CoralGripper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MotorInterface;

/**
 * This "component" example has one motor and zero encoders. This component will work best 
 * for shooters and simple intakes that take in <b>velocity</b> setpoints
 * <p>
 * This skeleton is also good as a starting point for more complex/unique systems
 */
public class GripperKraken implements MotorInterface {
    // declare IO
    private final TalonFX motor;

    private double target;
    private boolean targetActive;
    private boolean configOK;
    private StatusCode subsystemFault;

    // status signals
    private final StatusSignal<Angle> GripperPosition;
    private final StatusSignal<AngularVelocity> GripperVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;

    public GripperKraken() {
        motor = new TalonFX(
            CoralGripperConstants.kGripperMotorID, CoralGripperConstants.canbus);
        

        target = 0;
        targetActive = false;
        configOK = false;
        subsystemFault = StatusCode.StatusCodeNotInitialized;

        GripperPosition = motor.getPosition();
        GripperVelocity = motor.getVelocity();
        SupplyCurrent = motor.getSupplyCurrent();
        TorqueCurrent = motor.getTorqueCurrent();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            CoralGripperConstants.kStatusSignalUpdateRate,
            GripperPosition,
            GripperVelocity,
            SupplyCurrent,
            TorqueCurrent
          );

        // Configure motors
        configureIO(CoralGripperConstants.GripperConfigs);
        stop();
    }

    /**
     * This exists to have different configs depending on different situations, 
     * such as different current limits for auto and teleop, or a possible 'limp mode' if excessive errors are detected
     */
    private void configureIO(TalonFXConfiguration krackenConfig) {
        StatusCode motorStatus = motor.getConfigurator().apply(krackenConfig);
        configOK = motorStatus.isOK();
        if (!configOK) {
            System.err.println("Gripper config error. " + "motor=" + motorStatus);
        }
    }

	@Override
    public void update() {
        this.subsystemFault = BaseStatusSignal.refreshAll(
            GripperPosition,
            GripperVelocity,
            SupplyCurrent,
            TorqueCurrent
        );
    }

	@Override
    public void goToSetpoint(double volts) {
        // do not execute if any major errors are present
        if (!CoralGripperConstants.kPresetsCalibrated 
                || !configOK
                || !Double.isFinite(volts)
                || volts < CoralGripperConstants.kMinVolts
                || volts > CoralGripperConstants.kMaxVolts
                || subsystemFault.isError()) {

            // Publish Errors (true = bad)
            SmartDashboard.putBoolean("Gripper/Command Too Low", volts < CoralGripperConstants.kMinVolts);
            SmartDashboard.putBoolean("Gripper/Command Too High", volts > CoralGripperConstants.kMaxVolts);
            SmartDashboard.putBoolean("Gripper/setpoint isnt finite", !Double.isFinite(volts));
            SmartDashboard.putBoolean("Gripper/Config Bad", !configOK);
            stop();
            return;
        }

        target = volts;
        targetActive = true;
        motor.setVoltage(volts);
    }

	@Override
    public double getState() {
        return GripperVelocity.getValueAsDouble();
    }

    public double getTarget() { return target; }

	@Override
    public boolean isActive() { return targetActive; }

	@Override
    public boolean isConfigured() { return configOK; }

    @Override
    public StatusCode getComponentStatus() {
        return this.subsystemFault;
    }

	@Override
    public boolean atTarget() {
        return false;
    }

	@Override
    public void stop() {
        targetActive = false;
        motor.stopMotor();
    }
}
