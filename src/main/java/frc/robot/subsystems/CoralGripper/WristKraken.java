package frc.robot.subsystems.CoralGripper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MotorInterface;

/**
 * This "component" example has one motor and a fused encoder. This component will work best 
 * for elevators and arms that take in <b>positional</b> setpoints
 */
public class WristKraken implements MotorInterface {
    private final CANcoder encoder;
    private final TalonFXS motor;

    private final PositionVoltage positionVoltage;

    private double target;
    private boolean targetActive;
    private boolean configOK;
    private StatusCode subsystemFault;

    // status signals
    private final StatusSignal<Angle> WristPosition;
    private final StatusSignal<AngularVelocity> WristVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;

    private final StatusSignal<Angle> WristCancoderPosition;

    public WristKraken() {
        encoder = new CANcoder(
            CoralGripperConstants.kWristEncoderID, CoralGripperConstants.canbus);
        motor = new TalonFXS(
            CoralGripperConstants.kWristMotorID, CoralGripperConstants.canbus);

        positionVoltage = new PositionVoltage(0);

        target = 0;
        targetActive = false;
        configOK = false;
        subsystemFault = StatusCode.StatusCodeNotInitialized;


        WristPosition = motor.getPosition();
        WristVelocity = motor.getVelocity();
        SupplyCurrent = motor.getSupplyCurrent();
        TorqueCurrent = motor.getTorqueCurrent();

        WristCancoderPosition = encoder.getPosition();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            CoralGripperConstants.kStatusSignalUpdateRate,
            WristPosition,
            WristVelocity,
            SupplyCurrent,
            TorqueCurrent,
            WristCancoderPosition
          );

        // Configure motors
        configureIO(CoralGripperConstants.WristEncoderConfigs, CoralGripperConstants.WristConfigs);
        stop();
    }

    /**
     * This exists to have different configs depending on different situations, 
     * such as different current limits for auto and teleop, or a possible 'limp mode' if excessive errors are detected
     */
    private void configureIO(CANcoderConfiguration cancoderConfig, TalonFXSConfiguration fxsConfig) {
        StatusCode encoderStatus = encoder.getConfigurator().apply(cancoderConfig);
        StatusCode motorStatus = motor.getConfigurator().apply(fxsConfig);
        configOK = encoderStatus.isOK() && motorStatus.isOK();
        if (!configOK) {
            System.err.println("Wrist config error. Encoder=" + encoderStatus
                + " motor=" + motorStatus);
        }
    }

	@Override
    public void update() {
        this.subsystemFault = BaseStatusSignal.refreshAll(
            WristPosition,
            WristVelocity,
            SupplyCurrent,
            TorqueCurrent,
            WristCancoderPosition
        );
    }

	@Override
    public void goToSetpoint(double angle) {
        // do not execute if any major errors are present
        if (!CoralGripperConstants.kPresetsCalibrated 
                || !configOK
                || !Double.isFinite(angle)
                || angle < CoralGripperConstants.kMinAngle
                || angle > CoralGripperConstants.kMaxAngle
                || subsystemFault.isError()) {

            // Publish Errors
            SmartDashboard.putBoolean("Wrist/Command Too Low", angle < CoralGripperConstants.kMinAngle);
            SmartDashboard.putBoolean("Wrist/Command Too High", angle > CoralGripperConstants.kMaxAngle);
            SmartDashboard.putBoolean("Wrist/position isnt finite", !Double.isFinite(angle));
            SmartDashboard.putBoolean("Wrist/Config Bad", !configOK);
            SmartDashboard.putBoolean("Wrist/Encoder Bad", !WristCancoderPosition.refresh().getStatus().isOK());
            stop();
            return;
        }

        target = angle;
        targetActive = true;
        motor.setControl(positionVoltage.withPosition(angle).withSlot(0));
    }

	@Override
    public double getState() {
        return WristPosition.getValueAsDouble();
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
        return targetActive && Math.abs(getState() - target) <= CoralGripperConstants.kPositionTolerance;
    }

	@Override
    public void stop() {
        targetActive = false;
        motor.stopMotor();
    }
}
