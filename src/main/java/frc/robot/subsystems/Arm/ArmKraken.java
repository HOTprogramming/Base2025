package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MotorInterface;


public class ArmKraken implements MotorInterface {
    private final CANcoder encoder;
    private final TalonFX motor;

    private final MotionMagicExpoTorqueCurrentFOC motionMagic;

    private double target;
    private boolean targetActive;
    private boolean configOK;
    private StatusCode subsystemFault;

    // status signals
    private final StatusSignal<Angle> ArmPosition;
    private final StatusSignal<AngularVelocity> ArmVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;

    private final StatusSignal<Angle> ArmCancoderPosition;

    public ArmKraken() {
        encoder = new CANcoder(
            ArmConstants.kArmEncoderID, ArmConstants.canbus);
        motor = new TalonFX(
            ArmConstants.kArmMotorID, ArmConstants.canbus);

        motionMagic = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);

        target = 0;
        targetActive = false;
        configOK = false;
        subsystemFault = StatusCode.StatusCodeNotInitialized;

        ArmPosition = motor.getPosition();
        ArmVelocity = motor.getVelocity();
        SupplyCurrent = motor.getSupplyCurrent();
        TorqueCurrent = motor.getTorqueCurrent();

        ArmCancoderPosition = encoder.getPosition();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            ArmConstants.kStatusSignalUpdateRate,
            ArmPosition,
            ArmVelocity,
            SupplyCurrent,
            TorqueCurrent,
            ArmCancoderPosition
          );

        motor.optimizeBusUtilization();
        encoder.optimizeBusUtilization();

        // Configure motors
        configureIO(ArmConstants.ArmEncoderConfigs, ArmConstants.ArmConfigs);
        stop();
    }

    /**
     * This exists to have different configs depending on different situations, 
     * such as different current limits for auto and teleop, or a possible 'limp mode' if excessive errors are detected
     */
    private void configureIO(CANcoderConfiguration cancoderConfig, TalonFXConfiguration krackenConfig) {
        StatusCode encoderStatus = encoder.getConfigurator().apply(cancoderConfig);
        StatusCode motorStatus = motor.getConfigurator().apply(krackenConfig);
        configOK = encoderStatus.isOK() && motorStatus.isOK();
        if (!configOK) {
            System.err.println("Arm config error. Encoder=" + encoderStatus
                + " motor=" + motorStatus);
        }
    }

	@Override
    public void update() {
        this.subsystemFault = BaseStatusSignal.refreshAll(
            ArmPosition,
            ArmVelocity,
            SupplyCurrent,
            TorqueCurrent,
            ArmCancoderPosition
        );
    }

    /**
     * @param angle in degrees (package is -90)
     */
	@Override
    public void goToSetpoint(double angle) {
        // do not execute if any major errors are present
        if (!ArmConstants.kPresetsCalibrated 
                || !configOK
                || !Double.isFinite(angle)
                || angle < ArmConstants.kMinPosition
                || angle > ArmConstants.kMaxPosition
                || subsystemFault.isError()) {

            // Publish Errors
            SmartDashboard.putBoolean("Arm/Command Too Low", angle < ArmConstants.kMinPosition);
            SmartDashboard.putBoolean("Arm/Command Too High", angle > ArmConstants.kMaxPosition);
            SmartDashboard.putBoolean("Arm/position isnt finite", !Double.isFinite(angle));
            SmartDashboard.putBoolean("Arm/Config Bad", !configOK);
            SmartDashboard.putBoolean("Arm/Encoder Bad", !ArmCancoderPosition.refresh().getStatus().isOK());
            stop();
            return;
        }

        target = angle;
        targetActive = true;
        motor.setControl(motionMagic.withPosition(angle).withSlot(0));
    }

    /**
     * Angle in degrees (package is -90)
     */
	@Override
    public double getState() {
        return ArmPosition.getValueAsDouble();
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
        return targetActive && Math.abs(getState() - target) <= ArmConstants.kPositionTolerance;
    }
    
	@Override
    public void stop() {
        targetActive = false;
        motor.stopMotor();
    }
}
