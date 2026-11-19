package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MotorInterface;

/** Two Kraken X60 TalonFX motors using the sample's leader/follower CANcoder approach. */
public class ElevatorKraken implements MotorInterface {
    public final CANcoder encoder;
    public final TalonFX motor;
    public final TalonFX motor2;

    private final MotionMagicExpoTorqueCurrentFOC motionMagic;
    private final Follower follower;

    private double target;
    private boolean targetActive;
    private boolean configOK;
    private StatusCode subsystemFault;

    // status signals
    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;

    private final StatusSignal<Angle> elevatorPosition2;
    private final StatusSignal<AngularVelocity> elevatorVelocity2;
    private final StatusSignal<Current> SupplyCurrent2;
    private final StatusSignal<Current> TorqueCurrent2;

    private final StatusSignal<Angle> elevatorCancoderPosition;

    public ElevatorKraken() {
        encoder = new CANcoder(
            ElevatorConstants.kelevatorEncoderID, ElevatorConstants.canbus);
        motor = new TalonFX(
            ElevatorConstants.kelevatorMotorID, ElevatorConstants.canbus);
        motor2 = new TalonFX(
            ElevatorConstants.kelevatorMotor2ID, ElevatorConstants.canbus);

        motionMagic = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
        follower = new Follower(motor.getDeviceID(), true);

        target = 0;
        targetActive = false;
        configOK = false;
        subsystemFault = StatusCode.StatusCodeNotInitialized;

        elevatorPosition = motor.getPosition();
        elevatorVelocity = motor.getVelocity();
        SupplyCurrent = motor.getSupplyCurrent();
        TorqueCurrent = motor.getTorqueCurrent();

        elevatorPosition2 = motor2.getPosition();
        elevatorVelocity2 = motor2.getVelocity();
        SupplyCurrent2 = motor2.getSupplyCurrent();
        TorqueCurrent2 = motor2.getTorqueCurrent();
        elevatorCancoderPosition = encoder.getPosition();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            ElevatorConstants.kStatusSignalUpdateRate,
            elevatorPosition,
            elevatorVelocity,
            SupplyCurrent,
            TorqueCurrent,
            elevatorPosition2,
            elevatorVelocity2,
            SupplyCurrent2,
            TorqueCurrent2,
            elevatorCancoderPosition
          );

        motor.optimizeBusUtilization();
        motor2.optimizeBusUtilization();
        encoder.optimizeBusUtilization();

        // Configure motors
        configureIO(ElevatorConstants.ElevatorEncoderConfigs, ElevatorConstants.ElevatorConfigs, ElevatorConstants.ElevatorConfigs);
        stop();
    }

    /**
     * This exists to have different configs depending on different situations, 
     * such as different current limits for auto and teleop, or a possible 'limp mode' if excessive errors are detected
     */
    private void configureIO(CANcoderConfiguration cancoderConfig, TalonFXConfiguration kracken1Config, TalonFXConfiguration kracken2Config) {
        StatusCode encoderStatus = encoder.getConfigurator().apply(cancoderConfig);
        StatusCode leaderStatus = motor.getConfigurator().apply(kracken1Config);
        StatusCode followerStatus = motor2.getConfigurator().apply(kracken2Config);
        configOK = encoderStatus.isOK() && leaderStatus.isOK() && followerStatus.isOK();
        if (!configOK) {
            System.err.println("Elevator config error. Encoder=" + encoderStatus
                + " leader=" + leaderStatus + " follower=" + followerStatus);
        }
    }

	@Override
    public void update() {
        this.subsystemFault = BaseStatusSignal.refreshAll(
            elevatorPosition,
            elevatorVelocity,
            SupplyCurrent,
            TorqueCurrent,
            elevatorPosition2,
            elevatorVelocity2,
            SupplyCurrent2,
            TorqueCurrent2,
            elevatorCancoderPosition
          );
        if (targetActive) motor2.setControl(follower);
    }

    /**
     * @param position in inches (package is 10)
     */
	@Override
    public void goToSetpoint(double position) {
        // do not execute if any major errors are present
        if (!ElevatorConstants.kPresetsCalibrated 
                || !configOK
                || !Double.isFinite(position)
                || position < ElevatorConstants.kMinPosition
                || position > ElevatorConstants.kMaxPosition
                || subsystemFault.isError()) {

            // Publish Errors
            SmartDashboard.putBoolean("Elevator/Command Too Low", position < ElevatorConstants.kMinPosition);
            SmartDashboard.putBoolean("Elevator/Command Too High", position > ElevatorConstants.kMaxPosition);
            SmartDashboard.putBoolean("Elevator/position isnt finite", !Double.isFinite(position));
            SmartDashboard.putBoolean("Elevator/Config Bad", !configOK);
            SmartDashboard.putBoolean("Elevator/Encoder Bad", !elevatorCancoderPosition.refresh().getStatus().isOK());
            stop();
            return;
        }

        target = position;
        targetActive = true;
        motor2.setControl(follower);
        motor.setControl(motionMagic.withPosition(target));
    }

    /**
     * @return Position in inches (package is 10)
     */
	@Override
    public double getState() {
        return elevatorPosition.getValueAsDouble();
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
        return targetActive && Math.abs(getState() - target) <= ElevatorConstants.kPositionTolerance;
    }

	@Override
    public void stop() {
        targetActive = false;
        motor.stopMotor();
        motor2.stopMotor();
    }
}
