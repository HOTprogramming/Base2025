package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MotorInterface;

/** Two Kraken X60 TalonFX motors using the sample's leader/follower CANcoder approach. */
public class ElevatorKraken implements MotorInterface {
    public final TalonFX motor = new TalonFX(
        ElevatorConstants.kelevatorMotorID, ElevatorConstants.canbus);
    public final TalonFX motor2 = new TalonFX(
        ElevatorConstants.kelevatorMotor2ID, ElevatorConstants.canbus);
    public final CANcoder encoder = new CANcoder(
        ElevatorConstants.kelevatorEncoderID, ElevatorConstants.canbus);

    private final MotionMagicExpoTorqueCurrentFOC motionMagic =
        new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
    private final Follower follower = new Follower(motor.getDeviceID(), true);
    private double target = 0;
    private boolean targetActive = false;
    private boolean configOK = false;

    public ElevatorKraken() {
        StatusCode encoderStatus = encoder.getConfigurator().apply(ElevatorConstants.EncoderConfigs);
        StatusCode leaderStatus = motor.getConfigurator().apply(ElevatorConstants.ElevatorConfigs);
        StatusCode followerStatus = motor2.getConfigurator().apply(ElevatorConstants.ElevatorConfigs);
        configOK = encoderStatus.isOK() && leaderStatus.isOK() && followerStatus.isOK();
        if (!configOK) {
            System.err.println("Elevator config error. Encoder=" + encoderStatus
                + " leader=" + leaderStatus + " follower=" + followerStatus);
        }
        stop();
    }

	@Override
    public void update() {
        if (targetActive) motor2.setControl(follower);
    }

	@Override
    public void goToPosition(double position) {
        if (!ElevatorConstants.kPresetsCalibrated || !configOK
            || !Double.isFinite(position)
            || position < ElevatorConstants.kMinPosition
            || position > ElevatorConstants.kMaxPosition
            || !encoder.getPosition().refresh().getStatus().isOK()) {
				//SmartDashboard.getBoolean("elevator1", position < ElevatorConstants.kMinPosition);
				//SmartDashboard.putBoolean("elevatr2", position > ElevatorConstants.kMaxPosition);
				SmartDashboard.putBoolean("Finite", !Double.isFinite(position));
				//SmartDashboard.putBoolean("Config", configOK);
				//SmartDashboard.putBoolean("encoder", encoder.getPosition().refresh().getStatus().isOK());
           stop();
            return;
        }
        target = position;
        targetActive = true;
        motor2.setControl(follower);
        motor.setControl(motionMagic.withPosition(target));
    }

	@Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getTarget() { return target; }

	@Override
    public boolean isActive() { return targetActive; }

	@Override
    public boolean isConfigured() { return configOK; }

	@Override
    public boolean atTarget() {
        return targetActive && Math.abs(getPosition() - target) <= ElevatorConstants.kPositionTolerance;
    }

	@Override
    public void stop() {
        targetActive = false;
        motor.stopMotor();
        motor2.stopMotor();
    }
}
