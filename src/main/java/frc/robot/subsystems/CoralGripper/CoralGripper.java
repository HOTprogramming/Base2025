package frc.robot.subsystems.CoralGripper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorInterface;

/**
 * To create your subsystem, copy this entire folder and change each instance of the word "Example" to your subsystem name
 * <p>
 * you should evaluate how many "components" your subsystem has. Each component should have their own "____Kracken" file.
 * components are explained in more detail in the <b>"ExampleMotorInterface.java"</b> file
 * <p>
 * This format assumes you have at least ONE motor with a fused caNcoder.
 */
public class CoralGripper extends SubsystemBase {
    private final MotorInterface mWristKraken;
    private final MotorInterface mGripperKracken;
    private final CoralSensor mCoralSensor;
    private double commandedAngle;
    private double commandedVolts;

    public CoralGripper() {
        mWristKraken = new WristKraken();
        mGripperKracken = new GripperKraken();
        mCoralSensor = new CoralSensor();
        commandedAngle = CoralGripperConstants.kPackage;
    }

    public Command goToHpIntake() {
        return goToFlat().alongWith(intakeRollers()).until(mCoralSensor::getHasCoral)
            .andThen(goToPackage());
    }

    public Command goToPackage() {
        return WristCommand(CoralGripperConstants.kPackage).alongWith(stopRollers());
    }

    public Command goToScore() {
        return WristCommand(CoralGripperConstants.kPackage);
    }

    public Command goToFlat() {
        return WristCommand(CoralGripperConstants.kFlat);
    }

    public Command slowSpit() {
        return GripperCommand(CoralGripperConstants.kSlowSpit);
    }

    public Command intakeRollers() {
        return GripperCommand(CoralGripperConstants.kIntake);
    }

    public Command stopRollers() {
        return runOnce(mGripperKracken::stop);
    }

    public Command stopCoralGripper() {
        return runOnce(() -> {
            mWristKraken.stop();
            mGripperKracken.stop();
        });
    }

	private FunctionalCommand WristCommand(double angle){
		return new FunctionalCommand(
			() -> this.commandedAngle = angle,
			() -> this.mWristKraken.goToSetpoint(angle),
			interrupted -> this.mWristKraken.goToSetpoint(angle),
			() -> this.mWristKraken.atTarget(),
			this);
	}

    /**
     * Command will not end, rollers will run until manually stopped
     */
    private FunctionalCommand GripperCommand(double voltage){
        return new FunctionalCommand(
			() -> this.commandedVolts = voltage,
			() -> this.mGripperKracken.goToSetpoint(voltage),
			interrupted -> this.mGripperKracken.goToSetpoint(voltage),
			() -> false,
			this);
    }

    public double getPosition() { return this.mWristKraken.getState(); }

    public double getCommandedPosition() { return this.commandedAngle; }

    public double getCommandedVolts() { return this.commandedVolts; }


    
    @Override
    public void periodic() {
        mWristKraken.update();
        SmartDashboard.putNumber("CoralGripper/Position", mWristKraken.getState());
        SmartDashboard.putNumber("CoralGripper/Target", mWristKraken.getTarget());
        SmartDashboard.putBoolean("CoralGripper/At Target", mWristKraken.atTarget());
        SmartDashboard.putBoolean("CoralGripper/Control Active", mWristKraken.isActive());
        SmartDashboard.putBoolean("CoralGripper/Configured", mWristKraken.isConfigured());
    }
}