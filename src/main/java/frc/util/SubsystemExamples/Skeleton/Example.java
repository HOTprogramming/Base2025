package frc.util.SubsystemExamples.Skeleton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.SubsystemExamples.ExampleMotorInterface;

/**
 * To create your subsystem, copy this entire folder and change each instance of the word "Example" to your subsystem name
 * <p>
 * you should evaluate how many "components" your subsystem has. Each component should have their own "____Kracken" file.
 * components are explained in more detail in the <b>"ExampleMotorInterface.java"</b> file
 * <p>
 * This format assumes you have at least ONE motor. If you do not have any motors associated with your subsystem,
 * it may be a better choice NOT to follow this format.
 */
public class Example extends SubsystemBase {
    private final ExampleMotorInterface mExampleKraken;
    private double commandedState;

    public Example() {
        mExampleKraken = new ExampleKraken();
        commandedState = ExampleConstants.kSetpoint1;
    }

    public Command goToSetpoint1() {
        return ExampleCommand(ExampleConstants.kSetpoint1);
    }

    public Command stopExample() {
        return runOnce(mExampleKraken::stop);
    }

    /**
     * @param state Label what units this command takes!
     */
	private FunctionalCommand ExampleCommand(double state){
		return new FunctionalCommand(
			() -> this.commandedState = state,
			() -> this.mExampleKraken.goToSetpoint(state),
			interrupted -> this.mExampleKraken.goToSetpoint(state),
			() -> this.mExampleKraken.atTarget(),
			this);
	}

    /**
     * @return position/rpm in UNITS
     */
    public double getState() { return this.mExampleKraken.getState(); }

    public double getCommandedState() { return this.commandedState; }

    
    @Override
    public void periodic() {
        mExampleKraken.update();
        SmartDashboard.putNumber("Example/State", mExampleKraken.getState());
        SmartDashboard.putNumber("Example/Target", mExampleKraken.getTarget());
        SmartDashboard.putBoolean("Example/At Target", mExampleKraken.atTarget());
        SmartDashboard.putBoolean("Example/Control Active", mExampleKraken.isActive());
        SmartDashboard.putBoolean("Example/Configured", mExampleKraken.isConfigured());
    }
}