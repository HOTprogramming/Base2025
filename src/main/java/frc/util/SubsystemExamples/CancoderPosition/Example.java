package frc.util.SubsystemExamples.CancoderPosition;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.SubsystemExamples.ExampleMotorInterface;

public class Example extends SubsystemBase {
    private final ExampleMotorInterface mExampleKraken;
    private double commandedPosition;

    public Example() {
        mExampleKraken = new ExampleKraken();
        commandedPosition = ExampleConstants.kSetpoint1;
    }

    public Command goToSetpoint1() {
        return ExampleCommand(ExampleConstants.kSetpoint1);
    }

    public Command stopExample() {
        return runOnce(mExampleKraken::stop);
    }

	private FunctionalCommand ExampleCommand(double position){
		return new FunctionalCommand(
			() -> this.commandedPosition = position,
			() -> this.mExampleKraken.goToSetpoint(position),
			interrupted -> this.mExampleKraken.goToSetpoint(position),
			() -> this.mExampleKraken.atTarget(),
			this);
	}

    public double getPosition() { return this.mExampleKraken.getState(); }

    public double getCommandedPosition() { return this.commandedPosition; }

    
    @Override
    public void periodic() {
        mExampleKraken.update();
        SmartDashboard.putNumber("Example/Position", mExampleKraken.getState());
        SmartDashboard.putNumber("Example/Target", mExampleKraken.getTarget());
        SmartDashboard.putBoolean("Example/At Target", mExampleKraken.atTarget());
        SmartDashboard.putBoolean("Example/Control Active", mExampleKraken.isActive());
        SmartDashboard.putBoolean("Example/Configured", mExampleKraken.isConfigured());
    }
}