package frc.util.SubsystemExample;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorInterface;

public class Example extends SubsystemBase {
    private final MotorInterface mExampleKraken;
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
			() -> this.mExampleKraken.goToPosition(position),
			interrupted -> this.mExampleKraken.goToPosition(position),
			() -> this.mExampleKraken.atTarget(),
			this);
	}

    public double getPosition() { return this.mExampleKraken.getPosition(); }

    public double getCommandedPosition() { return this.commandedPosition; }

    
    @Override
    public void periodic() {
        mExampleKraken.update();
        SmartDashboard.putNumber("Example/Position", mExampleKraken.getPosition());
        SmartDashboard.putNumber("Example/Target", mExampleKraken.getTarget());
        SmartDashboard.putBoolean("Example/At Target", mExampleKraken.atTarget());
        SmartDashboard.putBoolean("Example/Control Active", mExampleKraken.isActive());
        SmartDashboard.putBoolean("Example/Configured", mExampleKraken.isConfigured());
    }
}