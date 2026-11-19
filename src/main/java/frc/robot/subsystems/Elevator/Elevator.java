package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorInterface;

public class Elevator extends SubsystemBase {
    private final MotorInterface mElevatorKraken;
    private double commandedPosition;

    public Elevator() {
        mElevatorKraken = new ElevatorKraken();
        commandedPosition = ElevatorConstants.kSetpoint1;
    }

    public Command goToSetpoint1() {
        return elevatorCommand(ElevatorConstants.kSetpoint1);
    }

    public Command goToSetpoint2() {
        return elevatorCommand(ElevatorConstants.kSetpoint2);
    }

    public Command stopElevator() {
        return runOnce(mElevatorKraken::stop);
    }

	private FunctionalCommand elevatorCommand(double position){
		return new FunctionalCommand(
			() -> this.commandedPosition = position,
			() -> this.mElevatorKraken.goToPosition(position),
			interrupted -> this.mElevatorKraken.goToPosition(position),
			() -> this.mElevatorKraken.atTarget(),
			this);
	}

    public double getPosition() { return this.mElevatorKraken.getPosition(); }

    public double getCommandedPosition() { return this.commandedPosition; }

    
    @Override
    public void periodic() {
        mElevatorKraken.update();
        SmartDashboard.putNumber("Elevator/Position", mElevatorKraken.getPosition());
        SmartDashboard.putNumber("Elevator/Target", mElevatorKraken.getTarget());
        SmartDashboard.putBoolean("Elevator/At Target", mElevatorKraken.atTarget());
        SmartDashboard.putBoolean("Elevator/Control Active", mElevatorKraken.isActive());
        SmartDashboard.putBoolean("Elevator/Configured", mElevatorKraken.isConfigured());
    }
}