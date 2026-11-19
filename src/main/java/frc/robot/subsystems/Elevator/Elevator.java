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
        commandedPosition = ElevatorConstants.kPackage;
    }

    public Command goToPackage() {
        return elevatorCommand(ElevatorConstants.kPackage);
    }

    public Command goToL3() {
        return elevatorCommand(ElevatorConstants.kL3);
    }

    public Command stopElevator() {
        return runOnce(mElevatorKraken::stop);
    }

	private FunctionalCommand elevatorCommand(double position){
		return new FunctionalCommand(
			() -> this.commandedPosition = position,
			() -> this.mElevatorKraken.goToSetpoint(position),
			interrupted -> this.mElevatorKraken.goToSetpoint(position),
			() -> this.mElevatorKraken.atTarget(),
			this);
	}

    public double getPosition() { return this.mElevatorKraken.getState(); }

    public double getCommandedPosition() { return this.commandedPosition; }

    
    @Override
    public void periodic() {
        mElevatorKraken.update();
        SmartDashboard.putNumber("Elevator/Position", mElevatorKraken.getState());
        SmartDashboard.putNumber("Elevator/Target", mElevatorKraken.getTarget());
        SmartDashboard.putBoolean("Elevator/At Target", mElevatorKraken.atTarget());
        SmartDashboard.putBoolean("Elevator/Control Active", mElevatorKraken.isActive());
        SmartDashboard.putBoolean("Elevator/Configured", mElevatorKraken.isConfigured());
    }
}