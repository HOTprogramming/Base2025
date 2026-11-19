package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public final ElevatorKraken mElevatorKraken = new ElevatorKraken();

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
			() -> {},
			() -> mElevatorKraken.goToPosition(position),
			interrupted -> mElevatorKraken.goToPosition(position),
			() -> mElevatorKraken.atTarget(),
			this);
	}

	// private FunctionalCommand elevatorCommand(double position){
    // //return new FunctionalCommand(
    //   //() -> this.elevatorCommandedPos.setDouble(position),
    //   () -> io.setElevatorMotorControl(position),
    //   interrupted -> io.setElevatorMotorControl(position), 
    //   () -> checkRange(1.0),
    //   this);
  //}

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