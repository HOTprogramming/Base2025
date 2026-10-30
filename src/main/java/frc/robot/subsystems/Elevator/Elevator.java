package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
	public final ElevatorKraken mShooterKraken = new ElevatorKraken();

	

	public Elevator() {
		

		
	}

	public double getShooterSpeed() {
		return mShooterKraken.m_krakenFXLeftTop.motor.getVelocity().getValueAsDouble();
	}

	public double getHoodAngle() {
		return mShooterKraken.m_krakenFXHood.motor.getPosition().getValueAsDouble();
	}

	@Override
	public void periodic() {
		mShooterKraken.update();
		// SmartDashboard.putNumber("target hood", targetHood);		
		// SmartDashboard.putNumber("target speed", targetSpeed);

	}

	public Command reverseShooter() {
		return Commands.sequence(
			runOnce(() -> mShooterKraken.runVelocity(0.1)),
			Commands.waitSeconds(0.5),
			run(() -> mShooterKraken.runVelocity(-20)));
	}

	
}