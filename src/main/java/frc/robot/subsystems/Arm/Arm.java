package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorInterface;


public class Arm extends SubsystemBase {
    private final MotorInterface mArmKraken;
    private double commandedPosition;

    public Arm() {
        mArmKraken = new ArmKraken();
        commandedPosition = ArmConstants.kPackage;
    }

    public Command goToPackage() {
        return ArmCommand(ArmConstants.kPackage);
    }

    public Command horizontal() {
        return ArmCommand(ArmConstants.kHorizontal);
    }

    public Command stopArm() {
        return runOnce(mArmKraken::stop);
    }

	private FunctionalCommand ArmCommand(double position){
		return new FunctionalCommand(
			() -> this.commandedPosition = position,
			() -> this.mArmKraken.goToSetpoint(position),
			interrupted -> this.mArmKraken.goToSetpoint(position),
			() -> this.mArmKraken.atTarget(),
			this);
	}

    public double getPosition() { return this.mArmKraken.getState(); }

    public double getCommandedPosition() { return this.commandedPosition; }

    
    @Override
    public void periodic() {
        mArmKraken.update();
        SmartDashboard.putNumber("Arm/Position", mArmKraken.getState());
        SmartDashboard.putNumber("Arm/Target", mArmKraken.getTarget());
        SmartDashboard.putBoolean("Arm/At Target", mArmKraken.atTarget());
        SmartDashboard.putBoolean("Arm/Control Active", mArmKraken.isActive());
        SmartDashboard.putBoolean("Arm/Configured", mArmKraken.isConfigured());
    }
}