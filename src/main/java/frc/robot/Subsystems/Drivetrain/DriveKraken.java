package frc.robot.Subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Subsystems.Drivetrain.DriveConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.pathplanner.lib.auto.AutoBuilder;




public class DriveKraken extends SwerveDrivetrain implements DriveIO {


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    


    private SwerveDriveState currentState;

    private DriveIOdata iOdata = new DriveIOdata();



    public DriveKraken() {
        // drivetrain constants
        super(DriveConfig.DRIVETRAIN(), 
        DriveConfig.FRONT_LEFT(), 
        DriveConfig.FRONT_RIGHT(), 
        DriveConfig.BACK_LEFT(), 
        DriveConfig.BACK_RIGHT());

        for (int i = 0; i < ModuleCount; i++) {
            Modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.SWERVE_DRIVE_GAINS, 1.0);
            Modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.SWERVE_STEER_GAINS, 1.0);

            // apply all types of current limits because 2024 ptsd :)
            Modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.DRIVE_CURRENT_LIMITS, 1.0);
            Modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.AZIMUTH_CURRENT_LIMITS, 1.0);

            Modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.DRIVE_TORQUE_CONFIGS, 1.0);
            Modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.AZIMUTH_TORQUE_CONFIGS, 1.0);
        }

        this.iOdata.m_moduleLocations = getModuleLocations();
    }

    @Override
    public void setSwerveRequest(SwerveRequest requestToApply) {
        setControl(requestToApply);
    }

    

    @Override
    public DriveIOdata update() {
        this.currentState = getState();

        if (currentState != null) {
            this.iOdata.state = this.currentState;
        }

        // TODO ADD MODULE DATA
        return this.iOdata;
    }

    // @Override
    // public void seedFieldRelative(Pose2d seedling) {
        
    //     this.seedFieldRelative(seedling);
    // }

    @Override
    public void setTeamRotation(Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
        } else {
            this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
        }
    }

    @Override
    public void resetPidgeon() {
        m_pigeon2.setYaw(0);
    }
}
