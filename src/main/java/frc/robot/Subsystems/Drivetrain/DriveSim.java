package frc.robot.Subsystems.Drivetrain;


import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Subsystems.Drivetrain.DriveConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.Drivetrain.DriveConstants.DriveConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;

public class DriveSim extends SwerveDrivetrain  implements DriveIO {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private SwerveDriveState currentState;

    private DriveIOdata iOdata = new DriveIOdata();


    
    public DriveSim() {
        super(DriveConfig.DRIVETRAIN(), 
        DriveConfig.FRONT_LEFT(), 
        DriveConfig.FRONT_RIGHT(), 
        DriveConfig.BACK_LEFT(), 
        DriveConfig.BACK_RIGHT());
        startSimThread();

        for (int i = 0; i < ModuleCount; i++) { // this fixes the jitter in auton, because the sim motors have inertia
            Modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.SWERVE_DRIVE_GAINS, 1.0);
            Modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.SWERVE_STEER_GAINS, 1.0);

            // apply all types of current limits because 2024 ptsd :)
            Modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.DRIVE_CURRENT_LIMITS, 1.0);
            Modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.AZIMUTH_CURRENT_LIMITS, 1.0);

            Modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.DRIVE_TORQUE_CONFIGS, 1.0);
            Modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.AZIMUTH_TORQUE_CONFIGS, 1.0);
        }

        this.iOdata.m_moduleLocations = m_moduleLocations;
    }

    @Override
    public void setSwerveRequest(SwerveRequest requestToApply) {
        setControl(requestToApply);
    }

    @Override
    public DriveIOdata update() {
        updateSimState(.02, 12);
        this.currentState = getState();

        if (currentState != null) {
            this.iOdata.state = this.currentState;
        }   

        return this.iOdata;
    }

    public void setTeamRotation(DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
        } else {
            this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
