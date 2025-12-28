package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;

public class DriveSim extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>  implements DriveIO {
    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private Pigeon2 m_Pigeon2;

    private SwerveDriveState currentState;

    private DriveIOdata iOdata = new DriveIOdata();

    
    public DriveSim() {
        super(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            DriveConfig.DRIVETRAIN(),
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(
                    new SwerveModuleConstants[] {
                            DriveConfig.FRONT_LEFT(),
                            DriveConfig.FRONT_RIGHT(),
                            DriveConfig.BACK_LEFT(),
                            DriveConfig.BACK_RIGHT()
                    }
            )
    );

        startSimThread();

        m_Pigeon2 = getPigeon2();

        this.iOdata.m_moduleLocations = getModuleLocations();
    }

    public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    private void startSimThread() {
                mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                Pounds.of(115),
                Inches.of(30),
                Inches.of(30),
                DCMotor.getKrakenX60(1),
                DCMotor.getFalcon500(1),
                1.2,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                DriveConfig.FRONT_LEFT(),
                DriveConfig.FRONT_RIGHT(),
                DriveConfig.BACK_LEFT(),
                DriveConfig.BACK_RIGHT());
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void setCurrentLimits() { // used for setting different current limits for auton/teleop
        SwerveModule<TalonFX, TalonFX, CANcoder>[] m_modules = getModules();
        
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.driveInitialConfigs.CurrentLimits, 1.0);
            m_modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.steerInitialConfigs.CurrentLimits, 1.0);
            // apply these seperatly (not in a TalonFXConfiguration obj) to ensure PID doesn't get overwritten
            m_modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.driveInitialConfigs.TorqueCurrent, 1.0);
            m_modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.steerInitialConfigs.TorqueCurrent, 1.0);
        }
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

        this.iOdata.pigeon = m_Pigeon2.getRotation3d();

        return this.iOdata;
    }

    public void setTeamRotation(DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
        } else {
            this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.1); // wait for simulation to update
        super.resetPose(pose);
    }

    @Override
    public void seedFieldRelative(Pose2d seedling) {
        resetPose(seedling);
    }

    @Override
    public void updateVision(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
        addVisionMeasurement(calculatedPose, timestamp, stDevs);
    }

    @Override
    public void setOperatorPerspective(Rotation2d rotation2d) {
        setOperatorPerspectiveForward(rotation2d);
    }
    
    @Override
    public void setSimulationWorldPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            this.mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        }
    }

}
