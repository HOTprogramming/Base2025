package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.DriveIO.DriveIOdata;

import com.ctre.phoenix6.swerve.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;



public class Drive extends SubsystemBase {
    private DriveIO driveIO;
    private DriveIOdata iOdata;

    private ShuffleboardTab driveTab;
    private GenericEntry speedEntry;
    private GenericEntry poseEntry;

    private PathPlannerPath ringPath;

    private Rotation2d heading;

    private PIDConstants pidConstantsTranslation = new PIDConstants(10, 0, 0);
    private PIDConstants pidConstantsTheta = new PIDConstants (10, 0, 0); 

    private PIDController thetaController = new PIDController(10, 0, 0);

    private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric ROBOT_CENTRIC = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();



    public Drive(DriveIO driveIO) { 
        this.driveIO = driveIO;
        this.iOdata = driveIO.update();

        heading = new Rotation2d(0);


        driveTab = Shuffleboard.getTab("Drive");
        speedEntry = driveTab.add("Speed", 0.0).getEntry();
        poseEntry = driveTab.add("Pose", new Double[] {0.0, 0.0, 0.0}).getEntry();

        double driveBaseRadius = 0;
        for (var moduleLocation : this.iOdata.m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        configurePathPlanner();
    }

    public void teleopDrive(double driveX, double driveY, double driveTheta)  {
       driveIO.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConstants.MAX_VELOCITY_METERS)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConstants.MAX_VELOCITY_METERS)
            .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConstants.MAX_ANGULAR_VELOCITY_RADS)
        );

        heading = this.iOdata.state.Pose.getRotation();
        // System.out.println("Tele");
    }

    public void robotCentricTeleopDrive(double driveX, double driveY, double driveTheta)  {
       driveIO.setSwerveRequest(ROBOT_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConstants.MAX_VELOCITY_METERS)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConstants.MAX_VELOCITY_METERS)
            .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConstants.MAX_ANGULAR_VELOCITY_RADS)
             //.withRotationalRate(0.0);
             );

        heading = this.iOdata.state.Pose.getRotation();
        //System.out.println(driveTheta);
    }


    public void headingControl(double driveX, double driveY) {
        driveIO.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConstants.MAX_VELOCITY_METERS)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConstants.MAX_VELOCITY_METERS)
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), heading.getRadians()))
        );
        // System.out.println("head");
    }


    public void lockRotation(double driveX, double driveY, Rotation2d rtarget) {
        driveIO.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConstants.MAX_VELOCITY_METERS)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConstants.MAX_VELOCITY_METERS)
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), rtarget.getRadians()))
        );
        heading = rtarget;
        System.out.println("lock rotation");
    }

    public void facePoint(double driveX, double driveY, Pose2d point) {
        driveIO.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConstants.MAX_VELOCITY_METERS)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConstants.MAX_VELOCITY_METERS)
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(),
            Math.atan2(point.getY() - iOdata.state.Pose.getY(), point.getX() - iOdata.state.Pose.getX())))
        );        
        heading = this.iOdata.state.Pose.getRotation();
        System.out.println("look at goal");
    }

    public Command brake() {
        return run(() -> driveIO.setSwerveRequest(BRAKE));
    }

    @Override
    public void periodic() {
        this.iOdata = driveIO.update();
        if (this.iOdata.state.Speeds != null) {
            speedEntry.setDouble(Math.hypot(
                this.iOdata.state.Speeds.vxMetersPerSecond,
                this.iOdata.state.Speeds.vyMetersPerSecond));
        }
        if (this.iOdata.state.Pose != null) {
            poseEntry.setDoubleArray(new Double[]{
                this.iOdata.state.Pose.getX(), 
                this.iOdata.state.Pose.getY(), 
                this.iOdata.state.Pose.getRotation().getDegrees()});
        } 


    }

    public Command pathOTF() {
        return AutoBuilder.followPath(ringPath);
    }

    public Command resetPidgeon() {
        return runOnce(() -> {driveIO.resetPidgeon();});
    }

    private void configurePathPlanner() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> this.iOdata.state.Pose,   // Supplier of current robot pose
                this::seedPose,         // Consumer for seeding pose against auto
                () -> this.iOdata.state.Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> driveIO.setSwerveRequest(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> false,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public Command seedPose(Pose2d seedling){
        return runOnce(() -> this.driveIO.seedFieldRelative(seedling));
    }
}
