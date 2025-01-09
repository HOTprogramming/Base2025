package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.DriveIO.DriveIOdata;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;



public class Drive extends SubsystemBase {
    private DriveIO driveIO;
    private DriveIOdata iOdata;

    private ShuffleboardTab driveTab;
    private GenericEntry speedEntry;
    private GenericEntry poseEntry;

    private Rotation2d heading;



    private PIDController thetaController = new PIDController(10, 0, 0);

    private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric()
    .withDeadband(5.0 * 0.1).withRotationalDeadband(3.14 * 0.1);
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
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
            .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConfig.MAX_ANGULAR_VELOCITY())
        );

        heading = this.iOdata.state.Pose.getRotation();
    }

    public void robotCentricTeleopDrive(double driveX, double driveY, double driveTheta)  {
        driveIO.setSwerveRequest(ROBOT_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
             .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConfig.MAX_ANGULAR_VELOCITY())
              //.withRotationalRate(0.0);
              );
 
         heading = this.iOdata.state.Pose.getRotation();
     }
 
 
     public void headingControl(double driveX, double driveY) {
         driveIO.setSwerveRequest(FIELD_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
             .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), heading.getRadians()))
         );
         // System.out.println("head");
     }
 
 
     public void lockRotation(double driveX, double driveY, Rotation2d rtarget) {
         driveIO.setSwerveRequest(FIELD_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
             .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), rtarget.getRadians()))
         );
         heading = rtarget;
     }
 
     public void facePoint(double driveX, double driveY, Pose2d point) {
         driveIO.setSwerveRequest(FIELD_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
             .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(),
             Math.atan2(point.getY() - iOdata.state.Pose.getY(), point.getX() - iOdata.state.Pose.getX())))
         );        
         heading = this.iOdata.state.Pose.getRotation();
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


    public Command resetPidgeon() {
        return runOnce(() -> {driveIO.resetPidgeon();});
    }

    private void configurePathPlanner() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> this.iOdata.state.Pose,   // Supplier of robot pose
                this::setPose,         // Consumer for seeding pose
                () -> this.iOdata.state.Speeds, // Supplier of robot speeds
                // Consumer of ChassisSpeeds and feedforwards
                (speeds, feedforwards) -> driveIO.setSwerveRequest(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // translation
                    DriveConfig.AUTON_TRANSLATION_PID(),
                    // rotation
                    DriveConfig.AUTON_ROTATION_PID()
                ),
                config,
                // Assume the path doesnt flip (Sep auto files for red and blue side)
                () -> false,
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }
    private void setPose(Pose2d pose) {
        this.driveIO.seedFieldRelative(pose);
    }
}
