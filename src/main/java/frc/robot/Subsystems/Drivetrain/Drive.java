package frc.robot.Subsystems.Drivetrain;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain.DriveIO.DriveIOdata;
import com.ctre.phoenix6.swerve.*;




import java.util.List;

import javax.swing.text.AbstractDocument.BranchElement;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
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
    private HolonomicPathFollowerConfig holonomicPathFollowerConfig;

    private PIDController thetaController = new PIDController(10, 0, 0);

   


    private enum DriveMode {
        JOYSTICK_NORMAL,
        JOYSTICK_AUTOHEADING,
        AUTON_PATHPLANNER,
        AUTON_STATE
    }

    private DriveMode driveMode;

    private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
    private static final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric();
     private static final SwerveRequest.RobotCentric ROBOT_CENTRIC = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyChassisSpeeds AUTON_CHASSIS_SPEEDS = new SwerveRequest.ApplyChassisSpeeds();


    public Drive(DriveIO driveIO) { 
        this.driveIO = driveIO;
        this.iOdata = driveIO.update();

        heading = new Rotation2d(0);


        driveTab = Shuffleboard.getTab("Drive");
        speedEntry = driveTab.add("Speed", 0.0).getEntry();
        poseEntry = driveTab.add("Pose", new Double[] {0.0, 0.0, 0.0}).getEntry();

        driveMode = DriveMode.JOYSTICK_NORMAL;

        double driveBaseRadius = 0;
        for (var moduleLocation : this.iOdata.m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
            pidConstantsTranslation,
            pidConstantsTheta,
            DriveConstants.MAX_VELOCITY_METERS,
            driveBaseRadius,
            new ReplanningConfig()
            );

        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        configurePathPlanner();

        List<Translation2d> waypoints = PathPlannerPath.bezierFromPoses(List.of(
            new Pose2d(5.56, 7.4, Rotation2d.fromDegrees(0)),
            new Pose2d(8.1, 7.2, Rotation2d.fromDegrees(0)),
            new Pose2d(6.65, 4.6, Rotation2d.fromRadians(4.4))
        ));


        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        ringPath = new PathPlannerPath(
                waypoints,
                constraints,
                new GoalEndState(4.0, Rotation2d.fromRadians(0.0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        ringPath.preventFlipping = true;
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
        if (this.iOdata.state.speeds != null) {
            speedEntry.setDouble(Math.hypot(
                this.iOdata.state.speeds.vxMetersPerSecond,
                this.iOdata.state.speeds.vyMetersPerSecond));
        }
        if (this.iOdata.state.Pose != null) {
            poseEntry.setDoubleArray(new Double[]{
                this.iOdata.state.Pose.getX(), 
                this.iOdata.state.Pose.getY(), 
                this.iOdata.state.Pose.getRotation().getDegrees()});
        } 


    }

    public Command regenOTF() {
        return runOnce(() -> ringPath = ringPath.replan(this.iOdata.state.Pose, this.iOdata.state.speeds));
    }

    public Command pathOTF() {
        return AutoBuilder.followPath(ringPath);
    }

    public Command resetPidgeon() {
        return runOnce(() -> {driveIO.resetPidgeon();});
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : this.iOdata.m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            () -> this.iOdata.state.Pose, // Supplier of current robot pose
            (Pose2d location) -> {this.driveIO.seedFieldRelative(location);},  // Consumer for seeding pose against auto
            () -> this.iOdata.state.speeds,
            (speeds) -> driveIO.setSwerveRequest(AUTON_CHASSIS_SPEEDS.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            holonomicPathFollowerConfig,
            () -> false, // Supplier to flip the path, not needed without cameras
            this); // Subsystem
    }

    public Command seedPose(Pose2d seedling){
        return runOnce(() -> this.driveIO.seedFieldRelative(seedling));
    }
}
