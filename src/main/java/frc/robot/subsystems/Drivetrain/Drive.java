package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.DriveIO.DriveIOdata;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;



public class Drive extends SubsystemBase {
    private DriveIO driveIO;
    private DriveIOdata iOdata;

    private ShuffleboardTab driveTab;
    private GenericEntry speedEntry;
    private GenericEntry poseEntry;
    private GenericEntry pathXEntry;
    private GenericEntry pathYEntry;
    private GenericEntry pathRotEntry;
    private GenericEntry pathGoalPoseEntry;
    private GenericEntry matchTimeEntry;
    private GenericEntry voltsEntry;
    private GenericEntry ampsEntry;

    private double matchTime;
    private double volts;
    private double amps;

    private Rotation2d heading;

    private PathConstraints constraints;
    private List<Waypoint> waypoints;

    private PathPlannerPath path;
    private Pose2d reefTarget;
    private Pose2d objectAbsolute;
    private Pose2d objectRelative;
    private Pose2d pathGoalPose;

    private Alert alert;

    private PIDController thetaController = new PIDController(10, 0, 0);

    private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric()
    .withDeadband(5.0 * 0.1).withRotationalDeadband(3.14 * 0.1);
    private final SwerveRequest.RobotCentric ROBOT_CENTRIC = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private int currentPathIndex;
    private List<PathPlannerPath> pathGroup;


    public Drive(DriveIO driveIO) { 
        this.driveIO = driveIO;
        this.iOdata = driveIO.update();

        heading = Rotation2d.fromDegrees(0);

        driveTab = Shuffleboard.getTab("Drive");
        speedEntry = driveTab.add("Speed", 0.0).getEntry();
        poseEntry = driveTab.add("Pose", new Double[] {0.0, 0.0, 0.0}).getEntry();
        pathXEntry = driveTab.add("Path X", 0.0).getEntry();
        pathYEntry = driveTab.add("Path Y", 0.0).getEntry();
        pathRotEntry = driveTab.add("Path Rot", 0.0).getEntry();
        pathGoalPoseEntry = driveTab.add("pathGoal", new Double[] {0.0, 0.0, 0.0}).getEntry();
        matchTimeEntry = driveTab.add("Match time",0.0).getEntry();
        voltsEntry = driveTab.add("Volts",0.0).getEntry();
        ampsEntry = driveTab.add("Amps",0.0).getEntry();

        double driveBaseRadius = 0;
        for (var moduleLocation : this.iOdata.m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        pathGroup = new ArrayList<PathPlannerPath>();

        configurePathPlanner();

        constraints = PathConstraints.unlimitedConstraints(12.0);
        reefTarget = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        objectRelative = new Pose2d(2, 3, Rotation2d.fromDegrees(0));

        try {
            this.pathGroup.addAll(PathPlannerAuto.getPathGroupFromAutoFile("OTF_TESTING"));
        } catch (IOException e) {
            System.out.println("Auto String Invalid");
        } catch (ParseException e) {
            System.out.println("Auto Data Invalid");
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(13.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(16.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(15.0, 3.0, Rotation2d.fromDegrees(90))
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        alert = new Alert("test alert", AlertType.kWarning);
    }

    public boolean drivetrainAtTarget() {
        if (path != null) {
            
            return Math.abs(
                this.iOdata.state.Pose.getTranslation().getDistance(
                    path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation())) 
                    < OTF_end_tolerance;
        }
        return false;
    }

    public Command generateOnTheFly() {
        return runOnce(() -> {
            SmartDashboard.putBoolean("Path Generated", true);

            System.err.println("Start Gen");
            List<Pose2d> currentPathPoses = pathGroup.get(currentPathIndex).getPathPoses();
            PathPlannerPath nextPath = pathGroup.get(currentPathIndex + 1);
            waypoints = PathPlannerPath.waypointsFromPoses(
                currentPathPoses.get(currentPathPoses.size() - 1),
                new Pose2d(pathXEntry.getDouble(0), pathYEntry.getDouble(0), Rotation2d.fromDegrees(pathRotEntry.getDouble(0))),
                nextPath.getPathPoses().get(0)
            );
            path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(nextPath.getIdealStartingState().velocityMPS(), nextPath.getInitialHeading()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
            System.err.println("Finish Gen");
            AutoBuilder.followPath(path).unless(this::drivetrainAtTarget).schedule();
        });
    }

    public Command runOnTheFly() {
        return Commands.sequence(
            generateOnTheFly(),
            AutoBuilder.followPath(path).unless(this::drivetrainAtTarget));
    }

    public void chaseObject(int leftRight) {
        objectAbsolute = new Pose2d(
            (iOdata.state.Pose.getX() + (iOdata.state.Pose.getRotation().plus(objectRelative.getTranslation().getAngle()).getCos() * objectRelative.getTranslation().getNorm())) - (iOdata.state.Pose.getRotation().plus(objectRelative.getRotation()).getSin() * 0.1524 * leftRight),
            (iOdata.state.Pose.getY() + (iOdata.state.Pose.getRotation().plus(objectRelative.getTranslation().getAngle()).getSin() * objectRelative.getTranslation().getNorm())) + (iOdata.state.Pose.getRotation().plus(objectRelative.getRotation()).getCos() * 0.1524 * leftRight),
            iOdata.state.Pose.getRotation().plus(objectRelative.getRotation())
        );
        heading = objectAbsolute.getRotation();
        pathGoalPose = objectAbsolute;
        AutoBuilder.pathfindToPose(objectAbsolute, constraints).schedule();
    }

    public void alignReef(int leftRight) {
        switch ((int) (60*Math.round(Math.toDegrees(Math.atan2(REEF_CENTER.getY() - iOdata.state.Pose.getY(), (DriverStation.getAlliance().get() == Alliance.Blue ? REEF_CENTER.getX(): REEF_CENTER.getX() + OFFSET_TO_RED) - iOdata.state.Pose.getX()))/60))) {
            case 0:
                reefTarget = SIDE_0;
                heading = Rotation2d.fromDegrees(0);
                break;
            case 60:
                reefTarget = SIDE_60;
                heading = Rotation2d.fromDegrees(60);
                break;
            case 120:
                reefTarget = SIDE_120;
                heading = Rotation2d.fromDegrees(120);
                break;
            case -180:
                reefTarget = SIDE_180;
                heading = Rotation2d.fromDegrees(180);
                break;
            case 180:
                reefTarget = SIDE_180;
                heading = Rotation2d.fromDegrees(180);
                break;
            case -120:
                reefTarget = SIDE_240;
                heading = Rotation2d.fromDegrees(-120);
                break;
            case -60:
                reefTarget = SIDE_300;
                heading = Rotation2d.fromDegrees(-60);
                break;
        }
        reefTarget = new Pose2d(
            (DriverStation.getAlliance().get() == Alliance.Blue ? reefTarget.getX(): reefTarget.getX() + OFFSET_TO_RED) - (reefTarget.getRotation().getSin() * 0.1524 * leftRight),
            reefTarget.getY() + (reefTarget.getRotation().getCos() * 0.1524 * leftRight),
            reefTarget.getRotation()
        );
        pathGoalPose = reefTarget;
        AutoBuilder.pathfindToPose(reefTarget, constraints).schedule();
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
              );
         heading = this.iOdata.state.Pose.getRotation();
     }
 
 
     public void headingControl(double driveX, double driveY) {
         driveIO.setSwerveRequest(FIELD_CENTRIC
             .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
             .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
             .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), heading.getRadians()))
         );
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

     public void lockReef(double driveX, double driveY) {
        double degToReef = Math.toDegrees(Math.atan2(REEF_CENTER.getY() - iOdata.state.Pose.getY(), (DriverStation.getAlliance().get() == Alliance.Blue ? REEF_CENTER.getX(): REEF_CENTER.getX() + OFFSET_TO_RED) - iOdata.state.Pose.getX()));
        driveIO.setSwerveRequest(ROBOT_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY() * 0.25)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY() * 0.25)
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(),
            Math.toRadians(60*Math.round(degToReef/60))))
        );        
        heading = new Rotation2d (Math.toRadians(60*Math.round(degToReef/60)));
    }

    public void lockReefManual(double driveX, double driveY, double rightX, double rightY) {
        double joystickDeg = Math.toDegrees(Math.atan2(rightY, -rightX)) - 90;
        driveIO.setSwerveRequest(ROBOT_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY() * 0.25)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY() * 0.25)
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(),
            Math.toRadians(60*Math.round(joystickDeg/60))))
        );        
        heading = new Rotation2d (Math.toRadians(60*Math.round(joystickDeg/60)));
    }

    public void setSelectedAutoName(String name) {
        try {
            this.pathGroup.addAll(PathPlannerAuto.getPathGroupFromAutoFile("OTF_TESTING"));
        } catch (IOException e) {
            System.out.println("Auto String Invalid");
        } catch (ParseException e) {
            System.out.println("Auto Data Invalid");
        }
    }

    @Override
    public void periodic() {
        matchTime = DriverStation.getMatchTime();
        matchTimeEntry.setDouble(matchTime);
        volts = RoboRioDataJNI.getVInVoltage();
        voltsEntry.setDouble(volts);
        amps = RoboRioDataJNI.getVInCurrent();
        ampsEntry.setDouble(amps);

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
                this.iOdata.state.Pose.getRotation().getRadians()});
        }
        if (pathGoalPose != null) {
            pathGoalPoseEntry.setDoubleArray(new Double[]{
                pathGoalPose.getX(), 
                pathGoalPose.getY(), 
                pathGoalPose.getRotation().getRadians()});
        } 

        if (!DriverStation.isTeleop()) {
            heading = iOdata.state.Pose.getRotation();
        }

        if (pathGroup != null) {
            currentPathIndex = IntStream.range(0, pathGroup.size())
            .filter(i -> pathGroup.get(i).name.equals(PathPlannerAuto.currentPathName))
            .findFirst()
            .orElse(0);
        }

        alert.set(true);
    }

    public void addVisionMeasurement(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
        this.driveIO.updateVision(calculatedPose, timestamp, stDevs);
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
