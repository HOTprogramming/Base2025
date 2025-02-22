package frc.robot.subsystems.Camera;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon.ProtobufPhotonTrackedTarget;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import java.beans.VetoableChangeListener;
import java.io.IOException;
import java.time.Month;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import javax.xml.crypto.Data;

import frc.robot.Robot;
import frc.robot.subsystems.Camera.CameraConstants.CameraConstant;
import frc.robot.subsystems.Drivetrain.Drive;
public class Camera extends SubsystemBase {

    public enum CameraPositions {
        FRONT,
        TOP,
        LEFT,
        RIGHT
    }

    // front
    PhotonCamera frontCamera;
    PhotonPipelineResult result;
    PhotonTrackedTarget frontBestTarget;
    double frontSampleTime;
    Pose2d frontCamEstimation;
    Matrix<N3, N1> frontDevs = VecBuilder.fill(1, 1, 1);

    // left
    PhotonCamera leftCamera;
    PhotonPipelineResult leftResult;
    PhotonTrackedTarget leftBestTarget;
    double leftSampleTime;
    Pose2d leftCamEstimation;
    Matrix<N3, N1> leftDevs = VecBuilder.fill(1, 1, 1);

    // right
    PhotonCamera rightCamera;
    PhotonPipelineResult rightResult;
    PhotonTrackedTarget rightBestTarget;
    double rightSampleTime;
    Pose2d rightCamEstimation;
    Matrix<N3, N1> rightDevs = VecBuilder.fill(1, 1, 1);

    // rear
    PhotonCamera rearCamera;
    PhotonPipelineResult rearResult;
    PhotonTrackedTarget rearBestTarget;
    double rearSampleTime;
    Pose2d rearCamEstimation;
    Matrix<N3, N1> rearDevs = VecBuilder.fill(1, 1, 1);

    
    AprilTagFieldLayout tags;

    boolean drawWireframes = false; // resource heavy

    Nat<N3> rows = new Nat<N3>() {

        @Override
        public int getNum() {
            return 3;
        }
        
    };
    Nat<N4> colls = new Nat<N4>() {

        @Override
        public int getNum() {
            return 4;
        }
        
    };

    Matrix<N3, N4> stDevs = new Matrix<N3, N4>(rows, colls);
    double[] timestamps = new double[] {-1, -1, -1 ,-1};
    Pose2d[] poses = new Pose2d[] {
        new Pose2d(),
        new Pose2d(),
        new Pose2d(),
        new Pose2d()
    };



    
    // NetworkTables
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("Vision");
    StringPublisher fieldTypePublisher = table.getStringTopic(".type").publish();

    DoubleArrayPublisher frontCameraPub;
    DoubleArrayPublisher leftCameraPub;
    DoubleArrayPublisher rightCameraPub;
    DoubleArrayPublisher rearCameraPub;

    Field2d fieldRight = new Field2d();
    Field2d fieldLeft = new Field2d();

    Map<CameraPositions, Optional<EstimatedRobotPose>> cameraMeasurements = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, Matrix<N3, N1>> cameraStdDeviations = new EnumMap<>(CameraPositions.class);

    Map<CameraPositions, PhotonCamera> cameras = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, DoubleArrayPublisher> publishers = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, PhotonPoseEstimator> photonPoseEstimators = new EnumMap<>(CameraPositions.class); 
    Map<CameraPositions, Double> lastEstTimestamps = new EnumMap<>(CameraPositions.class); 
    private Map<CameraPositions, List<PhotonTrackedTarget>> targetsSeenByCamera  = new EnumMap<>(CameraPositions.class);
    CameraPositions allowMultiTag;

    private int minimumTagsSeenByAnyCamera;
    private int lastMinimumTagsSeenByAnyCamera = 0;

    private int loopsPast;

    private CameraConstants constants;

    // docs https://docs.photonvision.org/ 
    private Drive drivetrain;

    private boolean ignoring = false;

    public Camera(Drive drive) {
        SmartDashboard.putData("Left", fieldLeft);
        SmartDashboard.putData("Right", fieldRight);

        drivetrain = drive;
        constants = new CameraConstants();
        
        try {
            tags = AprilTagFieldLayout.loadFromResource(("/org/Apriltags/2025-reefscape.json"));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        CameraConstant cameraConstant = constants.cameraConstants.get(CameraPositions.FRONT);
        if (cameraConstant != null) {
                cameras.put(CameraPositions.FRONT, new PhotonCamera(cameraConstant.getName()));
                publishers.put(CameraPositions.FRONT, table.getDoubleArrayTopic("Front_Camera").publish());
                PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraConstant.getTransform());
                estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                photonPoseEstimators.put(CameraPositions.FRONT, estimator);
                lastEstTimestamps.put(CameraPositions.FRONT, -1.0);
        }

        cameraConstant = constants.cameraConstants.get(CameraPositions.LEFT);
        if (cameraConstant != null) {
            cameras.put(CameraPositions.LEFT, new PhotonCamera(cameraConstant.getName()));
            publishers.put(CameraPositions.LEFT, table.getDoubleArrayTopic("Left_Camera").publish());
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraConstant.getTransform());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            photonPoseEstimators.put(CameraPositions.LEFT, estimator);
                lastEstTimestamps.put(CameraPositions.LEFT, -1.0);
        }
        
        cameraConstant = constants.cameraConstants.get(CameraPositions.RIGHT);
        if (cameraConstant != null) {
            cameras.put(CameraPositions.RIGHT, new PhotonCamera(cameraConstant.getName()));
            publishers.put(CameraPositions.RIGHT, table.getDoubleArrayTopic("Right_Camera").publish());
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraConstant.getTransform());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            photonPoseEstimators.put(CameraPositions.RIGHT, estimator);
                lastEstTimestamps.put(CameraPositions.RIGHT, -1.0);
        }
        
        cameraConstant = constants.cameraConstants.get(CameraPositions.TOP);
        if (cameraConstant != null) {
            cameras.put(CameraPositions.TOP, new PhotonCamera(cameraConstant.getName()));
            publishers.put(CameraPositions.TOP, table.getDoubleArrayTopic("Top_Camera").publish());
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraConstant.getTransform());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                lastEstTimestamps.put(CameraPositions.TOP, -1.0);
            photonPoseEstimators.put(CameraPositions.TOP, estimator);
        }
    }

    public Command setIgnore() {
        return runOnce(() -> ignoring = true);
    }

    public Command setUnIgnore() {
        return runOnce(() -> ignoring = false);
    }

    private Optional<EstimatedRobotPose> updateCameraMeasurment(CameraPositions key, CameraConstant constant, PhotonCamera camera, DoubleArrayPublisher publisher, PhotonPoseEstimator estimator, double lastEstTimestamp) {
        var visionEst = estimator.update(camera.getLatestResult());
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

        targetsSeenByCamera.put(key,camera.getLatestResult().targets);

        if (newResult) {
            lastEstTimestamp = latestTimestamp;
        } else {
            publisher.set(null);
        }
            visionEst.ifPresent(est -> { 
                publisher.set(new double[] {est.estimatedPose.toPose2d().getX(),
                est.estimatedPose.toPose2d().getY(),
                est.estimatedPose.toPose2d().getRotation().getDegrees()});
            });
            return visionEst;
     }

     @Override
    public void periodic() {

        drivetrain.seesReefTag = false;

        minimumTagsSeenByAnyCamera = 0;

        constants.cameraConstants.forEach((key,constant) -> {
            if (constant != null) {
                cameraMeasurements.put(key,updateCameraMeasurment(key, constant, cameras.get(key), publishers.get(key), photonPoseEstimators.get(key), lastEstTimestamps.get(key)));
            }
        });

        targetsSeenByCamera.forEach((key, list) -> {
            if (list.size() > minimumTagsSeenByAnyCamera) {
                minimumTagsSeenByAnyCamera = list.size();
            }
            SmartDashboard.putNumber("Camera" + key + "sees:", list.size());
        });

        allowMultiTag = CameraPositions.TOP;

        if (targetsSeenByCamera.containsKey(CameraPositions.TOP) && targetsSeenByCamera.get(CameraPositions.TOP).size() >=2 ) {
            allowMultiTag = CameraPositions.TOP;
        } else if (targetsSeenByCamera.containsKey(CameraPositions.RIGHT) && targetsSeenByCamera.get(CameraPositions.RIGHT).size() >=2 ) {
            allowMultiTag = CameraPositions.RIGHT;
        } else if (targetsSeenByCamera.containsKey(CameraPositions.LEFT) && targetsSeenByCamera.get(CameraPositions.LEFT).size() >=2 ) {
            allowMultiTag = CameraPositions.LEFT;
        } else if (targetsSeenByCamera.containsKey(CameraPositions.FRONT) && targetsSeenByCamera.get(CameraPositions.FRONT).size() >=2 ) {
            allowMultiTag = CameraPositions.FRONT;
        } 

        cameraMeasurements.forEach((key,measurement) -> {
            if (measurement.isPresent()) {
                cameraStdDeviations.put(key,getEstimationStdDevs(measurement.get().estimatedPose.toPose2d(), cameras.get(key), constants.cameraConstants.get(key), photonPoseEstimators.get(key),allowMultiTag == key));
            }
        });

        // robotState.putTargetsSeenByCamera(targetsSeenByCamera);

        cameraMeasurements.forEach((key,measurement) -> {
            // ignore top camera when aligning to reef
            if (!(key == CameraPositions.TOP && ignoring)) {
                measurement.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = cameraStdDeviations.get(key);

                        drivetrain.addVisionMeasurement(
                                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        
                        // SmartDashboard.putNumber(key + " X", est.estimatedPose.toPose2d().getX());
                        // SmartDashboard.putNumber(key + " Y", est.estimatedPose.toPose2d().getY());
                        // SmartDashboard.putNumber(key + " Rotation", est.estimatedPose.toPose2d().getRotation().getDegrees());

                        if(key == CameraPositions.RIGHT){
                            fieldRight.setRobotPose(est.estimatedPose.toPose2d()); 
                        } else {
                            fieldLeft.setRobotPose(est.estimatedPose.toPose2d()); 
                        }

                    });
            }
            });
        // robotState.setVisionMeasurements(cameraMeasurements);
        // robotState.setCameraStdDeviations(cameraStdDeviations);
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonCamera camera, CameraConstant constant, PhotonPoseEstimator estimator, boolean allowMultiTag) {
        var estStdDevs = constant.getSingleTagStdDevs();
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (allowMultiTag && numTags > 1) estStdDevs = constant.getMultiTagStdDevs();;
        // Increase std devs based on (average) distance
        if (numTags > 1 && avgDist > 6.5) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        for (PhotonTrackedTarget target : targets) {
            for (int id : CameraConstants.REEF_TAGS_RED) {
                if (target.fiducialId == id && avgDist < 2) {
                    estStdDevs = VecBuilder.fill(0.25, 0.25, 2);
                    drivetrain.tagTransform = new Pose3d()
                            .plus(target.getBestCameraToTarget())
                            .plus(constant.getTransform().inverse());
                            
                    drivetrain.reefTagID = target.fiducialId;
                    drivetrain.seesReefTag = true;

                    // SmartDashboard.put
                    // SmartDashboard.putNumber("Tag X", drivetrain.tagTransform.getX());
                    // SmartDashboard.putNumber("Tag Y", drivetrain.tagTransform.getY());
                    // SmartDashboard.putNumber("Tag Rotation", Units.radiansToDegrees(drivetrain.tagTransform.getRotation().getAngle()));
                }
            }
            for (int id : CameraConstants.IGNORE_ALWAYS) {
                if (target.fiducialId == id) {
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

                }
            }
        }

        if (numTags == 1 && avgDist > 2.75) {
            estStdDevs =VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        return estStdDevs;
    }    
}