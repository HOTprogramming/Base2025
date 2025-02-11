package frc.robot.subsystems.Camera;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Camera.Camera.CameraPositions;

public class CameraConstants {
    public class CameraConstant{

        public CameraConstant(String name, 
                              Translation3d relativeTranslation, 
                              Rotation3d relativeRotation, 
                              Matrix<N3, N1> singleTagStdDevs,
                              Matrix<N3, N1> multiTagStdDevs) {
            this.name = name;
            this.transform = new Transform3d(relativeTranslation, relativeRotation);
            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevs;
        }

        private Matrix<N3, N1> singleTagStdDevs;
        public Matrix<N3, N1> getSingleTagStdDevs() {
            return singleTagStdDevs;
        }

        private Matrix<N3, N1> multiTagStdDevs;
        public Matrix<N3, N1> getMultiTagStdDevs() {
            return multiTagStdDevs;
        }

        private String name;
        public String getName() {
            return name;
        }
        private Transform3d transform;
        public Transform3d getTransform() {
            return transform;
        }
    }

    //Intake (Front): 12.483in forward of robot middle. On center. 8.625in above ground Perpendicular to floor
    //Shooting (Back): 12.425in reward of robot middle. On center. 6.008in above ground. Angled 8 degrees up from floor
    //Left Side: 10.696in left of robot middle. 30 degrees left of rear. 16.838in above ground. angled 5 degrees up from floor
    //Right Side: 10.696in right of robot middle. 30 degrees right of rear. 16.838in above ground. angled 5 degrees up from floor
    //(X, Y, Z)
    //X: Front and back (Front +)
    //Y: Left and right (Left +)
    //Z: Vertical distance from the floor to the camera (Up +)

    public double[] STDEV_GAIN = new double[] {.7, .7, .5};
    public double MAX_DISTANCE = 5.5;
    
    public Map<CameraPositions, CameraConstant> cameraConstants = null;
    
    public CameraConstants(){
        cameraConstants = new EnumMap<>(CameraPositions.class);         
        switch (Constants.getRobot()) {
            case DEVBOT:
                cameraConstants.put(CameraPositions.LEFT, new CameraConstant("LEFT",
                        new Translation3d(Units.inchesToMeters(10.66), Units.inchesToMeters(11.516), Units.inchesToMeters(7.752)), // without wheel squish
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(20.0), Units.degreesToRadians(-55.0)),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                
                cameraConstants.put(CameraPositions.RIGHT, new CameraConstant("RIGHT",
                        new Translation3d(Units.inchesToMeters(10.66), Units.inchesToMeters(-11.516), Units.inchesToMeters(7.752)),
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(20.0), Units.degreesToRadians(55.0)),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                cameraConstants.put(CameraPositions.BACK, new CameraConstant("TOP", // top
                        new Translation3d(Units.inchesToMeters(8.75), Units.inchesToMeters(7.829), Units.inchesToMeters(41.863)),
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(67.48), Units.degreesToRadians(23.23)),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));
                break;
            case COMPBOT:
                break;
            case SIMBOT:
                cameraConstants.put(CameraPositions.FRONT, new CameraConstant("a",
                        new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
                        new Rotation3d(0, 0, 0),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));

                
                cameraConstants.put(CameraPositions.BACK, new CameraConstant("b",
                        new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
                        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180)),
                        VecBuilder.fill(4, 4, 8),
                        VecBuilder.fill(0.5, 0.5, 1)));
                break;
        }
        

    }
}
