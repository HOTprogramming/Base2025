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
            cameraConstants.put(CameraPositions.FRONT, new CameraConstant("front_camera",
            new Translation3d(Units.inchesToMeters(12.483), Units.inchesToMeters(0), Units.inchesToMeters(8.625)),
            new Rotation3d(0, 0, 0),
            VecBuilder.fill(4, 4, 8),
            VecBuilder.fill(0.5, 0.5, 1)));

        
        cameraConstants.put(CameraPositions.BACK, new CameraConstant("back_camera",
                new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(0), Units.inchesToMeters(6.193)),
                new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)),
                VecBuilder.fill(4, 4, 8),
                VecBuilder.fill(0.5, 0.5, 1)));

        cameraConstants.put(CameraPositions.LEFT,  new CameraConstant("left_camera",
                                                    new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(11.49), Units.inchesToMeters(16.74)),
                                                    new Rotation3d(Units.degreesToRadians(-5.77), Units.degreesToRadians(-9.92), Units.degreesToRadians(120.38)),
                                                    VecBuilder.fill(4, 4, 8),
                                                    VecBuilder.fill(0.5, 0.5, 1)));

        cameraConstants.put(CameraPositions.RIGHT, new CameraConstant("right_camera",
                                                    new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(-11.49), Units.inchesToMeters(16.74)),
                                                    new Rotation3d(Units.degreesToRadians(5.77), Units.degreesToRadians(-9.92), Units.degreesToRadians(-120.38)),
                                                    VecBuilder.fill(4, 4, 8),
                                                    VecBuilder.fill(0.5, 0.5, 1)));
    }
}
