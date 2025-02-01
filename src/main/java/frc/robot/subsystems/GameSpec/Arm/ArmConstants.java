package frc.robot.subsystems.GameSpec.Arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmConstants {
    
    public static final int armMotorID = 14;
    public static final int armEncoderID = 44;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains armGains = new MMGains(300, 200, 300, 0.2, 0.0, 0.0, 0, 0);
    public static final DCMotor simGearBox = DCMotor.getKrakenX60Foc(1);
    public static final double simGearing = 200;
    public static final double simInertia = SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 8.0);
    public static final double simArmLength = Units.inchesToMeters(60);
    public static final double simMinAngle = Units.degreesToRadians(-90);
    public static final double simMaxAngle = Units.degreesToRadians(90);
    public static final boolean gravity = false;
    public static final double startingAngle = Units.degreesToRadians(0);
    public static final double measurementSTDDEVS = 2.0 * Math.PI / 4096.0;

    public static final double PackageAngle = 0;

    public static final double FeederAngle = -45;
    public static final double L1Angle = 0;
    public static final double L2Angle = 0;
    public static final double L3Angle = -45;
    public static final double L4Angle = 45;

    
    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
}