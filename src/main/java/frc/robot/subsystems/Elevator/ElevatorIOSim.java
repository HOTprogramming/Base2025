package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorIOSim implements ElevatorIO {

    public TalonFX elevatorMotor;
    public CANcoder elevatorEncoder;

    public TalonFXSimState elevatorMotorSimState;
    public CANcoderSimState elevatorEncoderSimState;

    public ElevatorSim elevatorSim;

    public DCMotorSim motorSim;

    double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * Units.inchesToMeters(2.0) / 4096;

    private final Mechanism2d m_mech2d = new Mechanism2d(5, 5);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 2.5, 0);

    private final MechanismLigament2d m_elevatorMech2d;

    public ElevatorIOSim(){


        elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID, "CamBot");
        elevatorEncoder = new CANcoder(ElevatorConstants.elevatorEncoderID, "CamBot");
        
        elevatorMotorSimState = elevatorMotor.getSimState();
        elevatorEncoderSimState = elevatorEncoder.getSimState();

        motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                100,
                100),
            DCMotor.getKrakenX60Foc(1));

        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60Foc(1),
            10, 
            4, 
            Units.inchesToMeters(2.0),
            0,
            2,
            true,
            0);

        m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90, 100, new Color8Bit(235, 137, 52)));

        SmartDashboard.putData("Elevator Sim", m_mech2d);

    }

    public void setElevatorMotorControl(double power){
        elevatorMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        elevatorEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorMotor.set(power);

        ioStats.TempCelsius = elevatorMotorSimState.getMotorVoltage();
        
        motorSim.setInputVoltage(elevatorMotorSimState.getMotorVoltage());
        motorSim.update(.02);

        elevatorMotorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations());
        elevatorMotorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM());

        // ioStats.elevatorPosition = motorSim.getAngularPositionRotations();
        // ioStats.elevatorVelocity = motorSim.getAngularVelocityRPM();

        elevatorSim.setInputVoltage(elevatorMotorSimState.getMotorVoltage());

        elevatorSim.update(.02);

        elevatorMotorSimState.setRawRotorPosition(elevatorSim.getPositionMeters());
        elevatorMotorSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond());

        ioStats.elevatorPosition = elevatorSim.getPositionMeters();
        ioStats.elevatorVelocity = elevatorSim.getVelocityMetersPerSecond();

        m_elevatorMech2d.setLength(elevatorSim.getPositionMeters());
    }
}