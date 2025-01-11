package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorIOSim extends ElevatorIO {
    public TalonFXSimState elevatorMotorSimState;

    public ElevatorSim elevatorSim;

    // Additional simulation variables
    private final Mechanism2d m_mech2d = new Mechanism2d(5, 5);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 2.5, 0);
    private final MechanismLigament2d m_elevatorMech2d;

    public ElevatorIOSim() {        
        elevatorMotorSimState = elevator.getSimState();

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

    @Override
    public void periodic() {
        // NOTE: There is some conversion here that I am missing, so it will not work like this on the real robot, I will look into this
        elevatorMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInputVoltage(elevatorMotorSimState.getMotorVoltage());

        elevatorSim.update(.02);

        elevatorMotorSimState.setRawRotorPosition(elevatorSim.getPositionMeters());
        elevatorMotorSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond());

        m_elevatorMech2d.setLength(elevatorSim.getPositionMeters());
    }
}