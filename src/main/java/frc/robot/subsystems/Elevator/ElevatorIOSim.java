package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.Constants;


public class ElevatorIOSim extends ElevatorIO {

    public CANcoder elevatorEncoder;

    public TalonFXSimState elevatorMotorSimState;
    public CANcoderSimState elevatorEncoderSimState;

    public ElevatorSim elevatorSim;

    // Additional simulation variables
    private final Mechanism2d m_mech2d = new Mechanism2d(5, 5);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 2.5, 0);
    private final MechanismLigament2d m_elevatorMech2d;

    private final ProfiledPIDController m_controller =
            new ProfiledPIDController(
                15,
                0,
                0,
                new TrapezoidProfile.Constraints(2.45, 2.45));
        ElevatorFeedforward m_feedforward =
            new ElevatorFeedforward(
                0,
                .5,
                0,
                0);

    public ElevatorIOSim() {

        super(ElevatorConstants.elevatorMotorID, "SimulatedBus");
   
        elevatorEncoder = new CANcoder(ElevatorConstants.elevatorEncoderID, "CamBot");
        
        elevatorMotorSimState = elevatorMotor.getSimState();
        elevatorEncoderSimState = elevatorEncoder.getSimState();

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
    public void updateStats(ElevatorIOStats stats) {

    }

    @Override
    public void setElevatorMotorControl(double power) {
        elevatorMotor.set(power);
    }

    @Override
    public void reachGoal(double goal) {
        m_controller.setGoal(goal);

        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(elevatorSim.getPositionMeters());
        double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
        elevatorMotor.setVoltage(pidOutput + feedforwardOutput);
    }

    @Override
    public void simStuff() {
        elevatorMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        elevatorEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInputVoltage(elevatorMotorSimState.getMotorVoltage());

        elevatorSim.update(.02);

        elevatorMotorSimState.setRawRotorPosition(elevatorSim.getPositionMeters());
        elevatorMotorSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond());

        ioStats.elevatorPosition = elevatorSim.getPositionMeters();
        ioStats.elevatorVelocity = elevatorSim.getVelocityMetersPerSecond();

        m_elevatorMech2d.setLength(elevatorSim.getPositionMeters());
    }
}