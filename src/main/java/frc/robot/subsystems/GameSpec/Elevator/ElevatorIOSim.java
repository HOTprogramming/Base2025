package frc.robot.subsystems.GameSpec.Elevator;

import javax.sound.midi.SysexMessage;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorIOSim extends ElevatorIO {
    public TalonFXSimState elevatorMotorSimState;
    public CANcoderSimState encoderSimState;

    public static ElevatorSim elevatorSim;

    // Additional simulation variables
    private static Mechanism2d m_mech2d = new Mechanism2d(5, 5);
    private static MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 2.5, 0);
    private static MechanismLigament2d m_elevatorMech2d;

    public ElevatorIOSim() {        
        encoderCfg.MagnetSensor.MagnetOffset = 0;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        encoderCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        applyConfig();

        elevatorMotorSimState = elevator.getSimState();
        encoderSimState = elevatorCancoder.getSimState();

        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            10, 
            4, 
            Units.inchesToMeters(2.0),
            0,
            3,
            true,
            0);

        m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90, 10, new Color8Bit(235, 137, 52)));

        SmartDashboard.putData("Elevator Sim", m_mech2d);
    }

    @Override
    public void periodic() {
        // NOTE: There is some conversion here that I am missing, so it will not work like this on the real robot, I will look into this
        elevatorMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        encoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInputVoltage(elevatorMotorSimState.getMotorVoltage());

        elevatorSim.update(.02);

        encoderSimState.setRawPosition(elevatorSim.getPositionMeters()/3.0226);
        // encoderSimState.setVelocity(elevatorSim.getVelocityMetersPerSecond());

        m_elevatorMech2d.setLength(elevatorSim.getPositionMeters());
    }

    public static MechanismObject2d getElevatorLigament() {
        return m_elevatorMech2d;
    }
}