package frc.robot.subsystems.GameSpec.CoralHand;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.GameSpec.Arm.Arm;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOSim;

public class CoralHandIOSim extends CoralHandIO {

    ElevatorIOSim elevatorSim = new ElevatorIOSim();

    public TalonFXSimState armSimState;

    public final SingleJointedArmSim armSim;

      // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  //private final MechanismLigament2d m_armTower;
  public MechanismLigament2d m_arm;


  public CoralHandIOSim(){
    armSimState = CoralHand.getSimState();

    armSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60Foc(1),
          200.0,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 8.0),
          Units.inchesToMeters(30),
          Units.degreesToRadians(-75),
          Units.degreesToRadians(255),
          true,
          0,
          2.0 * Math.PI / 4096.0,
          0.0 // Add noise with a std-dev of 1 tick
          );
        
       //m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    //    m_arm = elevatorSim.m_elevatorMech2d.append(
    //     new MechanismLigament2d(
    //         "Arm",
    //         30,
    //         Units.radiansToDegrees(armSim.getAngleRads()),
    //         6,
    //         new Color8Bit(Color.kYellow))); 

    //         SmartDashboard.putData("Arm Sim", m_mech2d);
  }

    @Override
    public void periodic() {

            // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(CoralHand.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    }
}