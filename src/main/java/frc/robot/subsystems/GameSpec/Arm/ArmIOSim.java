package frc.robot.subsystems.GameSpec.Arm;

import com.ctre.phoenix6.sim.ChassisReference;
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
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOSim;

public class ArmIOSim extends ArmIO {
  public TalonFXSimState armSimState;

  public final SingleJointedArmSim armSim;

  public MechanismLigament2d m_arm;

  public ArmIOSim(){
    armSimState = arm.getSimState();

    armSim = new SingleJointedArmSim(
        ArmConstants.simGearBox,
        ArmConstants.simGearing,
        ArmConstants.simInertia,
        ArmConstants.simArmLength,
        ArmConstants.simMinAngle,
        ArmConstants.simMaxAngle,
        ArmConstants.gravity,
        ArmConstants.startingAngle,
        ArmConstants.measurementSTDDEVS,
          0.0 // Add noise with a std-dev of 1 tick
          );
        
       //m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
       m_arm = ElevatorIOSim.getElevatorLigament().append(
        new MechanismLigament2d(
            "Arm",
            ArmConstants.simArmLength, // Length of the arm
            Units.radiansToDegrees(armSim.getAngleRads()), // Initial angle
            50, // Line width
            new Color8Bit(Color.kYellow)));
  }

    @Override
    public void periodic() {
      // In this method, we update our simulation of what our arm is doing
      // First, we set our "inputs" (voltages)
      armSim.setInput(arm.get() * RobotController.getBatteryVoltage());

      // Next, we update it. The standard loop time is 20ms.
      armSim.update(0.020);

      // SimBattery estimates loaded battery voltages
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

      armSimState.setRawRotorPosition(armSim.getAngleRads());
      armSimState.setRotorVelocity(armSim.getVelocityRadPerSec());

      // Update the Mechanism Arm angle based on the simulated arm angle
      m_arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }
}