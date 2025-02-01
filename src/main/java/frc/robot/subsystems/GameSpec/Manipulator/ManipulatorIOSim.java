package frc.robot.subsystems.GameSpec.Manipulator;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
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
import frc.robot.subsystems.GameSpec.Arm.ArmConstants;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOSim;

public class ManipulatorIOSim extends ManipulatorIO {
  
  TalonFXSimState coralSimState;
  TalonFXSSimState coralWristSimState;
  CANcoderSimState encoderSimState;

  SingleJointedArmSim coralWristSim;
  DCMotorSim coralSim;

  private static Mechanism2d mech = new Mechanism2d(1, 1);
  private static MechanismRoot2d root = mech.getRoot("Coral", .5, .5);
  private static MechanismLigament2d coralLig;

  public ManipulatorIOSim(){
    coralSimState = coral.getSimState();
    coralWristSimState = coralWrist.getSimState();
    encoderSimState = coralCancoder.getSimState();

    coralWristSim = new SingleJointedArmSim(
      DCMotor.getBag(1),
      10,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(1), 1),
      Units.inchesToMeters(1),
      Units.degreesToRadians(-180),
      Units.degreesToRadians(180),
      false,
      Units.degreesToRadians(0)
    );

    double[] ms = {0,0};

    coralSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 
      1, 10), 
      DCMotor.getKrakenX60(1),
      ms
      );

      coralLig =
      root.append(
          new MechanismLigament2d("Elevator", coralWristSim.getAngleRads(), 90, 10, new Color8Bit(235, 137, 52)));
      coralLig.setLength(.5);

      SmartDashboard.putData("Coral Sim", mech);
  }

  @Override
  public void periodic() {
      // In this method, we update our simulation of what our arm is doing
      // First, we set our "inputs" (voltages)
      coralWristSim.setInput(coralWrist.get() * RobotController.getBatteryVoltage());
      coralSim.setInput(coral.get() * RobotController.getBatteryVoltage());

      // Next, we update it. The standard loop time is 20ms.
      coralWristSim.update(0.020);
      coralSim.update(0.020);

      encoderSimState.setRawPosition(coralWristSim.getAngleRads()/6.28319);
      coralSimState.setRawRotorPosition(coralSim.getAngularPositionRotations());
      coralSimState.setRotorVelocity(coralSim.getAngularVelocity());
      // encoderSimState.setVelocity(armSim.getVelocityRadPerSec());

      coralLig.setAngle(coralWrist.getPosition().getValueAsDouble());
  }
}