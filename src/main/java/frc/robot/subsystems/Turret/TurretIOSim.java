package frc.robot.subsystems.Turret;


import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.annotation.JsonTypeInfo.As;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class TurretIOSim implements TurretIO {
  private static final double moi = 1.2;
  private static final double kGearRatio = 10.0;
  private double appliedVolts = 0.0; 
  
  // Hardware
  private final TalonFX turretTalon = new TalonFX(TurretConstants.turretMotorID, "rio");
  private final CANcoder turretEncoder = new CANcoder(TurretConstants.turretCanCoderID);

  //Simulation States 
  private final TalonFXSimState talonSim = turretTalon.getSimState(); 
  private final CANcoderSimState encoderSim = turretEncoder.getSimState();

  //Define Simulation Model
  private DCMotorSim turretSim = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio ), 
      DCMotor.getKrakenX60Foc(1));
    
  //Define PID Controller     
  private final PIDController turretController =
      new PIDController(TurretConstants.turretGains.kP(), TurretConstants.turretGains.kI(), TurretConstants.turretGains.kD());
 
  // Status Signals
  private final StatusSignal<Angle> Position;
  private final StatusSignal<AngularVelocity> Velocity;
  private final StatusSignal<Voltage> AppliedVolts;
  private final StatusSignal<Current> SupplyCurrent;
  private final StatusSignal<Current> TorqueCurrent;
  private final StatusSignal<Temperature> TempCelsius;

  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  //private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);

  //Constructor  
  public TurretIOSim() {
  // Set signals
    Position = turretTalon.getPosition();
    Velocity = turretTalon.getVelocity();
    AppliedVolts = turretTalon.getMotorVoltage();
    SupplyCurrent = turretTalon.getSupplyCurrent();
    TorqueCurrent = turretTalon.getTorqueCurrent();
    TempCelsius = turretTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        Position,
        Velocity,
        AppliedVolts,
        SupplyCurrent,
        TorqueCurrent,
        TempCelsius);
  }

  public void simulationPeriodic() {
    //set the supply voltage of the TalonFX 
    talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = talonSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    turretSim.setInputVoltage(motorVoltage.in(Volts));
    //age sim model 20ms 
    turretSim.update(0.020); // assume 20 ms loop time
    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
   
    talonSim.setRawRotorPosition(turretSim.getAngularPosition().times(kGearRatio));
    talonSim.setRotorVelocity(turretSim.getAngularVelocity().times(kGearRatio));
  }

    @Override
    public void updateStats(TurretIOStats stats) {
      stats.MotorConnected =
          BaseStatusSignal.refreshAll(
                  Position,
                  Velocity,
                  AppliedVolts,
                  SupplyCurrent,
                  TorqueCurrent,
                  TempCelsius)
              .isOK();
  
      stats.PositionRads = Units.rotationsToRadians(Position.getValueAsDouble());
      stats.VelocityRadPerSec = Velocity.getValueAsDouble() * 60.0;
      stats.AppliedVolts = AppliedVolts.getValueAsDouble();
      stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
      stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
      stats.TempCelsius = TempCelsius.getValueAsDouble();

      //Not Using CTRE just simulation ......
      //turretSim.setInputVoltage(appliedVolts);
      //turretSim.update(0.02);
      //stats.PositionRads = turretSim.getAngularPositionRad();
      //stats.VelocityRadPerSec = turretSim.getAngularVelocityRadPerSec();
      //stats.AppliedVolts = appliedVolts;
      //stats.TorqueCurrentAmps = turretSim.getCurrentDrawAmps();
      //stats.SuppyCurrentAmps = RobotController.getBatteryVoltage();
      //stats.TempCelsius = 0.0; // no value in simulation 
    }



    @Override
    public void runVolts(double Voltage) {
      appliedVolts = MathUtil.clamp(Voltage,-12.0, 12.0);
      turretTalon.setVoltage(appliedVolts);
    }

    @Override
    public void runVelocity(double Rpm, double Feedforward) {
      turretTalon.setControl(velocityControl.withVelocity(Rpm / 60.0));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
      turretController.setPID(kP, kI, kD);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void runCharacterization(double input) {
       
    }
}