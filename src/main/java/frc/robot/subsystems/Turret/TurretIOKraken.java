package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOKraken  implements TurretIO {
  // Hardware
  private final TalonFX turretTalon;

  // Status Signals
  private final StatusSignal<Angle> Position;
  private final StatusSignal<AngularVelocity> Velocity;
  private final StatusSignal<Voltage> AppliedVolts;
  private final StatusSignal<Current> SupplyCurrent;
  private final StatusSignal<Current> TorqueCurrent;
  private final StatusSignal<Temperature> TempCelsius;

  // Control
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  //private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public TurretIOKraken() {
    turretTalon = new TalonFX(TurretConstants.turretMotorID, "rio");
      

    // General config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    //*Cory Added Does not apply to Torque control on Voltage/Velocity Control*/
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true; 
    //*End of Cory Added  */
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = TurretConstants.kReduction;

    // Controller config;
    controllerConfig.kP = TurretConstants.turretGains.kP();
    controllerConfig.kI = TurretConstants.turretGains.kI();
    controllerConfig.kD = TurretConstants.turretGains.kD();
    controllerConfig.kS = TurretConstants.turretGains.kS();
    controllerConfig.kV = TurretConstants.turretGains.kV();
    controllerConfig.kA = TurretConstants.turretGains.kA();

    // Apply configs
    turretTalon.getConfigurator().apply(config, 1.0);
    turretTalon.getConfigurator().apply(controllerConfig, 1.0);

    // Set inverts
    //turretTalon.setInverted(true);

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
    stats.VelocityRadPerSec = Velocity.getValueAsDouble();
    stats.AppliedVolts = AppliedVolts.getValueAsDouble();
    stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
    stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
    stats.TempCelsius = TempCelsius.getValueAsDouble();

  }

  @Override
  public void runVolts(double Volts) {
    turretTalon.setControl(voltageControl.withOutput(Volts));
  }

  @Override
  public void stop() {
    turretTalon.setControl(neutralControl);
  }

  @Override
  public void runVelocity(double Rpm, double Feedforward) {
    turretTalon.setControl(
        velocityControl.withVelocity(Rpm / 60.0));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.kP = kP;
    controllerConfig.kI = kI;
    controllerConfig.kD = kD;
    turretTalon.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void runCharacterization(double input) {
    turretTalon.setControl(voltageControl.withOutput(input));
  }

}