package frc.robot.subsystems.Turret;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOKraken implements TurretIO {
  public static final double reduction = (74.0 / 16.0) / (72.0 / 18.0) * 125.0;

  // Hardware
  private final TalonFX krakenMotor;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temp;

  // Control Requests
  private final VoltageOut voltsRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final CoastOut coastRequest = new CoastOut();

  public TurretIOKraken() {
    krakenMotor = new TalonFX(TurretConstants.turretLeftMotorID);

    var config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = reduction;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
        config.CurrentLimits.StatorCurrentLimit = 120.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    StatusCode turretStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            turretStatus = krakenMotor.getConfigurator().apply(config);
        }
        if (!turretStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + turretStatus.toString());
        }

    /* Configure the Base Status Signal Data */
    position = krakenMotor.getPosition();
    velocity = krakenMotor.getVelocity();
    appliedVolts = krakenMotor.getMotorVoltage();
    supplyCurrentAmps = krakenMotor.getSupplyCurrent();
    torqueCurrentAmps = krakenMotor.getTorqueCurrent();
    temp = krakenMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrentAmps, torqueCurrentAmps, temp);

    krakenMotor.optimizeBusUtilization();
  }

  @Override
  public void updateStatusSignals(TurretIOInputs inputs) {
    inputs.data =
        new TurretIOData(
            BaseStatusSignal.isAllGood(position, velocity, appliedVolts, supplyCurrentAmps, temp),
            position.getValue().in(Radians),
            velocity.getValue().in(RadiansPerSecond),
            appliedVolts.getValueAsDouble(),
            torqueCurrentAmps.getValueAsDouble(),
            supplyCurrentAmps.getValueAsDouble(),
            temp.getValueAsDouble());
  }

  @Override
  public void runVolts(double volts) {
    krakenMotor.setControl(voltsRequest.withOutput(volts));
  }

  @Override
  public void runTorqueCurrent(double current) {
    krakenMotor.setControl(torqueCurrentRequest.withOutput(current));
  }

  @Override
  public void stop() {
    krakenMotor.stopMotor();
  }

  @Override
  public void coast() {
    krakenMotor.setControl(coastRequest);
  }
}