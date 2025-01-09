package frc.robot.subsystems.Elevator;
import javax.swing.text.Position;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;


public class ElevatorIOKraken extends ElevatorIO {

    MotionMagicVoltage elevatorMagic;

    private final StatusSignal<Boolean> f_fusedSensorOutOfSync;
    private final StatusSignal<Boolean> sf_fusedSensorOutOfSync;
    private final StatusSignal<Boolean> f_remoteSensorInvalid;
    private final StatusSignal<Boolean> sf_remoteSensorInvalid;

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Angle> elevatorRotorPos;
    private final StatusSignal<Angle> cancoderPosition;
    private final StatusSignal<AngularVelocity> cancoderVelocity;

    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;
    private final StatusSignal<Temperature> TempCelsius;

    public ElevatorIOKraken() {

    super(ElevatorConstants.elevatorMotorID, ElevatorConstants.elevatorEncoderID, "CamBot"); // Pass ID and CAN bus to abstract class
      
    elevatorMagic = new MotionMagicVoltage(0);
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = ElevatorConstants.elevatorGains.CruiseVelocity(); //rps
    mm.MotionMagicAcceleration = ElevatorConstants.elevatorGains.Acceleration();
    mm.MotionMagicJerk = ElevatorConstants.elevatorGains.Jerk();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = ElevatorConstants.elevatorGains.kP();
    slot0.kI = ElevatorConstants.elevatorGains.kI();
    slot0.kD = ElevatorConstants.elevatorGains.kD();
    slot0.kV = ElevatorConstants.elevatorGains.kV();
    slot0.kS = ElevatorConstants.elevatorGains.kS(); // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 1;

    CANcoderConfiguration cancoderConfigElevator = new CANcoderConfiguration();
    cancoderConfigElevator.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfigElevator.MagnetSensor.MagnetOffset = 0.2478;
    elevatorEncoder.getConfigurator().apply(cancoderConfigElevator);

    cfg.Feedback.FeedbackRemoteSensorID = elevatorEncoder.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.SensorToMechanismRatio = 1; //changes what the cancoder and fx encoder ratio is
    cfg.Feedback.RotorToSensorRatio = 1; //12.8;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;


    StatusCode elevatorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      elevatorStatus = elevator.getConfigurator().apply(cfg);
      if (elevatorStatus.isOK()) break;
    }
    if (!elevatorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + elevatorStatus.toString());
    }

    f_fusedSensorOutOfSync = elevator.getFault_FusedSensorOutOfSync();
    sf_fusedSensorOutOfSync = elevator.getStickyFault_FusedSensorOutOfSync();
    f_remoteSensorInvalid = elevator.getFault_RemoteSensorDataInvalid();
    sf_remoteSensorInvalid = elevator.getStickyFault_RemoteSensorDataInvalid();

    elevatorPosition = elevator.getPosition();
    elevatorVelocity = elevator.getVelocity();
    elevatorRotorPos = elevator.getRotorPosition();
    cancoderPosition = elevatorEncoder.getPosition();
    cancoderVelocity = elevatorEncoder.getVelocity();

    SupplyCurrent = elevator.getSupplyCurrent();
    TorqueCurrent = elevator.getTorqueCurrent();
    TempCelsius = elevator.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        f_fusedSensorOutOfSync,
        sf_fusedSensorOutOfSync,
        f_remoteSensorInvalid,
        sf_remoteSensorInvalid,
        elevatorPosition,
        elevatorVelocity,
        cancoderPosition,
        cancoderVelocity,
        elevatorRotorPos,
        SupplyCurrent,
        TorqueCurrent,
        TempCelsius
      );
    }

    @Override
    public void updateStats(ElevatorIOStats stats) {
        stats.elevatorMotorConnected =
        BaseStatusSignal.refreshAll(
          f_fusedSensorOutOfSync,
          sf_fusedSensorOutOfSync,
          f_remoteSensorInvalid,
          sf_remoteSensorInvalid,
          elevatorPosition,
          elevatorVelocity,
          cancoderPosition,
          cancoderVelocity,
          elevatorRotorPos,
          SupplyCurrent,
          TorqueCurrent,
          TempCelsius)
            .isOK();

    stats.cancoderPosition = cancoderPosition.getValueAsDouble();
    stats.cancoderVelocity = cancoderVelocity.getValueAsDouble();
    stats.elevatorPosition = elevatorPosition.getValueAsDouble();
    stats.elevatorVelocity = elevatorVelocity.getValueAsDouble();

    stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
    stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
    stats.TempCelsius = TempCelsius.getValueAsDouble();
    }

    @Override
    public void setElevatorMotorControl(double commandedPosition) {
      elevator.setControl(elevatorMagic.withPosition(commandedPosition).withSlot(0));
    }
}
