package frc.robot.subsystems.GameSpec.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public abstract class ElevatorIO {

    // Protected TalonFX object accessible to subclasses
    protected TalonFX elevator;
    protected MotionMagicVoltage elevatorMagic;

    public static class ElevatorIOStats {
        public boolean elevatorMotorConnected = true;
        public double elevatorPosition = 0.0;
        public double elevatorVelocity = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double elevatorCurrentAmps = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;
    }

    protected static ElevatorIOStats stats = new ElevatorIOStats();

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;
    private final StatusSignal<Temperature> TempCelsius;

    /** Constructor to initialize the TalonFX */
    public ElevatorIO() {
        this.elevator = new TalonFX(ElevatorConstants.elevatorMotorID, "CamBot");

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
        slot0.kS = ElevatorConstants.elevatorGains.kS();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;

        StatusCode elevatorStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            elevatorStatus = elevator.getConfigurator().apply(cfg);
        if (elevatorStatus.isOK()) break;}
        if (!elevatorStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + elevatorStatus.toString());
        }

        elevatorPosition = elevator.getPosition();
        elevatorVelocity = elevator.getVelocity();
        SupplyCurrent = elevator.getSupplyCurrent();
        TorqueCurrent = elevator.getTorqueCurrent();
        TempCelsius = elevator.getDeviceTemp();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            elevatorPosition,
            elevatorVelocity,
            SupplyCurrent,
            TorqueCurrent,
            TempCelsius
          );
    }

    /** Update stats */
    public void updateStats() {
        stats.elevatorMotorConnected =
        BaseStatusSignal.refreshAll(
          elevatorPosition,
          elevatorVelocity,
          SupplyCurrent,
          TorqueCurrent,
          TempCelsius)
            .isOK();

        stats.elevatorPosition = elevatorPosition.getValueAsDouble();
        stats.elevatorVelocity = elevatorVelocity.getValueAsDouble();
        stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
        stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
        stats.TempCelsius = TempCelsius.getValueAsDouble();
    }


    /** Apply motion magic control mode */
    public void setElevatorMotorControl(double commandedPosition) {
        elevator.setControl(elevatorMagic.withPosition(commandedPosition).withSlot(0));
    }

    /** Stop motor */
    public void stop() {
        elevator.setVoltage(0);
    }

    /** Perform simulation-specific tasks */
    abstract public void periodic();
}