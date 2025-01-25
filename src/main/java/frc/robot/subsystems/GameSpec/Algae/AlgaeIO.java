package frc.robot.subsystems.GameSpec.Algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public abstract class AlgaeIO {

    // Protected TalonFX object accessible to subclasses
    protected TalonFX algae;
    protected TalonFX algae2;
    protected MotionMagicVoltage algaeMagic;

    public static class AlgaeIOStats {
        public boolean algaeMotorConnected = true;
        public double algaePosition = 0.0;
        public double algaeVelocity = 0.0;
        public double algaeAppliedVolts = 0.0;
        public double algaeCurrentAmps = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;

        public boolean algaeMotorConnected2 = true;
        public double algaePosition2 = 0.0;
        public double algaeVelocity2 = 0.0;
        public double algaeAppliedVolts2 = 0.0;
        public double algaeCurrentAmps2 = 0.0;
        public double SupplyCurrentAmps2 = 0.0;
        public double TorqueCurrentAmps2 = 0.0;
        public double TempCelsius2 = 0.0;
    }

    protected static AlgaeIOStats stats = new AlgaeIOStats();

    private final StatusSignal<Angle> algaePosition;
    private final StatusSignal<AngularVelocity> algaeVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;
    private final StatusSignal<Temperature> TempCelsius;

    private final StatusSignal<Angle> algaePosition2;
    private final StatusSignal<AngularVelocity> algaeVelocity2;
    private final StatusSignal<Current> SupplyCurrent2;
    private final StatusSignal<Current> TorqueCurrent2;
    private final StatusSignal<Temperature> TempCelsius2;

    /** Constructor to initialize the TalonFX */
    public AlgaeIO() {
        this.algae = new TalonFX(AlgaeConstants.algaeMotorID, "CamBot");
        this.algae2 = new TalonFX(AlgaeConstants.algaeMotor2ID, "CamBot");

        algaeMagic = new MotionMagicVoltage(0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = AlgaeConstants.algaeGains.CruiseVelocity(); //rps
        mm.MotionMagicAcceleration = AlgaeConstants.algaeGains.Acceleration();
        mm.MotionMagicJerk = AlgaeConstants.algaeGains.Jerk();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = AlgaeConstants.algaeGains.kP();
        slot0.kI = AlgaeConstants.algaeGains.kI();
        slot0.kD = AlgaeConstants.algaeGains.kD();
        slot0.kV = AlgaeConstants.algaeGains.kV();
        slot0.kS = AlgaeConstants.algaeGains.kS();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;

        StatusCode algaeStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            algaeStatus = algae.getConfigurator().apply(cfg);
        if (algaeStatus.isOK()) break;}
        if (!algaeStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + algaeStatus.toString());
        }

        StatusCode algaeStatus2 = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            algaeStatus2 = algae2.getConfigurator().apply(cfg);
        if (algaeStatus2.isOK()) break;}
        if (!algaeStatus2.isOK()) {
            System.out.println("Could not configure device. Error: " + algaeStatus.toString());
        }

        algaePosition = algae.getPosition();
        algaeVelocity = algae.getVelocity();
        SupplyCurrent = algae.getSupplyCurrent();
        TorqueCurrent = algae.getTorqueCurrent();
        TempCelsius = algae.getDeviceTemp();

        algaePosition2 = algae2.getPosition();
        algaeVelocity2 = algae2.getVelocity();
        SupplyCurrent2 = algae2.getSupplyCurrent();
        TorqueCurrent2 = algae2.getTorqueCurrent();
        TempCelsius2 = algae2.getDeviceTemp();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            algaePosition,
            algaeVelocity,
            SupplyCurrent,
            TorqueCurrent,
            TempCelsius,
            algaePosition2,
            algaeVelocity2,
            SupplyCurrent2,
            TorqueCurrent2,
            TempCelsius2
          );
    }



    /** Update stats */
    public void updateStats() {
        stats.algaeMotorConnected =
        BaseStatusSignal.refreshAll(
          algaePosition,
          algaeVelocity,
          SupplyCurrent,
          TorqueCurrent,
          TempCelsius,
          algaePosition2,
          algaeVelocity2,
          SupplyCurrent2,
          TorqueCurrent2,
          TempCelsius2)
            .isOK();

        stats.algaePosition = algaePosition.getValueAsDouble();
        stats.algaeVelocity = algaeVelocity.getValueAsDouble();
        stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
        stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
        stats.TempCelsius = TempCelsius.getValueAsDouble();

        stats.algaePosition2 = algaePosition2.getValueAsDouble();
        stats.algaeVelocity2 = algaeVelocity2.getValueAsDouble();
        stats.SupplyCurrentAmps2 = SupplyCurrent2.getValueAsDouble();
        stats.TorqueCurrentAmps2 = TorqueCurrent2.getValueAsDouble();
        stats.TempCelsius2 = TempCelsius2.getValueAsDouble();
    }


    /** Apply motion magic control mode */
    public void setAlgaeMotorControl(double commandedPosition) {
        algae.setControl(algaeMagic.withPosition(commandedPosition).withSlot(0));
        algae2.setControl(new Follower(algae.getDeviceID(), false));
    }

    /** Stop motor */
    public void stop() {
        algae.setVoltage(0);
    }

    /** Perform simulation-specific tasks */
    abstract public void periodic();
}