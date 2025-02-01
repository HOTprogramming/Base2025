package frc.robot.subsystems.GameSpec.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public abstract class IntakeIO {

    // Protected TalonFX object accessible to subclasses
    protected TalonFX intakeRoller;
    protected TalonFX intakeRotation;
    protected MotionMagicVoltage intakeMagic;
    protected VelocityTorqueCurrentFOC velocityControl;

    public static class IntakeIOStats {
        public boolean intakeMotorConnected = true;
        public double intakePosition = 0.0;
        public double intakeVelocity = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;

        public boolean intakeMotorConnected2 = true;
        public double intakePosition2 = 0.0;
        public double intakeVelocity2 = 0.0;
        public double intakeAppliedVolts2 = 0.0;
        public double intakeCurrentAmps2 = 0.0;
        public double SupplyCurrentAmps2 = 0.0;
        public double TorqueCurrentAmps2 = 0.0;
        public double TempCelsius2 = 0.0;
    }

    protected static IntakeIOStats stats = new IntakeIOStats();

    private final StatusSignal<Angle> intakePosition;
    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;
    private final StatusSignal<Temperature> TempCelsius;

    private final StatusSignal<Angle> intakePosition2;
    private final StatusSignal<AngularVelocity> intakeVelocity2;
    private final StatusSignal<Current> SupplyCurrent2;
    private final StatusSignal<Current> TorqueCurrent2;
    private final StatusSignal<Temperature> TempCelsius2;

    /** Constructor to initialize the TalonFX */
    public IntakeIO() {
        this.intakeRoller = new TalonFX(IntakeConstants.intakeRollerID, "CamBot");
        this.intakeRotation = new TalonFX(IntakeConstants.intakeRotationID, "CamBot");

        intakeMagic = new MotionMagicVoltage(0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = IntakeConstants.intakeGains.CruiseVelocity(); //rps
        mm.MotionMagicAcceleration = IntakeConstants.intakeGains.Acceleration();
        mm.MotionMagicJerk = IntakeConstants.intakeGains.Jerk();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = IntakeConstants.intakeGains.kP();
        slot0.kI = IntakeConstants.intakeGains.kI();
        slot0.kD = IntakeConstants.intakeGains.kD();
        slot0.kV = IntakeConstants.intakeGains.kV();
        slot0.kS = IntakeConstants.intakeGains.kS();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;

        StatusCode intakeStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            intakeStatus = intakeRoller.getConfigurator().apply(cfg);
        if (intakeStatus.isOK()) break;}
        if (!intakeStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + intakeStatus.toString());
        }

        StatusCode intakeStatus2 = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            intakeStatus2 = intakeRotation.getConfigurator().apply(cfg);
        if (intakeStatus2.isOK()) break;}
        if (!intakeStatus2.isOK()) {
            System.out.println("Could not configure device. Error: " + intakeStatus.toString());
        }

        intakePosition = intakeRoller.getPosition();
        intakeVelocity = intakeRoller.getVelocity();
        SupplyCurrent = intakeRoller.getSupplyCurrent();
        TorqueCurrent = intakeRoller.getTorqueCurrent();
        TempCelsius = intakeRoller.getDeviceTemp();

        intakePosition2 = intakeRotation.getPosition();
        intakeVelocity2 = intakeRotation.getVelocity();
        SupplyCurrent2 = intakeRotation.getSupplyCurrent();
        TorqueCurrent2 = intakeRotation.getTorqueCurrent();
        TempCelsius2 = intakeRotation.getDeviceTemp();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            intakePosition,
            intakeVelocity,
            SupplyCurrent,
            TorqueCurrent,
            TempCelsius,
            intakePosition2,
            intakeVelocity2,
            SupplyCurrent2,
            TorqueCurrent2,
            TempCelsius2
          );
    }



    /** Update stats */
    public void updateStats() {
        stats.intakeMotorConnected =
        BaseStatusSignal.refreshAll(
          intakePosition,
          intakeVelocity,
          SupplyCurrent,
          TorqueCurrent,
          TempCelsius,
          intakePosition2,
          intakeVelocity2,
          SupplyCurrent2,
          TorqueCurrent2,
          TempCelsius2)
            .isOK();

        stats.intakePosition = intakePosition.getValueAsDouble();
        stats.intakeVelocity = intakeVelocity.getValueAsDouble();
        stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
        stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
        stats.TempCelsius = TempCelsius.getValueAsDouble();

        stats.intakePosition2 = intakePosition2.getValueAsDouble();
        stats.intakeVelocity2 = intakeVelocity2.getValueAsDouble();
        stats.SupplyCurrentAmps2 = SupplyCurrent2.getValueAsDouble();
        stats.TorqueCurrentAmps2 = TorqueCurrent2.getValueAsDouble();
        stats.TempCelsius2 = TempCelsius2.getValueAsDouble();
    }


    /** Apply motion magic control mode */
    public void setIntakeMotorControl(double rotationPosition, double rollerSpeedRPM) {
        intakeRoller.setControl(velocityControl.withVelocity(rollerSpeedRPM / 60.0));
        intakeRotation.setControl(intakeMagic.withPosition(rotationPosition).withSlot(0));
    }

    /** Stop motor */
    public void stop() {
        intakeRoller.setVoltage(0);
    }

    /** Perform simulation-specific tasks */
    abstract public void periodic();
}