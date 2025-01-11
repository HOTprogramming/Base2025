package frc.robot.subsystems.GameSpec.Arm;

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

public abstract class ArmIO {

    // Protected TalonFX object accessible to subclasses
    protected TalonFX arm;
    protected MotionMagicVoltage armMagic;

    public static class ArmIOStats {
        public boolean armMotorConnected = true;
        public double armPosition = 0.0;
        public double armVelocity = 0.0;
        public double armAppliedVolts = 0.0;
        public double armCurrentAmps = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;
    }

    protected static ArmIOStats stats = new ArmIOStats();

    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<AngularVelocity> armVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;
    private final StatusSignal<Temperature> TempCelsius;

    /** Constructor to initialize the TalonFX */
    public ArmIO() {
        this.arm = new TalonFX(ArmConstants.armMotorID, "CamBot");

        armMagic = new MotionMagicVoltage(0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = ArmConstants.armGains.CruiseVelocity(); //rps
        mm.MotionMagicAcceleration = ArmConstants.armGains.Acceleration();
        mm.MotionMagicJerk = ArmConstants.armGains.Jerk();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = ArmConstants.armGains.kP();
        slot0.kI = ArmConstants.armGains.kI();
        slot0.kD = ArmConstants.armGains.kD();
        slot0.kV = ArmConstants.armGains.kV();
        slot0.kS = ArmConstants.armGains.kS();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;

        StatusCode armStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            armStatus = arm.getConfigurator().apply(cfg);
        if (armStatus.isOK()) break;}
        if (!armStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + armStatus.toString());
        }

        armPosition = arm.getPosition();
        armVelocity = arm.getVelocity();
        SupplyCurrent = arm.getSupplyCurrent();
        TorqueCurrent = arm.getTorqueCurrent();
        TempCelsius = arm.getDeviceTemp();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            armPosition,
            armVelocity,
            SupplyCurrent,
            TorqueCurrent,
            TempCelsius
          );
    }

    /** Update stats */
    public void updateStats() {
        stats.armMotorConnected =
        BaseStatusSignal.refreshAll(
          armPosition,
          armVelocity,
          SupplyCurrent,
          TorqueCurrent,
          TempCelsius)
            .isOK();

        stats.armPosition = armPosition.getValueAsDouble();
        stats.armVelocity = armVelocity.getValueAsDouble();
        stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
        stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
        stats.TempCelsius = TempCelsius.getValueAsDouble();
    }


    /** Apply motion magic control mode */
    public void setArmMotorControl(double commandedPosition) {
        arm.setControl(armMagic.withPosition(commandedPosition).withSlot(0));
    }

    /** Stop motor */
    public void stop() {
        arm.setVoltage(0);
    }

    /** Perform simulation-specific tasks */
    abstract public void periodic();
}