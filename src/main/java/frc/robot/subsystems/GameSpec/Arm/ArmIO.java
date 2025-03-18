package frc.robot.subsystems.GameSpec.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public abstract class ArmIO {

    // Protected TalonFX object accessible to subclasses
    protected TalonFX arm;
    protected PositionTorqueCurrentFOC armControl;
    //protected MotionMagicVoltage armControl;
    protected CANcoder armCancoder;

    public static class ArmIOStats {
        public boolean armMotorConnected = true;
        public double armPosition = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double armCancoderPosition = 0.0;
    }

    protected static ArmIOStats stats = new ArmIOStats();

    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;
    private final StatusSignal<Angle> armCancoderPosition;

    public TalonFXConfiguration cfg;
    public CANcoderConfiguration encoderCfg;

    /** Constructor to initialize the TalonFX */
    public ArmIO() {
        this.arm = new TalonFX(ArmConstants.armMotorID, "robot");
        this.armCancoder = new CANcoder(ArmConstants.armEncoderID, "robot");

        armControl = new PositionTorqueCurrentFOC(0);
        //armControl = new MotionMagicVoltage(0);
        cfg = new TalonFXConfiguration();
        encoderCfg = new CANcoderConfiguration();

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

        cfg.Feedback.FeedbackRemoteSensorID = armCancoder.getDeviceID();
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.SensorToMechanismRatio = 1/360.0;//changes what the cancoder and fx encoder ratio is
        cfg.Feedback.RotorToSensorRatio = 1;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;
        cfg.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(90)
                .withSupplyCurrentLimitEnable(true)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(65)
                .withPeakReverseTorqueCurrent(-65)
        );
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        encoderCfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderCfg.MagnetSensor.MagnetOffset = ArmConstants.armEncoderOffset;

        setConfig();

        armPosition = arm.getPosition();
        SupplyCurrent = arm.getSupplyCurrent();
        TorqueCurrent = arm.getTorqueCurrent();
        armCancoderPosition = armCancoder.getPosition();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            armPosition,
            SupplyCurrent,
            TorqueCurrent,
            armCancoderPosition
          );
    }

    /** Update stats */
    public void updateStats() {
        stats.armMotorConnected =
        BaseStatusSignal.refreshAll(
          armPosition,
          SupplyCurrent,
          TorqueCurrent,
          armCancoderPosition)
            .isOK();

        stats.armPosition = armPosition.getValueAsDouble();
        stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
        stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
        stats.armCancoderPosition = armCancoderPosition.getValueAsDouble();
    }


    public void setConfig(){
        StatusCode armStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            armStatus = arm.getConfigurator().apply(cfg);
        if (armStatus.isOK()) break;}
        if (!armStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + armStatus.toString());
        }

        StatusCode encoderStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            encoderStatus = armCancoder.getConfigurator().apply(encoderCfg);
        if (encoderStatus.isOK()) break;}
        if (!encoderStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + encoderStatus.toString());
        }

    }

    /** Apply motion magic control mode */
    public void setArmMotorControl(double commandedPosition) {
        arm.setControl(armControl.withPosition(commandedPosition).withSlot(0));
    }

    /** Stop motor */
    public void stop() {
        arm.setVoltage(0);
    }

    public void runVolts(double volts){
        arm.setVoltage(volts);
    }

    /** Perform simulation-specific tasks */
    abstract public void periodic();
}