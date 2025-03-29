package frc.robot.subsystems.GameSpec.Intake;

import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GameSpec.Arm.ArmConstants;

public abstract class IntakeIO {

    // Protected TalonFX object accessible to subclasses
    protected TalonFX orangeWheels;
    protected TalonFX blackWheels;
    protected TalonFX intakeRotation;
    protected CANcoder intakeCancoder;
    protected PositionVoltage voltageControl;
    public DigitalInput beambreak;

    public static class IntakeIOStats {
        public boolean intakeMotorConnected = true;
        public double intakePosition = 0.0;
        public double intakeVelocity = 0.0;

        public double intakeCancoderPosition = 0.0;

        public boolean PWM1;
    }

    protected static IntakeIOStats stats = new IntakeIOStats();

    private final StatusSignal<Angle> intakePosition;
    private final StatusSignal intakeVelocity;

    private final StatusSignal<Angle> intakeCancoderPosition;

    public CANcoderConfiguration encoderCfg;
    public TalonFXConfiguration cfg;
    public TalonFXConfiguration orangeCFG;
    public TalonFXConfiguration blackCFG;

    private VelocityVoltage blackVelocityVoltage = new VelocityVoltage(0.0);
    private VelocityVoltage orangeVelocityVoltage = new VelocityVoltage(0.0);


    /** Constructor to initialize the TalonFX */
    public IntakeIO() {
        this.orangeWheels = new TalonFX(IntakeConstants.orangeWheelsID, "rio");
        this.blackWheels = new TalonFX(IntakeConstants.blackWheelsID, "rio");
        this.intakeRotation = new TalonFX(IntakeConstants.intakeRotationID, "rio");
        this.intakeCancoder = new CANcoder(IntakeConstants.intakeEncoderID, "rio");
        this.beambreak = new DigitalInput(9);

        voltageControl = new PositionVoltage(0);
        cfg = new TalonFXConfiguration();
        orangeCFG = new TalonFXConfiguration();
        blackCFG = new TalonFXConfiguration();
        encoderCfg = new CANcoderConfiguration();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = IntakeConstants.intakeGains.kP();
        slot0.kI = IntakeConstants.intakeGains.kI();
        slot0.kD = IntakeConstants.intakeGains.kD();

        cfg.Feedback.FeedbackRemoteSensorID = intakeCancoder.getDeviceID();
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.SensorToMechanismRatio = 1/360.0;//changes what the cancoder and fx encoder ratio is
        cfg.Feedback.RotorToSensorRatio = 1;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.Voltage.PeakForwardVoltage = 8;
        cfg.Voltage.PeakReverseVoltage = -8;

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

        blackCFG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //Add PID Gains for Black Intake Motor (KRM 03/29)
        Slot0Configs blackslot0 = blackCFG.Slot0;
        blackslot0.kP = IntakeConstants.intakeBlackGains.kP();
        blackslot0.kI = IntakeConstants.intakeBlackGains.kI();
        blackslot0.kD = IntakeConstants.intakeBlackGains.kD();
        blackslot0.kS = IntakeConstants.intakeBlackGains.kS();
        blackslot0.kV = IntakeConstants.intakeBlackGains.kV();
        blackCFG.Voltage.PeakForwardVoltage = 12;
        blackCFG.Voltage.PeakReverseVoltage = -12;
        blackCFG.withCurrentLimits(
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

        

        orangeCFG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        Slot0Configs orangeslot0 = orangeCFG.Slot0;
        orangeslot0.kP = IntakeConstants.intakeOrangeGains.kP();
        orangeslot0.kI = IntakeConstants.intakeOrangeGains.kI();
        orangeslot0.kD = IntakeConstants.intakeOrangeGains.kD();
        orangeslot0.kS = IntakeConstants.intakeOrangeGains.kS();
        orangeslot0.kV = IntakeConstants.intakeOrangeGains.kV();
        orangeCFG.Voltage.PeakForwardVoltage = 12;
        orangeCFG.Voltage.PeakReverseVoltage = -12;
        orangeCFG.withCurrentLimits(
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
        
        encoderCfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderCfg.MagnetSensor.MagnetOffset = IntakeConstants.intakeEncoderOffset;

        StatusCode intakeStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            intakeStatus = intakeRotation.getConfigurator().apply(cfg);
        if (intakeStatus.isOK()) break;}
        if (!intakeStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + intakeStatus.toString());
        }

        StatusCode blackStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            blackStatus = blackWheels.getConfigurator().apply(blackCFG);
        if (blackStatus.isOK()) break;}
        if (!blackStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + blackStatus.toString());
        }

        StatusCode orangeStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            orangeStatus = orangeWheels.getConfigurator().apply(orangeCFG);
        if (orangeStatus.isOK()) break;}
        if (!orangeStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + orangeStatus.toString());
        }

        StatusCode encoderStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            encoderStatus = intakeCancoder.getConfigurator().apply(encoderCfg);
        if (encoderStatus.isOK()) break;}
        if (!encoderStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + encoderStatus.toString());
        }

        intakePosition = intakeRotation.getPosition();
        intakeVelocity = orangeWheels.getVelocity();

        intakeCancoderPosition = intakeCancoder.getPosition();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            intakePosition,
            intakeVelocity,
            intakeCancoderPosition
          );

          orangeWheels.optimizeBusUtilization();
          blackWheels.optimizeBusUtilization();
          intakeRotation.optimizeBusUtilization();
          intakeCancoder.optimizeBusUtilization();
    }

    /** Update stats */
    public void updateStats() {
        stats.intakeMotorConnected =
        BaseStatusSignal.refreshAll(
          intakePosition,
          intakeCancoderPosition)
            .isOK();

        stats.intakePosition = intakePosition.getValueAsDouble();
        stats.intakeCancoderPosition = intakeCancoderPosition.getValueAsDouble();
        stats.intakeVelocity = intakeVelocity.getValueAsDouble();
    }


    /** Apply motion magic control mode */
    public void setIntakeMotorControl(double rotationPosition) {
        intakeRotation.setControl(voltageControl.withPosition(rotationPosition).withSlot(0));
    }

    public void setIntakeSpinMotorControl(double orangeVoltage, double blackVoltage){
        orangeWheels.setVoltage(orangeVoltage);
        blackWheels.setVoltage(blackVoltage);
    }

    public void setIntakeSpinVelocityControl(double orangeVelocity, double blackVelocity){
        orangeWheels.setControl(orangeVelocityVoltage.withVelocity(orangeVelocity));
        blackWheels.setControl(blackVelocityVoltage.withVelocity(blackVelocity));
    }

    /** Perform simulation-specific tasks */
    abstract public void periodic();
}