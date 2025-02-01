package frc.robot.subsystems.GameSpec.Algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.GameSpec.Coral.Coral;
import frc.robot.subsystems.GameSpec.Coral.CoralConstants;
import frc.robot.subsystems.GameSpec.Coral.CoralIO;
import frc.robot.subsystems.GameSpec.Coral.CoralIOReal;


public abstract class AlgaeIO {

    // Protected TalonFX object accessible to subclasses
    protected TalonFXS algaeArm;
    protected TalonFX algaeRoller;
    protected MotionMagicVoltage algaeMagic;
    protected DigitalInput algaeBeamBreak;
    protected CANcoder algaeCancoder;

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

        public double algaeCancoderPosition = 0.0;
        public double algaeCancoderVelocity = 0.0;

        public boolean candiPWM2;

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

    private final StatusSignal<Angle> algaeCancoderPosition;
    private final StatusSignal<AngularVelocity> algaeCancoderVelocity;
    private final StatusSignal<Boolean> CANdiPWM2;

    /** Constructor to initialize the TalonFX */
    public AlgaeIO() {


        this.algaeArm = new TalonFXS(AlgaeConstants.algaeArmID, "CamBot");
        this.algaeRoller = new TalonFX(AlgaeConstants.algaeRollerID, "CamBot");
        this.algaeCancoder = new CANcoder(AlgaeConstants.algaeEncoderID, "CamBot");

        algaeMagic = new MotionMagicVoltage(0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        TalonFXSConfiguration cFXS = new TalonFXSConfiguration();

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

        cfg.Feedback.FeedbackRemoteSensorID = algaeCancoder.getDeviceID();
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.SensorToMechanismRatio = 1; //changes what the cancoder and fx encoder ratio is
        cfg.Feedback.RotorToSensorRatio = 1; //12.8;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;

        StatusCode algaeStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            algaeStatus = algaeArm.getConfigurator().apply(cFXS);
        if (algaeStatus.isOK()) break;}
        if (!algaeStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + algaeStatus.toString());
        }

        StatusCode algaeStatus2 = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            algaeStatus2 = algaeRoller.getConfigurator().apply(cfg);
        if (algaeStatus2.isOK()) break;}
        if (!algaeStatus2.isOK()) {
            System.out.println("Could not configure device. Error: " + algaeStatus.toString());
        }

        algaePosition = algaeArm.getPosition();
        algaeVelocity = algaeArm.getVelocity();
        SupplyCurrent = algaeArm.getSupplyCurrent();
        TorqueCurrent = algaeArm.getTorqueCurrent();
        TempCelsius = algaeArm.getDeviceTemp();

        algaePosition2 = algaeRoller.getPosition();
        algaeVelocity2 = algaeRoller.getVelocity();
        SupplyCurrent2 = algaeRoller.getSupplyCurrent();
        TorqueCurrent2 = algaeRoller.getTorqueCurrent();
        TempCelsius2 = algaeRoller.getDeviceTemp();

        algaeCancoderPosition = algaeCancoder.getPosition();
        algaeCancoderVelocity = algaeCancoder.getVelocity();

        CANdiPWM2 = CoralIO.coralCandi.getS2Closed();

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
            TempCelsius2,
            algaeCancoderPosition,
            algaeCancoderVelocity,
            CANdiPWM2
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
          TempCelsius2,
          algaeCancoderPosition,
          algaeCancoderVelocity,
          CANdiPWM2)
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

        stats.algaeCancoderPosition = algaeCancoderPosition.getValueAsDouble();
        stats.algaeCancoderVelocity = algaeCancoderVelocity.getValueAsDouble();

        stats.candiPWM2 = CANdiPWM2.getValue();
    }


    /** Apply motion magic control mode */
    public void setAlgaeMotorControl(double commandedPosition) {
        
    }

    /** Stop motor */
    public void stop() {
        algaeRoller.setVoltage(0);
    }

    /** Perform simulation-specific tasks */
    abstract public void periodic();
}