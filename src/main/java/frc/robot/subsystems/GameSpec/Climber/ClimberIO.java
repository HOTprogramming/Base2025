package frc.robot.subsystems.GameSpec.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class ClimberIO {

    // Protected TalonFX object accessible to subclasses
    protected Servo climberServo;
    protected Servo climberServo2;
    protected Servo ratchetServo;
    protected TalonFX climber;
    protected TalonFX climber2;
    protected MotionMagicVoltage climberMagic;

    public static class ClimberIOStats {
        public boolean climberMotorConnected = true;
        public double climberPosition = 0.0;
        public double climberVelocity = 0.0;
        public double climberAppliedVolts = 0.0;
        public double climberCurrentAmps = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;
        public double supplyCurrentVolts = 0.0;

        public boolean climberMotorConnected2 = true;
        public double climberPosition2 = 0.0;
        public double climberVelocity2 = 0.0;
        public double climberAppliedVolts2 = 0.0;
        public double climberCurrentAmps2 = 0.0;
        public double SupplyCurrentAmps2 = 0.0;
        public double TorqueCurrentAmps2 = 0.0;
        public double TempCelsius2 = 0.0;
        public double servoVelocity = 0.0;

        public double supplyCurrentVolts2 = 0.0;

    }

    protected static ClimberIOStats stats = new ClimberIOStats();

    private final StatusSignal<Angle> climberPosition;
    private final StatusSignal<AngularVelocity> climberVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;
    private final StatusSignal<Temperature> TempCelsius;
    private final StatusSignal<Voltage> voltage;

    private final StatusSignal<Angle> climberPosition2;
    private final StatusSignal<AngularVelocity> climberVelocity2;
    private final StatusSignal<Current> SupplyCurrent2;
    private final StatusSignal<Current> TorqueCurrent2;
    private final StatusSignal<Temperature> TempCelsius2;
    private final StatusSignal<AngularVelocity> servoVelocity;
    private final StatusSignal<Voltage> voltage2;

    /** Constructor to initialize the TalonFX */
    public ClimberIO() {
        this.climber = new TalonFX(ClimberConstants.climberMotorID, "robot");
        this.climber2 = new TalonFX(ClimberConstants.climberMotor2ID, "robot");
        this.climberServo = new Servo(ClimberConstants.ServoPort);
        this.climberServo2 = new Servo(ClimberConstants.ServoPort2);
        this.ratchetServo = new Servo(ClimberConstants.ServoPort3);
        climberMagic = new MotionMagicVoltage(0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = ClimberConstants.climberGains.CruiseVelocity(); //rps
        mm.MotionMagicAcceleration = ClimberConstants.climberGains.Acceleration();
        mm.MotionMagicJerk = ClimberConstants.climberGains.Jerk();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = ClimberConstants.climberGains.kP();
        slot0.kI = ClimberConstants.climberGains.kI();
        slot0.kD = ClimberConstants.climberGains.kD();
        slot0.kV = ClimberConstants.climberGains.kV();
        slot0.kS = ClimberConstants.climberGains.kS();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1;
        
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;

        cfg.CurrentLimits.StatorCurrentLimitEnable = false;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = false;
        
        StatusCode climberStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            climberStatus = climber.getConfigurator().apply(cfg);
        if (climberStatus.isOK()) break;}
        if (!climberStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + climberStatus.toString());
        }

        StatusCode climberStatus2 = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            climberStatus2 = climber2.getConfigurator().apply(cfg);
        if (climberStatus2.isOK()) break;}
        if (!climberStatus2.isOK()) {
            System.out.println("Could not configure device. Error: " + climberStatus.toString());
        }

        climberPosition = climber.getPosition();
        climberVelocity = climber.getVelocity();
        SupplyCurrent = climber.getSupplyCurrent();
        TorqueCurrent = climber.getTorqueCurrent();
        TempCelsius = climber.getDeviceTemp();
        voltage = climber.getMotorVoltage();
        servoVelocity = climber.getVelocity();

        climberPosition2 = climber2.getPosition();
        climberVelocity2 = climber2.getVelocity();
        SupplyCurrent2 = climber2.getSupplyCurrent();
        TorqueCurrent2 = climber2.getTorqueCurrent();
        TempCelsius2 = climber2.getDeviceTemp();
        voltage2 = climber.getMotorVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            climberPosition,
            climberVelocity,
            SupplyCurrent,
            TorqueCurrent,
            TempCelsius,
            voltage,
            climberPosition2,
            climberVelocity2,
            SupplyCurrent2,
            TorqueCurrent2,
            TempCelsius2,
            voltage2,
            servoVelocity
          );

          climber.setPosition(0.0);
    }



    /** Update stats */
    public void updateStats() {
        stats.climberMotorConnected =
        BaseStatusSignal.refreshAll(
            servoVelocity,
          climberPosition,
          climberVelocity,
          SupplyCurrent,
          TorqueCurrent,
          TempCelsius,
          voltage,
          climberPosition2,
          climberVelocity2,
          SupplyCurrent2,
          TorqueCurrent2,
          voltage2,
          TempCelsius2)
            .isOK();

        stats.climberPosition = climberPosition.getValueAsDouble();
        stats.climberVelocity = climberVelocity.getValueAsDouble();
        stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
        stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
        stats.TempCelsius = TempCelsius.getValueAsDouble();
        stats.climberAppliedVolts = voltage.getValueAsDouble();
        stats.climberAppliedVolts2 = voltage2.getValueAsDouble();

        stats.climberPosition2 = climberPosition2.getValueAsDouble();
        stats.climberVelocity2 = climberVelocity2.getValueAsDouble();
        stats.SupplyCurrentAmps2 = SupplyCurrent2.getValueAsDouble();
        stats.TorqueCurrentAmps2 = TorqueCurrent2.getValueAsDouble();
        stats.TempCelsius2 = TempCelsius2.getValueAsDouble();
        stats.servoVelocity = servoVelocity.getValueAsDouble();
        
    }


    /** Apply motion magic control mode */
    public void setClimberMotorControl(double commandedPosition) {
      climber.setVoltage(6);
        // climber.setControl(climberMagic.withPosition(commandedPosition).withSlot(0));
        climber2.setControl(new Follower(climber.getDeviceID(), false));
    }
    public void setNegClimberMotorControl(double commandedPosition) {
        climber.setVoltage(-6);
        // climber.setControl(climberMagic.withPosition(commandedPosition).withSlot(0));
        climber2.setControl(new Follower(climber.getDeviceID(), false));
    }
    // public void setServoMotorControl(double commandedPosition) {
    //     climberServo.set(commandedPosition);
    // }
    /** Stop motor */
    public void stop() {
        climber.setVoltage(0);
        climber2.setControl(new Follower(climber.getDeviceID(), false));
    }

    public void setPower(double power) {
        climber.setVoltage(power *16.0);
        climber2.setControl(new Follower(climber.getDeviceID(), false));
    }

    /** Perform simulation-specific tasks */
    abstract public void periodic();
}