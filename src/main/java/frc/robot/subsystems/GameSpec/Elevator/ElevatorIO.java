package frc.robot.subsystems.GameSpec.Elevator;

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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public abstract class ElevatorIO {

    // Protected TalonFX object accessible to subclasses
    protected TalonFX elevator;
    protected TalonFX elevator2;
    //protected MotionMagicVoltage elevatorMagic;
    //protected MotionMagicTorqueCurrentFOC elevatorControl;
    protected PositionTorqueCurrentFOC elevatorControl;
    protected CANcoder elevatorCancoder;

    public static class ElevatorIOStats {
        public boolean elevatorMotorConnected = true;
        public double elevatorPosition = 0.0;
        public double elevatorVelocity = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double elevatorCurrentAmps = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;

        public boolean elevatorMotorConnected2 = true;
        public double elevatorPosition2 = 0.0;
        public double elevatorVelocity2 = 0.0;
        public double elevatorAppliedVolts2 = 0.0;
        public double elevatorCurrentAmps2 = 0.0;
        public double SupplyCurrentAmps2 = 0.0;
        public double TorqueCurrentAmps2 = 0.0;
        public double TempCelsius2 = 0.0;

        public double elevatorCancoderPosition = 0.0;
        public double elevatorCancoderVelocity = 0.0;
    }

    protected static ElevatorIOStats stats = new ElevatorIOStats();

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Current> SupplyCurrent;
    private final StatusSignal<Current> TorqueCurrent;
    private final StatusSignal<Temperature> TempCelsius;

    private final StatusSignal<Angle> elevatorPosition2;
    private final StatusSignal<AngularVelocity> elevatorVelocity2;
    private final StatusSignal<Current> SupplyCurrent2;
    private final StatusSignal<Current> TorqueCurrent2;
    private final StatusSignal<Temperature> TempCelsius2;

    private final StatusSignal<Angle> elevatorCancoderPosition;
    private final StatusSignal<AngularVelocity> elevatorCancoderVelocity;

    private TalonFXConfiguration cfg;
    private CANcoderConfiguration encoderCfg;

    /** Constructor to initialize the TalonFX */
    public ElevatorIO() {
        this.elevator = new TalonFX(ElevatorConstants.elevatorMotorID, "robot");
        this.elevator2 = new TalonFX(ElevatorConstants.elevatorMotor2ID, "robot");
        this.elevatorCancoder = new CANcoder(ElevatorConstants.elevatorEncoderID, "robot");

        //sets up the second elevator motor as a follower
        elevator2.setControl(new Follower(elevator.getDeviceID(), true));

        //elevatorMagic = new MotionMagicVoltage(0);
        //elevatorControl = new MotionMagicTorqueCurrentFOC(0);
        elevatorControl = new PositionTorqueCurrentFOC(0);
        cfg = new TalonFXConfiguration();
        encoderCfg = new CANcoderConfiguration();

        // MotionMagicConfigs mm = cfg.MotionMagic;
        // mm.MotionMagicCruiseVelocity = ElevatorConstants.elevatorGains.CruiseVelocity(); //rps
        // mm.MotionMagicAcceleration = ElevatorConstants.elevatorGains.Acceleration();
        // mm.MotionMagicJerk = ElevatorConstants.elevatorGains.Jerk();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = ElevatorConstants.elevatorGains.kP();
        slot0.kI = ElevatorConstants.elevatorGains.kI();
        slot0.kD = ElevatorConstants.elevatorGains.kD();
        slot0.kV = ElevatorConstants.elevatorGains.kV();
        slot0.kS = ElevatorConstants.elevatorGains.kS();
        slot0.kG = ElevatorConstants.elevatorGains.kG();

        slot0.GravityType = GravityTypeValue.Elevator_Static;

        cfg.Feedback.FeedbackRemoteSensorID = elevatorCancoder.getDeviceID();
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        cfg.Feedback.SensorToMechanismRatio = 0.208/24.75; //changes what the cancoder and fx encoder ratio is
        cfg.Feedback.RotorToSensorRatio = 1;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;

        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(100)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(100)
                .withSupplyCurrentLimitEnable(true)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(160)
                .withPeakReverseTorqueCurrent(-160)
        );
        encoderCfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderCfg.MagnetSensor.MagnetOffset = ElevatorConstants.elevatorEncoderOffset;

        applyConfig();

        elevatorPosition = elevator.getPosition();
        elevatorVelocity = elevator.getVelocity();
        SupplyCurrent = elevator.getSupplyCurrent();
        TorqueCurrent = elevator.getTorqueCurrent();
        TempCelsius = elevator.getDeviceTemp();

        elevatorPosition2 = elevator2.getPosition();
        elevatorVelocity2 = elevator2.getVelocity();
        SupplyCurrent2 = elevator2.getSupplyCurrent();
        TorqueCurrent2 = elevator2.getTorqueCurrent();
        TempCelsius2 = elevator2.getDeviceTemp();

        elevatorCancoderPosition = elevatorCancoder.getPosition();
        elevatorCancoderVelocity = elevatorCancoder.getVelocity();
    
        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            elevatorPosition,
            elevatorVelocity,
            SupplyCurrent,
            TorqueCurrent,
            TempCelsius,
            elevatorPosition2,
            elevatorVelocity2,
            SupplyCurrent2,
            TorqueCurrent2,
            TempCelsius2,
            elevatorCancoderPosition,
            elevatorCancoderVelocity
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
          TempCelsius,
          elevatorPosition2,
          elevatorVelocity2,
          SupplyCurrent2,
          TorqueCurrent2,
          TempCelsius2,
          elevatorCancoderPosition,
          elevatorCancoderVelocity)
            .isOK();

        stats.elevatorPosition = elevatorPosition.getValueAsDouble();
        stats.elevatorVelocity = elevatorVelocity.getValueAsDouble();
        stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
        stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
        stats.TempCelsius = TempCelsius.getValueAsDouble();

        stats.elevatorPosition2 = elevatorPosition2.getValueAsDouble();
        stats.elevatorVelocity2 = elevatorVelocity2.getValueAsDouble();
        stats.SupplyCurrentAmps2 = SupplyCurrent2.getValueAsDouble();
        stats.TorqueCurrentAmps2 = TorqueCurrent2.getValueAsDouble();
        stats.TempCelsius2 = TempCelsius2.getValueAsDouble();

        stats.elevatorCancoderPosition = elevatorCancoderPosition.getValueAsDouble();
        stats.elevatorCancoderVelocity = elevatorCancoderVelocity.getValueAsDouble();
    }

    public void applyConfig(){
        StatusCode elevatorStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            elevatorStatus = elevator.getConfigurator().apply(cfg);
        if (elevatorStatus.isOK()) break;}
        if (!elevatorStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + elevatorStatus.toString());
        }

        StatusCode elevatorStatus2 = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            elevatorStatus2 = elevator2.getConfigurator().apply(cfg);
        if (elevatorStatus2.isOK()) break;}
        if (!elevatorStatus2.isOK()) {
            System.out.println("Could not configure device. Error: " + elevatorStatus.toString());
        }

        StatusCode encoderStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            encoderStatus = elevatorCancoder.getConfigurator().apply(encoderCfg);
        if (encoderStatus.isOK()) break;}
        if (!encoderStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + encoderStatus.toString());
        }
    }


    //applies control mode to the elevator motor
    public void setElevatorMotorControl(double commandedPosition) {
        elevator.setControl(elevatorControl.withPosition(commandedPosition).withSlot(0));
    }

    /** Stop motor */
    public void stop() {
        elevator.setVoltage(0);
    }

    /** Perform simulation-specific tasks */
    abstract public void periodic();
}