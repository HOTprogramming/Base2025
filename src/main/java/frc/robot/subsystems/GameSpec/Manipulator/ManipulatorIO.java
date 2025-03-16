package frc.robot.subsystems.GameSpec.Manipulator;
    
import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;

public abstract class ManipulatorIO {
    // Protected TalonFX object accessible to subclasses
    protected TalonFX coral;
    protected VelocityTorqueCurrentFOC coralSpinController;
    protected PositionVoltage positionVoltage;
    protected DigitalInput coralBeamBreak;
    protected CANdi coralCandi;
    public TalonFXS coralWrist;
    protected CANcoder coralCancoder;

    protected TalonFXS algaeArm;
    protected TalonFX algaeRoller;
    protected MotionMagicVoltage algaeMagic;
    protected DigitalInput algaeBeamBreak;
    protected CANcoder algaeCancoder;
    protected CANrange canRange;
    
        public static class ManipulatorIOStats {
            public boolean coralMotorConnected = true;
            public double coralPosition = 0.0;
            public double coralVelocity = 0.0;
            public double coralAppliedVolts = 0.0;
            public double coralCurrentAmps = 0.0;
            public double coralSupplyCurrentAmps = 0.0;
            public double coralTorqueCurrentAmps = 0.0;
            public double coralTempCelsius = 0.0;
            public double coralCancoderPosition = 0.0;
            public double coralCancoderVelocity = 0.0;
            public boolean candiPWM1;


            public boolean algaeMotorConnected = true;
            public double algaePosition = 0.0;
            public double algaeVelocity = 0.0;
            public double algaeAppliedVolts = 0.0;
            public double algaeCurrentAmps = 0.0;
            public double algaeSupplyCurrentAmps = 0.0;
            public double algaeTorqueCurrentAmps = 0.0;
            public double algaeTempCelsius = 0.0;

            public double algaeDistance = 0.0;

            public boolean candiPWM2;
            public boolean candiPWM3;
        }
    
        protected static ManipulatorIOStats stats = new ManipulatorIOStats();
    
        private final StatusSignal<Angle> CoralPosition;
        private final StatusSignal<AngularVelocity> CoralVelocity;
        private final StatusSignal<Current> CoralSupplyCurrent;
        private final StatusSignal<Current> CoralTorqueCurrent;
        private final StatusSignal<Temperature> CoralTempCelsius;
        private final StatusSignal<Angle> CoralCancoderPosition;
        private final StatusSignal<AngularVelocity> CoralCancoderVelocity;
        private final StatusSignal<Boolean> CANdiPWM1;

        private final StatusSignal<Angle> AlgaePosition;
        private final StatusSignal<AngularVelocity> AlgaeVelocity;
        private final StatusSignal<Current> AlgaeSupplyCurrent;
        private final StatusSignal<Current> AlgaeTorqueCurrent;
        private final StatusSignal<Temperature> AlgaeTempCelsius;

        private final StatusSignal<Boolean> CANdiPWM2;
        private final StatusSignal<Boolean> CANdiPWM3;
        private final StatusSignal<Distance> CANrangeDistance;
        private TalonFXConfiguration cfg;
        private TalonFXSConfiguration cFXS;
        private CANcoderConfiguration eCfg;
        private CANrangeConfiguration rangeConfig;
        
    
        /** Constructor to initialize the TalonFX */
        public ManipulatorIO() {
            this.coral = new TalonFX(ManipulatorConstants.coralMotorID, "robot");
            this.coralCandi = new CANdi(ManipulatorConstants.coralCandiID, "robot");
            this.coralWrist = new TalonFXS(ManipulatorConstants.coralWristID,"robot");
            this.coralCancoder = new CANcoder(ManipulatorConstants.coralEncoderID, "robot");
            this.canRange = new CANrange(ManipulatorConstants.canRangeID, "robot");

            this.algaeRoller = new TalonFX(ManipulatorConstants.algaeRollerID, "robot");
    
            positionVoltage = new PositionVoltage(0);
            cFXS = new TalonFXSConfiguration();
            cfg = new TalonFXConfiguration();
            eCfg = new CANcoderConfiguration();
    
            Slot0Configs slot0 = cFXS.Slot0;
            slot0.kP = ManipulatorConstants.coralWristGains.kP();
            slot0.kI = ManipulatorConstants.coralWristGains.kI();
            slot0.kD = ManipulatorConstants.coralWristGains.kD();
            slot0.kV = ManipulatorConstants.coralWristGains.kV();
            slot0.kS = ManipulatorConstants.coralWristGains.kS();

            FeedbackConfigs fdb = cfg.Feedback;
            fdb.SensorToMechanismRatio = 1;

            cfg.Feedback.SensorToMechanismRatio = 1; //changes what the cancoder and fx encoder ratio is
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
    
            cFXS.ExternalFeedback.FeedbackRemoteSensorID = coralCancoder.getDeviceID();
            cFXS.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
            cFXS.ExternalFeedback.SensorToMechanismRatio = 1/360.0; //changes what the cancoder and fx encoder ratio is
            cFXS.ExternalFeedback.RotorToSensorRatio = 1; //12.8;
            cFXS.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            cFXS.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            cFXS.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
            cFXS.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            cFXS.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;
            cFXS.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;
            
            cFXS.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(90)
                    .withSupplyCurrentLimitEnable(true)
            );
            eCfg.MagnetSensor.MagnetOffset = ManipulatorConstants.coralWristEncoderOffset;
            //score -0.246

            rangeConfig = new CANrangeConfiguration();

            rangeConfig.FovParams = new FovParamsConfigs()
                .withFOVCenterX(0.0)
                .withFOVCenterY(0.0)
                .withFOVRangeX(10.0)
                .withFOVRangeY(10.0);

            rangeConfig.ProximityParams = new ProximityParamsConfigs()
                .withMinSignalStrengthForValidMeasurement(7500);

            rangeConfig.ToFParams = new ToFParamsConfigs()
                .withUpdateMode(UpdateModeValue.LongRangeUserFreq);

    
            setConfig();
    
            CoralPosition = coral.getPosition();
            CoralVelocity = coral.getVelocity();
            CoralSupplyCurrent = coral.getSupplyCurrent();
            CoralTorqueCurrent = coral.getTorqueCurrent();
            CoralTempCelsius = coral.getDeviceTemp();
            CoralCancoderPosition = coralCancoder.getPosition();
            CoralCancoderVelocity = coralCancoder.getVelocity();
    
            //getPWM1Position, getRisetoRise
            CANdiPWM1 = coralCandi.getS1Closed();

            AlgaePosition = algaeRoller.getPosition();
            AlgaeVelocity = algaeRoller.getVelocity();
            AlgaeSupplyCurrent = algaeRoller.getSupplyCurrent();
            AlgaeTorqueCurrent = algaeRoller.getTorqueCurrent();
            AlgaeTempCelsius = algaeRoller.getDeviceTemp();

            CANdiPWM2 = coralCandi.getS2Closed();
            CANdiPWM3 = coralCandi.getS2Closed();

            CANrangeDistance = canRange.getDistance();

            BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                CoralPosition,
                CoralVelocity,
                CoralSupplyCurrent,
                CoralTorqueCurrent,
                CoralTempCelsius,
                CoralCancoderPosition,
                CoralCancoderVelocity,
                CANdiPWM1,
                AlgaePosition,
                AlgaeVelocity,
                AlgaeSupplyCurrent,
                AlgaeTorqueCurrent,
                AlgaeTempCelsius,
                CANdiPWM2,
                CANdiPWM3,
                CANrangeDistance
                );
        }
    
        /** Update stats */
        public void updateStats() {
            stats.coralMotorConnected =
            BaseStatusSignal.refreshAll(
                CoralPosition,
                CoralVelocity,
                CoralSupplyCurrent,
                CoralTorqueCurrent,
                CoralTempCelsius,
                CoralCancoderPosition,
                CoralCancoderVelocity,
                CANdiPWM1,
                AlgaePosition,
                AlgaeVelocity,
                AlgaeSupplyCurrent,
                AlgaeTorqueCurrent,
                AlgaeTempCelsius,
                CANdiPWM3,
                CANdiPWM2,
                CANrangeDistance)
                .isOK();
    
            stats.coralPosition = CoralPosition.getValueAsDouble();
            stats.coralVelocity = CoralVelocity.getValueAsDouble();
            stats.coralSupplyCurrentAmps = CoralSupplyCurrent.getValueAsDouble();
            stats.coralTorqueCurrentAmps = CoralTorqueCurrent.getValueAsDouble();
            stats.coralTempCelsius = CoralTempCelsius.getValueAsDouble();
            stats.coralCancoderPosition = CoralCancoderPosition.getValueAsDouble();
            stats.coralCancoderVelocity = CoralCancoderVelocity.getValueAsDouble();
            stats.candiPWM1 = CANdiPWM1.getValue();

            stats.algaePosition = AlgaePosition.getValueAsDouble();
            stats.algaeVelocity = AlgaeVelocity.getValueAsDouble();
            stats.algaeSupplyCurrentAmps = AlgaeSupplyCurrent.getValueAsDouble();
            stats.algaeTorqueCurrentAmps = AlgaeTorqueCurrent.getValueAsDouble();
            stats.algaeTempCelsius = AlgaeTempCelsius.getValueAsDouble();
    
            stats.candiPWM3 = CANdiPWM3.getValue();
            stats.candiPWM2 = CANdiPWM2.getValue();

            stats.algaeDistance = CANrangeDistance.getValueAsDouble();
        }
    
        public void setConfig(){

            StatusCode CoralStatus2 = StatusCode.StatusCodeNotInitialized;
            for(int i = 0; i < 5; ++i) {
                CoralStatus2 = coralWrist.getConfigurator().apply(cFXS);
            if (CoralStatus2.isOK()) break;}
            if (!CoralStatus2.isOK()) {
                System.out.println("Could not configure device. Error: " + CoralStatus2.toString());
            }
    
            
            StatusCode CoralEncoder = StatusCode.StatusCodeNotInitialized;
            for(int i = 0; i < 5; ++i) {
                CoralEncoder = coralCancoder.getConfigurator().apply(eCfg);
            if (CoralEncoder.isOK()) break;}
            if (!CoralEncoder.isOK()) {
                System.out.println("Could not configure device. Error: " + CoralEncoder.toString());
            }
    
            StatusCode algaeStatus = StatusCode.StatusCodeNotInitialized;
            for(int i = 0; i < 5; ++i) {
                algaeStatus = algaeRoller.getConfigurator().apply(cfg);
            if (algaeStatus.isOK()) break;}
            if (!algaeStatus.isOK()) {
                System.out.println("Could not configure device. Error: " + algaeStatus.toString());
            }

            StatusCode RangeStatus = StatusCode.StatusCodeNotInitialized;
            for(int i = 0; i < 5; ++i) {
                RangeStatus = canRange.getConfigurator().apply(rangeConfig);
            if (RangeStatus.isOK()) break;}
            if (!RangeStatus.isOK()) {
                System.out.println("Could not configure device. Error: " + RangeStatus.toString());
            }
        }
    
    
    /** Apply motion magic control mode */
    public void setCoralSpinMotorControl(double commandedVoltage) {
        // coral.setControl(coralSpinController.withVelocity(commandedVelocity).withSlot(0));
        coral.setVoltage(commandedVoltage);
    }

    public void setCoralAngleMotorControl(double commandedPosition) {
        // System.out.println(commandedPosition);
        coralWrist.setControl(positionVoltage.withPosition(commandedPosition).withSlot(0));
    }
    

    /** Stop motor */
    public void stop() {
        coral.setVoltage(0);
        if(stats.algaeDistance > ManipulatorConstants.algaeTriggerDistance){
            algaeRoller.setVoltage(0.0);
        }else{
            algaeRoller.setVoltage(ManipulatorConstants.algaeHoldVoltage);
        }
        coralWrist.setControl(positionVoltage.withPosition(ManipulatorConstants.coralWristScore).withSlot(0));
    }

    public void goScore() {
        if(stats.algaeDistance > ManipulatorConstants.algaeTriggerDistance){
            algaeRoller.setVoltage(0.0);
        }else{
            algaeRoller.setVoltage(ManipulatorConstants.algaeHoldVoltage);
        }
        coralWrist.setControl(positionVoltage.withPosition(ManipulatorConstants.coralWristScore).withSlot(0));
    }

    public void setAlgaeSpinMotorControl(double commandedVoltage){
        algaeRoller.setVoltage(commandedVoltage);
    }


    public abstract void periodic(); 
    

    public void runVelocity(double rPM, int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runVelocity'");
    }
}