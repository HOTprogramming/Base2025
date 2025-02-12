package frc.robot.subsystems.GameSpec.Manipulator;
    
import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;

public abstract class ManipulatorIO {
    // Protected TalonFX object accessible to subclasses
    protected TalonFX coral;
    protected MotionMagicVoltage coralWristMagic;
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
    
            public boolean algaeMotorConnected2 = true;
            public double algaePosition2 = 0.0;
            public double algaeVelocity2 = 0.0;
            public double algaeAppliedVolts2 = 0.0;
            public double algaeCurrentAmps2 = 0.0;
            public double algaeSupplyCurrentAmps2 = 0.0;
            public double algaeTorqueCurrentAmps2 = 0.0;
            public double algaeTempCelsius2 = 0.0;
    
            public double algaeCancoderPosition = 0.0;
            public double algaeCancoderVelocity = 0.0;
    
            public boolean candiPWM2;
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
    
        private final StatusSignal<Angle> AlgaePosition2;
        private final StatusSignal<AngularVelocity> AlgaeVelocity2;
        private final StatusSignal<Current> AlgaeSupplyCurrent2;
        private final StatusSignal<Current> AlgaeTorqueCurrent2;
        private final StatusSignal<Temperature> AlgaeTempCelsius2;
    
        private final StatusSignal<Angle> AlgaeCancoderPosition;
        private final StatusSignal<AngularVelocity> AlgaeCancoderVelocity;
        private final StatusSignal<Boolean> CANdiPWM2;
    
        private TalonFXConfiguration cfg;
        private TalonFXSConfiguration cFXS;
        private CANcoderConfiguration eCfg;
        
    
        /** Constructor to initialize the TalonFX */
        public ManipulatorIO() {
            this.coral = new TalonFX(ManipulatorConstants.coralMotorID, "robot");
            this.coralCandi = new CANdi(ManipulatorConstants.coralCandiID, "robot");
            this.coralWrist = new TalonFXS(ManipulatorConstants.coralWristID,"robot");
            this.coralCancoder = new CANcoder(ManipulatorConstants.coralEncoderID, "robot");

            this.algaeArm = new TalonFXS(ManipulatorConstants.algaeArmID, "robot");
            this.algaeRoller = new TalonFX(ManipulatorConstants.algaeRollerID, "robot");
            this.algaeCancoder = new CANcoder(ManipulatorConstants.algaeEncoderID, "robot");
    
            algaeMagic = new MotionMagicVoltage(0);
            coralWristMagic = new MotionMagicVoltage(0);
            positionVoltage = new PositionVoltage(0);
            coralSpinController = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
            cfg = new TalonFXConfiguration();
            cFXS = new TalonFXSConfiguration();
            eCfg = new CANcoderConfiguration();
        
            MotionMagicConfigs mmC = cFXS.MotionMagic;
            mmC.MotionMagicCruiseVelocity = ManipulatorConstants.coralWristGains.CruiseVelocity(); //rps
            mmC.MotionMagicAcceleration = ManipulatorConstants.coralWristGains.Acceleration();
            mmC.MotionMagicJerk = ManipulatorConstants.coralWristGains.Jerk();

            MotionMagicConfigs mmA = cfg.MotionMagic;
            mmA.MotionMagicCruiseVelocity = ManipulatorConstants.algaeGains.CruiseVelocity(); //rps
            mmA.MotionMagicAcceleration = ManipulatorConstants.algaeGains.Acceleration();
            mmA.MotionMagicJerk = ManipulatorConstants.algaeGains.Jerk();
    
            Slot0Configs slot0 = cFXS.Slot0;
            slot0.kP = ManipulatorConstants.coralWristGains.kP();
            slot0.kI = ManipulatorConstants.coralWristGains.kI();
            slot0.kD = ManipulatorConstants.coralWristGains.kD();
            slot0.kV = ManipulatorConstants.coralWristGains.kV();
            slot0.kS = ManipulatorConstants.coralWristGains.kS();

            // slot0.kP = ManipulatorConstants.coralSpinGains.kP();
            // slot0.kI = ManipulatorConstants.coralSpinGains.kI();
            // slot0.kD = ManipulatorConstants.coralSpinGains.kD();
            // slot0.kV = ManipulatorConstants.coralSpinGains.kV();
            // slot0.kS = ManipulatorConstants.coralSpinGains.kS();

            // slot0.kP = ManipulatorConstants.algaeGains.kP();
            // slot0.kI = ManipulatorConstants.algaeGains.kI();
            // slot0.kD = ManipulatorConstants.algaeGains.kD();
            // slot0.kV = ManipulatorConstants.algaeGains.kV();
            // slot0.kS = ManipulatorConstants.algaeGains.kS();


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

            eCfg.MagnetSensor.MagnetOffset = -0.472;
            //score -0.246


    
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


            AlgaePosition = algaeArm.getPosition();
            AlgaeVelocity = algaeArm.getVelocity();
            AlgaeSupplyCurrent = algaeArm.getSupplyCurrent();
            AlgaeTorqueCurrent = algaeArm.getTorqueCurrent();
            AlgaeTempCelsius = algaeArm.getDeviceTemp();

            AlgaePosition2 = algaeRoller.getPosition();
            AlgaeVelocity2 = algaeRoller.getVelocity();
            AlgaeSupplyCurrent2 = algaeRoller.getSupplyCurrent();
            AlgaeTorqueCurrent2 = algaeRoller.getTorqueCurrent();
            AlgaeTempCelsius2 = algaeRoller.getDeviceTemp();

            AlgaeCancoderPosition = algaeCancoder.getPosition();
            AlgaeCancoderVelocity = algaeCancoder.getVelocity();

            CANdiPWM2 = coralCandi.getS2Closed();
        
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
                AlgaePosition2,
                AlgaeVelocity2,
                AlgaeSupplyCurrent2,
                AlgaeTorqueCurrent2,
                AlgaeTempCelsius2,
                AlgaeCancoderPosition,
                AlgaeCancoderVelocity,
                CANdiPWM2
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
                AlgaePosition2,
                AlgaeVelocity2,
                AlgaeSupplyCurrent2,
                AlgaeTorqueCurrent2,
                AlgaeTempCelsius2,
                AlgaeCancoderPosition,
                AlgaeCancoderVelocity,
                CANdiPWM2)
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
    
            stats.algaePosition2 = AlgaePosition2.getValueAsDouble();
            stats.algaeVelocity2 = AlgaeVelocity2.getValueAsDouble();
            stats.algaeSupplyCurrentAmps2 = AlgaeSupplyCurrent2.getValueAsDouble();
            stats.algaeTorqueCurrentAmps2 = AlgaeTorqueCurrent2.getValueAsDouble();
            stats.algaeTempCelsius2 = AlgaeTempCelsius2.getValueAsDouble();
    
            stats.algaeCancoderPosition = AlgaeCancoderPosition.getValueAsDouble();
            stats.algaeCancoderVelocity = AlgaeCancoderVelocity.getValueAsDouble();
    
            stats.candiPWM2 = CANdiPWM2.getValue();
        }
    
        public void setConfig(){
            StatusCode CoralStatus = StatusCode.StatusCodeNotInitialized;
            for(int i = 0; i < 5; ++i) {
                CoralStatus = coral.getConfigurator().apply(cfg);
            if (CoralStatus.isOK()) break;}
            if (!CoralStatus.isOK()) {
                System.out.println("Could not configure device. Error: " + CoralStatus.toString());
            }

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
        }
    
    
    /** Apply motion magic control mode */
    public void setCoralSpinMotorControl(double commandedVelocity) {
        // coral.setControl(coralSpinController.withVelocity(commandedVelocity).withSlot(0));
        coral.setVoltage(commandedVelocity);
    }

    public void setCoralAngleMotorControl(double commandedPosition) {
        // System.out.println(commandedPosition);
        coralWrist.setControl(positionVoltage.withPosition(commandedPosition).withSlot(0));
    }
    

    /** Stop motor */
    public void stop() {
        coral.setVoltage(0);
        algaeRoller.setVoltage(0);
        coralWrist.setControl(positionVoltage.withPosition(ManipulatorConstants.coralWristScore).withSlot(0));
    }

    /** Apply motion magic control mode */
    public void setAlgaeMotorControl(double commandedPosition) {
        algaeArm.setControl(algaeMagic.withPosition(commandedPosition).withSlot(0));
    }


    public abstract void periodic(); 
    

    public void runVelocity(double rPM, int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runVelocity'");
    }
}