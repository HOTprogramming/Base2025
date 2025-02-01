package frc.robot.subsystems.GameSpec.Coral;
    
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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;

public abstract class CoralIO {
    // Protected TalonFX object accessible to subclasses
    protected TalonFX coral;
    protected MotionMagicVoltage coralWristMagic;
    protected VelocityVoltage coralSpinController;
    protected DigitalInput coralBeamBreak;
    protected CANdi coralCandi;
    protected TalonFXS coralWrist;
    protected CANcoder coralCancoder;
    
        public static class CoralIOStats {
            public boolean coralMotorConnected = true;
            public double coralPosition = 0.0;
            public double coralVelocity = 0.0;
            public double coralAppliedVolts = 0.0;
            public double coralCurrentAmps = 0.0;
            public double supplyCurrentAmps = 0.0;
            public double torqueCurrentAmps = 0.0;
            public double tempCelsius = 0.0;
            public double coralCancoderPosition = 0.0;
            public double coralCancoderVelocity = 0.0;
            public boolean candiPWM1;
        }
    
        protected static CoralIOStats stats = new CoralIOStats();
    
        private final StatusSignal<Angle> CoralPosition;
        private final StatusSignal<AngularVelocity> CoralVelocity;
        private final StatusSignal<Current> SupplyCurrent;
        private final StatusSignal<Current> TorqueCurrent;
        private final StatusSignal<Temperature> TempCelsius;
        private final StatusSignal<Angle> coralCancoderPosition;
        private final StatusSignal<AngularVelocity> coralCancoderVelocity;
        private final StatusSignal<Boolean> CANdiPWM1;
    
        TalonFXConfiguration cfg;
        TalonFXSConfiguration cFXS;
        CANcoderConfiguration eCfg;
    
        /** Constructor to initialize the TalonFX */
        public CoralIO() {
            this.coral = new TalonFX(CoralConstants.coralMotorID, "rio");
            this.coralCandi = new CANdi(CoralConstants.coralCandiID, "rio");
            this.coralWrist = new TalonFXS(CoralConstants.coralWristID,"rio");
            this.coralCancoder = new CANcoder(CoralConstants.coralEncoderID, "rio");
    
            coralWristMagic = new MotionMagicVoltage(0);
            coralSpinController = new VelocityVoltage(0);
            cfg = new TalonFXConfiguration();
            cFXS = new TalonFXSConfiguration();
            eCfg = new CANcoderConfiguration();
        
            MotionMagicConfigs mm = cFXS.MotionMagic;
            mm.MotionMagicCruiseVelocity = CoralConstants.coralWristGains.CruiseVelocity(); //rps
            mm.MotionMagicAcceleration = CoralConstants.coralWristGains.Acceleration();
            mm.MotionMagicJerk = CoralConstants.coralWristGains.Jerk();
    
            Slot0Configs slot0Wrist = cFXS.Slot0;
            slot0Wrist.kP = CoralConstants.coralWristGains.kP();
            slot0Wrist.kI = CoralConstants.coralWristGains.kI();
            slot0Wrist.kD = CoralConstants.coralWristGains.kD();
            slot0Wrist.kV = CoralConstants.coralWristGains.kV();
            slot0Wrist.kS = CoralConstants.coralWristGains.kS();
    
            Slot0Configs slot0Spin = cfg.Slot0;
            slot0Spin.kP = CoralConstants.coralSpinGains.kP();
            slot0Spin.kI = CoralConstants.coralSpinGains.kI();
            slot0Spin.kD = CoralConstants.coralSpinGains.kD();
            slot0Spin.kV = CoralConstants.coralSpinGains.kV();
            slot0Spin.kS = CoralConstants.coralSpinGains.kS();
    
            cFXS.ExternalFeedback.FeedbackRemoteSensorID = coralCancoder.getDeviceID();
            cFXS.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
            cFXS.ExternalFeedback.SensorToMechanismRatio = 1/360.0; //changes what the cancoder and fx encoder ratio is
            cFXS.ExternalFeedback.RotorToSensorRatio = 1; //12.8;
            cFXS.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            cFXS.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            cFXS.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
            cFXS.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            cFXS.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;
    
            setConfig();
    
            CoralPosition = coral.getPosition();
            CoralVelocity = coral.getVelocity();
            SupplyCurrent = coral.getSupplyCurrent();
            TorqueCurrent = coral.getTorqueCurrent();
            TempCelsius = coral.getDeviceTemp();
            coralCancoderPosition = coralCancoder.getPosition();
            coralCancoderVelocity = coralCancoder.getVelocity();
    
            //getPWM1Position, getRisetoRise
            CANdiPWM1 = coralCandi.getS1Closed();
        
            BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                CoralPosition,
                CoralVelocity,
                SupplyCurrent,
                TorqueCurrent,
                TempCelsius,
                coralCancoderPosition,
                coralCancoderVelocity,
                CANdiPWM1
                );
        }
    
        /** Update stats */
        public void updateStats() {
            stats.coralMotorConnected =
            BaseStatusSignal.refreshAll(
                CoralPosition,
                CoralVelocity,
                SupplyCurrent,
                TorqueCurrent,
                TempCelsius,
                coralCancoderPosition,
                coralCancoderVelocity,
                CANdiPWM1)
                .isOK();
    
            stats.coralPosition = CoralPosition.getValueAsDouble();
            stats.coralVelocity = CoralVelocity.getValueAsDouble();
            stats.supplyCurrentAmps = SupplyCurrent.getValueAsDouble();
            stats.torqueCurrentAmps = TorqueCurrent.getValueAsDouble();
            stats.tempCelsius = TempCelsius.getValueAsDouble();
            stats.coralCancoderPosition = coralCancoderPosition.getValueAsDouble();
            stats.coralCancoderVelocity = coralCancoderVelocity.getValueAsDouble();
            stats.candiPWM1 = CANdiPWM1.getValue();
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
        }
    
    
    /** Apply motion magic control mode */
    public void setCoralSpinMotorControl(double commandedVelocity) {
        coral.setControl(coralSpinController.withVelocity(commandedVelocity).withSlot(0));
    }

    public void setCoralAngleMotorControl(double commandedPosition) {
        // System.out.println(commandedPosition);
        coralWrist.setControl(coralWristMagic.withPosition(commandedPosition).withSlot(0));
    }
    

    /** Stop motor */
    public void stop() {
        coral.setVoltage(0);
    }

    public abstract void periodic();

    public void runVelocity(double rPM, int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runVelocity'");
    }
}