package frc.robot.subsystems.GameSpec.CoralHand;
    
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
import edu.wpi.first.wpilibj.DigitalInput;
public class CoralHandIO {
    
    
        // Protected TalonFX object accessible to subclasses
        protected TalonFX coralHand;
        protected MotionMagicVoltage coralHandMagic;
        protected DigitalInput coralBeamBreak;
    
        public static class CoralHandIOStats {
            public boolean coralHandMotorConnected = true;
            public double coralHandPosition = 0.0;
            public double coralHandVelocity = 0.0;
            public double coralHandAppliedVolts = 0.0;
            public double coralHandCurrentAmps = 0.0;
            public double supplyCurrentAmps = 0.0;
            public double torqueCurrentAmps = 0.0;
            public double tempCelsius = 0.0;
        }
    
        protected static CoralHandIOStats stats = new CoralHandIOStats();
    
        private final StatusSignal<Angle> CoralHandPosition;
        private final StatusSignal<AngularVelocity> CoralHandVelocity;
        private final StatusSignal<Current> SupplyCurrent;
        private final StatusSignal<Current> TorqueCurrent;
        private final StatusSignal<Temperature> TempCelsius;
    
        /** Constructor to initialize the TalonFX */
        public CoralHandIO() {
            this.coralHand = new TalonFX(CoralHandConstants.coralHandMotorID, "CamBot");
            this.coralBeamBreak = new DigitalInput(0);
    
            coralHandMagic = new MotionMagicVoltage(0);
            TalonFXConfiguration cfg = new TalonFXConfiguration();
    
            MotionMagicConfigs mm = cfg.MotionMagic;
            mm.MotionMagicCruiseVelocity = CoralHandConstants.coralHandGains.CruiseVelocity(); //rps
            mm.MotionMagicAcceleration = CoralHandConstants.coralHandGains.Acceleration();
            mm.MotionMagicJerk = CoralHandConstants.coralHandGains.Jerk();
    
            Slot0Configs slot0 = cfg.Slot0;
            slot0.kP = CoralHandConstants.coralHandGains.kP();
            slot0.kI = CoralHandConstants.coralHandGains.kI();
            slot0.kD = CoralHandConstants.coralHandGains.kD();
            slot0.kV = CoralHandConstants.coralHandGains.kV();
            slot0.kS = CoralHandConstants.coralHandGains.kS();
    
            FeedbackConfigs fdb = cfg.Feedback;
            fdb.SensorToMechanismRatio = 1;
    
            cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0;
            cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;
    
            StatusCode CoralHandStatus = StatusCode.StatusCodeNotInitialized;
            for(int i = 0; i < 5; ++i) {
                CoralHandStatus = coralHand.getConfigurator().apply(cfg);
            if (CoralHandStatus.isOK()) break;}
            if (!CoralHandStatus.isOK()) {
                System.out.println("Could not configure device. Error: " + CoralHandStatus.toString());
            }
    
            CoralHandPosition = coralHand.getPosition();
            CoralHandVelocity = coralHand.getVelocity();
            SupplyCurrent = coralHand.getSupplyCurrent();
            TorqueCurrent = coralHand.getTorqueCurrent();
            TempCelsius = coralHand.getDeviceTemp();
        
            BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                CoralHandPosition,
                CoralHandVelocity,
                SupplyCurrent,
                TorqueCurrent,
                TempCelsius
              );
        }
    
        /** Update stats */
        public void updateStats() {
            stats.coralHandMotorConnected =
            BaseStatusSignal.refreshAll(
                CoralHandPosition,
                CoralHandVelocity,
              SupplyCurrent,
              TorqueCurrent,
              TempCelsius)
                .isOK();
    
            stats.coralHandPosition = CoralHandPosition.getValueAsDouble();
            stats.coralHandVelocity = CoralHandVelocity.getValueAsDouble();
            stats.supplyCurrentAmps = SupplyCurrent.getValueAsDouble();
            stats.torqueCurrentAmps = TorqueCurrent.getValueAsDouble();
            stats.tempCelsius = TempCelsius.getValueAsDouble();
        }
    
    
        /** Apply motion magic control mode */
        public void setCoralHandMotorControl(double commandedPosition) {
            coralHand.setControl(coralHandMagic.withPosition(commandedPosition).withSlot(0));
        }

        public boolean coralBeamBreakTriggered(){
            return coralBeamBreak.get();
        }
    
        /** Stop motor */
        public void stop() {
            coralHand.setVoltage(0);
        }
    
        /** Perform simulation-specific tasks */
        public void periodic() {
        }

        public void runVelocity(double rPM, int i) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'runVelocity'");
        }
    }