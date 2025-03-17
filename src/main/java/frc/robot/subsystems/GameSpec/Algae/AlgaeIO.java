package frc.robot.subsystems.GameSpec.Algae;
    
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

public abstract class AlgaeIO {
    protected TalonFX algaeRoller;
    protected CANrange canRange;
    
        public static class AlgaeIOStats {

            public double algaeDistance = 0.0;

        }
    
        protected static AlgaeIOStats stats = new AlgaeIOStats();

        private final StatusSignal<Distance> CANrangeDistance;

        private CANrangeConfiguration rangeConfig;
        
    
        /** Constructor to initialize the TalonFX */
        public AlgaeIO() {
            this.canRange = new CANrange(AlgaeConstants.canRangeID, "robot");

            this.algaeRoller = new TalonFX(AlgaeConstants.algaeRollerID, "robot");

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

            CANrangeDistance = canRange.getDistance();

            BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                CANrangeDistance
                );
        }
    
        /** Update stats */
        public void updateStats() {
            BaseStatusSignal.refreshAll(
                CANrangeDistance)
                .isOK();

            stats.algaeDistance = CANrangeDistance.getValueAsDouble();
        }
    
        public void setConfig(){

            StatusCode RangeStatus = StatusCode.StatusCodeNotInitialized;
            for(int i = 0; i < 5; ++i) {
                RangeStatus = canRange.getConfigurator().apply(rangeConfig);
            if (RangeStatus.isOK()) break;}
            if (!RangeStatus.isOK()) {
                System.out.println("Could not configure device. Error: " + RangeStatus.toString());
            }
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