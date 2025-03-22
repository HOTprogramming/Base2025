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

import edu.wpi.first.wpilibj.DigitalInput;
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
    protected DigitalInput limitSwitch1;
    protected DigitalInput limitSwitch2;

    public static class ClimberIOStats {
        public double climberPosition = 0.0;
        public double supplyCurrentAmps = 0.0;

        public double climberPosition2 = 0.0;
        public double supplyCurrentAmps2 = 0.0;
        public boolean climberMotorConnected = false;
        
            }
        
            protected static ClimberIOStats stats = new ClimberIOStats();
        
            private final StatusSignal<Angle> climberPosition;
            private final StatusSignal<Current> SupplyCurrent;
        
            private final StatusSignal<Angle> climberPosition2;
            private final StatusSignal<Current> SupplyCurrent2;
        
            /** Constructor to initialize the TalonFX */
            public ClimberIO() {
                this.climber = new TalonFX(ClimberConstants.climberMotorID, "robot");
                this.climber2 = new TalonFX(ClimberConstants.climberMotor2ID, "robot");
                this.climberServo = new Servo(ClimberConstants.ServoPort);
                this.climberServo2 = new Servo(ClimberConstants.ServoPort2);
                this.ratchetServo = new Servo(ClimberConstants.ServoPort3);
                this.limitSwitch1 = new DigitalInput(7);
                this.limitSwitch2 = new DigitalInput(8);
                TalonFXConfiguration cfg = new TalonFXConfiguration();
        
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
                SupplyCurrent = climber.getSupplyCurrent();
        
                climberPosition2 = climber2.getPosition();
                SupplyCurrent2 = climber2.getSupplyCurrent();
                BaseStatusSignal.setUpdateFrequencyForAll(
                    100.0,
                    climberPosition,
                    SupplyCurrent,
                    climberPosition2,
                    SupplyCurrent2
                  );
        
                  climber.setPosition(0.0);

                  climber.optimizeBusUtilization();
                  climber2.optimizeBusUtilization();
            }
        
        
        
            /** Update stats */
            public void updateStats() {
        stats.climberMotorConnected =
        BaseStatusSignal.refreshAll(
          climberPosition,
          SupplyCurrent,
          climberPosition2,
          SupplyCurrent2
          )
            .isOK();

        stats.climberPosition = climberPosition.getValueAsDouble();
        stats.supplyCurrentAmps = SupplyCurrent.getValueAsDouble();

        stats.climberPosition2 = climberPosition2.getValueAsDouble();
        stats.supplyCurrentAmps2 = SupplyCurrent2.getValueAsDouble();
        
    }



    public void setClimberMotorControl(double commandedPosition) {
      climber.setVoltage(6);
        climber2.setControl(new Follower(climber.getDeviceID(), false));
    }
    public void setNegClimberMotorControl(double commandedPosition) {
        climber.setVoltage(-6);
        climber2.setControl(new Follower(climber.getDeviceID(), false));
    }

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