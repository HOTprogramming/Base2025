package frc.robot.subsystems.CoralGripper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;

/**
 * This class doesnt have an interface or much defined for it due to how varied the output of different sensors can be.
 * <p>
 * This class type was originally made for a set of beambreaks in a gripper, but can be applied to any sensor that does
 * not directly/cleanly interact with a single component.
 */
public class CoralSensor {
    // declare IO
    private final CANdi beambreaks;

    private StatusCode sensorFault;

    // status signals
    private final StatusSignal<Boolean> Beambreak1;
    private final StatusSignal<Boolean> Beambreak2;

    
    public CoralSensor() {
        beambreaks = new CANdi(
            CoralGripperConstants.kCandiID, CoralGripperConstants.canbus);

        sensorFault = StatusCode.StatusCodeNotInitialized;

        Beambreak1 = beambreaks.getS1Closed();
        Beambreak2 = beambreaks.getS2Closed();

        BaseStatusSignal.setUpdateFrequencyForAll(
            CoralGripperConstants.kStatusSignalUpdateRate,
            Beambreak1,
            Beambreak2
          );
    }

    public void update() {
        this.sensorFault = BaseStatusSignal.refreshAll(
            Beambreak1,
            Beambreak2
        );
    }

    public StatusCode getSensorStatus() {
        return this.sensorFault;
    }

    public boolean getHasCoral() {
        return Beambreak1.getValue();
    }

    public boolean getCoralAlignment() {
        return Beambreak2.getValue();
    }
}
