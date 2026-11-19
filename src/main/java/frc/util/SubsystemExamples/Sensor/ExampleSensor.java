package frc.util.SubsystemExamples.Sensor;

/**
 * This class doesnt have an interface or much defined for it due to how varied the output of different sensors can be.
 * <p>
 * This class type was originally made for a set of beambreaks in a gripper, but can be applied to any sensor that does
 * not directly/cleanly interact with a single component.
 */
public class ExampleSensor {
    // declare IO

    
    public ExampleSensor() {
        // initialize sensor object
    }

    public void update() {} // update sensor value

    public boolean getValue1() {
        return true;
    }

    public double getValue2() {
        return 0.0;
    }
}
