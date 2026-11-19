package frc.util.SubsystemExamples;

/**
 * There should be one interface PER subsystem component.
 * <p>
 * EX: an elevator will have one component,
 * an intake could have two, one to manage the extension, and one for the rollers
 * <p>
 * Subsystems should not share components, and components should not share data between eachother.
 */
public interface ExampleMotorInterface {
    /**
     * Call periodically
     */
    public void update();
    /**
     * Updates the components target
     * @param setpoint
     */
    public void goToSetpoint(double setpoint);
    /**
     * @return Current Rpm/Position
     */
    public double getState();
    /**
     * @return Current setpoint
     */
    public double getTarget();
    /**
     * @return Is the component attempting to achive the setpoint
     */
    public boolean isActive();
    /**
     * @return Is all IO in the component correctly configured
     */
    public boolean isConfigured();
    /**
     * @return Is the component at its target within the tolerance
     */
    public boolean atTarget();
    /**
     * Stops all motor output in component
     */
    public void stop();
}
