package frc.util.SubsystemExamples;

public interface ExampleMotorInterface {
    /**
     * Call periodically
     */
    public void update();
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
     * @return Is the subsystem attempting to achive the setpoint
     */
    public boolean isActive();
    /**
     * @return Is all IO in the subsystem correctly configured
     */
    public boolean isConfigured();
    /**
     * @return Is the subsystem at its target within the tolerance
     */
    public boolean atTarget();
    /**
     * Stops all output in subsystem
     */
    public void stop();
}
