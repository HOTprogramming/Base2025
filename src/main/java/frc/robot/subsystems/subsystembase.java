package frc.robot.subsystems;

public interface subsystembase {
    public void update();
    public void goToPosition(double position);
    public double getPosition();
    public double getTarget();
    public boolean isActive();
    public boolean isConfigured();
    public boolean atTarget();
    public void stop();
}
