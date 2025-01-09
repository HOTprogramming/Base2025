package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public abstract class ElevatorIO {

    // Protected TalonFX object accessible to subclasses
    protected TalonFX elevatorMotor;

    public static class ElevatorIOStats {
        public boolean elevatorMotorConnected = true;
        public boolean f_fusedSensorOutOfSync;
        public boolean sf_fusedSensorOutOfSync;
        public boolean f_remoteSensorInvalid;
        public boolean sf_remoteSensorInvalid;
        public double elevatorPosition = 0.0;
        public double krakenElevatorPosition = 0.0;
        public double elevatorVelocity = 0.0;
        public double cancoderPosition = 0.0;
        public double cancoderVelocity = 0.0;
        public double elevatorRotorPos = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double elevatorCurrentAmps = 0.0;

        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;
    }

    protected static ElevatorIOStats ioStats = new ElevatorIOStats();

    /** Constructor to initialize the TalonFX */
    public ElevatorIO(int talonFXId, String canBus) {
        this.elevatorMotor = new TalonFX(talonFXId, canBus);
    }

    /** Update stats */
    public abstract void updateStats(ElevatorIOStats stats);

    /** Apply motion magic control mode */
    public abstract void setElevatorMotorControl(double commandedPosition);

    /** Run motor at a specific voltage */
    public void runVolts(double volts) {
        // Optional override in subclasses
    }

    /** Run motor at a specific velocity with feedforward */
    public void runVelocity(double rpm, double feedforward) {
        // Optional override in subclasses
    }

    /** Set PID gains */
    public void setPID(double kP, double kI, double kD) {
        // Optional override in subclasses
    }

    /** Stop motor */
    public void stop() {
        // Optional override in subclasses
    }

    /** Run motor characterization */
    public void runCharacterization(double input) {
        // Optional override in subclasses
    }

    /** Move to a specific goal position */
    public void reachGoal(double goal) {
        // Optional override in subclasses
    }

    /** Perform simulation-specific tasks */
    public void simStuff() {
        // Optional override in subclasses
    }
}