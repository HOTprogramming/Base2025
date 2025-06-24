package frc.robot.subsystems.Turret;


public interface TurretIO {

  class TurretIOInputs {
    public TurretIOData data = new TurretIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record TurretIOData(
      boolean motorConnected,
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double torqueCurrentAmps,
      double supplyVolts,
      double tempCelsius) {}

  default void updateStatusSignals(TurretIOInputs inputs) {}

  default void runTorqueCurrent(double current) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void coast() {}
}