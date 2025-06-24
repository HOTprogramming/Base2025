package frc.robot.subsystems.Turret;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

import frc.robot.subsystems.Turret.TurretIO.TurretIOInputs;


public class Turret extends SubsystemBase {

  private final TurretIO turretIO;
  private TurretIOInputs turretSignals = new TurretIOInputs();
  private TurretState turretState = TurretState.IDLE;
  private BooleanSupplier coastOverride = () -> false;
  private final ShuffleboardTab turretShuffleboard;

  /* Shuffleboard entries */
  public GenericEntry turretPosition;
  public GenericEntry turretVelocity; 

  /* Constructor */
  public Turret(TurretIO TurretIO) {
    this.turretIO = TurretIO;

    this.turretShuffleboard = Shuffleboard.getTab("Turret");

    turretPosition = this.turretShuffleboard.add("Turret Position",0.0).getEntry();
    turretVelocity = this.turretShuffleboard.add("Turret Velocity",0.0).getEntry();
  }

  /*Called Automatically by the Command Scheduler */
  @Override
  public void periodic() {
    /*Update the status Signals from the motor */
    turretIO.updateStatusSignals(turretSignals);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      turretIO.stop();
      turretState = TurretState.IDLE;

      if (coastOverride.getAsBoolean()) {
        turretIO.coast();
      }
    } 

    /* Update Shuffleboard Telemetry */
    UpdateTelemetry();

    // switch (turretState) {
    //   case IDLE -> {
    //     if (DriverStation.isEnabled()) {
    //     }
    //   }
    //   case GOTOPOSITION -> {
     
    //     double position = turretSignals.data.positionRads();
    //     if (position >= Units.degreesToRadians(deployAngle)) {
    //       turretIO.runVolts(deployVolts);
    //     } else {
    //       turretIO.stop();
    //     }

       
    //   }
    // }
  }

private void UpdateTelemetry() {
  turretPosition.setDouble(turretSignals.data.positionRads());
  turretVelocity.setDouble(turretSignals.data.velocityRadsPerSec());
}

  public Command readyClimb() {
    return runOnce(
        () -> {
          if (turretState == TurretState.GOTOPOSITION) {
            turretState = TurretState.IDLE;
          } else {
            turretState = TurretState.IDLE;
          }
        });
  }


  public enum TurretState {
    IDLE,
    GOTOPOSITION,
  }
}