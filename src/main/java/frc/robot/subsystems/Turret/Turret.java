package frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret.TurretIO.TurretIOStats;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOStats stats = new TurretIOStats();
  
  private static final double INTAKE_SPEED = 2000.0; 
  private static final double EJECT_SPEED = 1000.0;
  private static final double INTAKE_STOP = 0.0; 

  public enum State {
    IDLE(INTAKE_STOP),
    INTAKE(INTAKE_SPEED),
    CHARACTERIZING(INTAKE_SPEED),
    EJECT(-EJECT_SPEED);

    private final double rpm;
  
    private State (double rpmIn) {
        this.rpm = rpmIn; 
    }

  }

  /* Variable to hold the current state of the state machine  */
  private State currentState = State.IDLE;

  /** Creates a new IntakeSubsystem. */
  public Turret(TurretIO io) {
    this.io = io; 
  }

  private void flipState(State inState ) {
   System.out.println("Setting state...." + inState.name());
   currentState = inState; 
  }

  /**
   * Set command to Intake
   *
   * @return a command
   */
  public Command intakeCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.INTAKE);
        });
  }

   /**
   * Set command to Idle
   *
   * @return a command
   */
  public Command idleCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.IDLE);
        });
  }

   /**
   * Set Command to Eject
   *
   * @return a command
   */
  public Command ejectCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {flipState(State.EJECT); });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    /* This method will be called once per scheduler run */
    
    //Detwrmine motor speed and process 
    double motorRPM = currentState.rpm;
    io.runVelocity(motorRPM, 0.0); 

    //go update the signal data 
    io.updateStats(stats);
    
    //Update Dashboard with Telementry Data 
    UpdateTelemetry();
  
   
  }

  private void UpdateTelemetry() {
    SmartDashboard.putNumber("Applied Volts:",stats.AppliedVolts);
    SmartDashboard.putNumber("Velocity Radians per Sec:",stats.VelocityRadPerSec);
    SmartDashboard.putNumber("Position (rads):",stats.PositionRads);
    SmartDashboard.putNumber("Supply Current(Amps):",stats.SupplyCurrentAmps);
    SmartDashboard.putString("StateName", currentState.name());
    SmartDashboard.putNumber("Goal RPM", currentState.rpm);
    SmartDashboard.putNumber("Applied Volts:",stats.AppliedVolts); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}