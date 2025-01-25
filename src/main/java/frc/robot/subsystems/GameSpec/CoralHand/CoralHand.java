package frc.robot.subsystems.GameSpec.CoralHand;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.GameSpec.CoralHand.CoralHandIO.CoralHandIOStats;

public class CoralHand extends SubsystemBase {

    private final CoralHandIO io;
    public final CoralHandIOStats stats;
    private final ShuffleboardTab coralHandShuffleboard;

    /* Shuffleboard entries */
    private GenericEntry coralHandVelocity;
    private GenericEntry coralHandPosition;
    private GenericEntry coralHandSupplyCurrent;
    private GenericEntry coralHandStatorCurrent;
    private GenericEntry coralHandTemp;
    private GenericEntry coralHandCommandedPos;
    

    public CoralHand(CoralHandIO io) {
        this.io = io;
        this.stats = CoralHandIO.stats;

        this.coralHandShuffleboard = Shuffleboard.getTab("CoralHand");

        coralHandVelocity = this.coralHandShuffleboard.add("CoralHand RPM", 0.0).getEntry();
        coralHandPosition = this.coralHandShuffleboard.add("CoralHand Position", 0.0).getEntry();
        coralHandSupplyCurrent = this.coralHandShuffleboard.add("CoralHand Supply Current", 0.0).getEntry();
        coralHandStatorCurrent = this.coralHandShuffleboard.add("CoralHand Stator Current", 0.0).getEntry();
        coralHandTemp = this.coralHandShuffleboard.add("CoralHand Temp", 0.0).getEntry();
    }

    @Override
    public void periodic() {
        io.periodic();

        io.updateStats();
          
        UpdateTelemetry();
    }

    private void UpdateTelemetry() {
        coralHandVelocity.setDouble(stats.coralHandVelocity);
        coralHandPosition.setDouble(stats.coralHandPosition);
        coralHandSupplyCurrent.setDouble(stats.supplyCurrentAmps);
        coralHandStatorCurrent.setDouble(stats.torqueCurrentAmps);
        coralHandTemp.setDouble(stats.tempCelsius);
    }

    private FunctionalCommand coralCommand(double position){
    return new FunctionalCommand(
      () -> this.coralHandCommandedPos.setDouble(position),
      () -> io.setCoralHandMotorControl(position),
      interrupted -> io.setCoralHandMotorControl(position), 
      () -> checkCoralRange(.1),
      this);
    }

      public  Command intakeCommand() {
        return coralCommand(0);
    }

    public   Command shootCommand() {
        return coralCommand(0);
    }

    public  Command degree45RotationCommand() {
        return coralCommand(0);
    }

    public  Command degree90RotationCommand() {
        return coralCommand(0);
    }

    public  Command packageCommand() {
        return coralCommand(0);
    }


    public boolean checkCoralRange(double deadband){
        return (stats.coralHandPosition >= coralHandCommandedPos.getDouble(0) - deadband) && 
               (stats.coralHandPosition <= coralHandCommandedPos.getDouble(0) + deadband);
    }

    @Override
    public void simulationPeriodic() {
        io.periodic(); // Update simulated stats
    }
}