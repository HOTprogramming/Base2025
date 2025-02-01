package frc.robot.subsystems.GameSpec.Manipulator;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Manipulator.ManipulatorIO.ManipulatorIOStats;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Manipulator extends SubsystemBase {

    private final ManipulatorIO io;
    public final ManipulatorIOStats stats;
    private final ShuffleboardTab coralShuffleboard;

    /* Shuffleboard entries */
    private GenericEntry coralVelocity;
    private GenericEntry coralPosition;
    private GenericEntry coralSupplyCurrent;
    private GenericEntry coralStatorCurrent;
    private GenericEntry coralTemp;
    private GenericEntry coralCommandedPos;
    private GenericEntry coralCommandedSpeed;
    private GenericEntry CANdiPWM1;

    public GenericEntry algaePosition;
    public GenericEntry algaeDirection;
    private GenericEntry algaeVelocity;
    private GenericEntry algaeSupplyCurrent;
    private GenericEntry algaeStatorCurrent;
    private GenericEntry algaeTemp;
    private GenericEntry algaeCommandedPos;
    private GenericEntry CANdiPWM2;
    
    public Manipulator(ManipulatorIO io) {
        this.io = io;
        this.stats = ManipulatorIO.stats;

        this.coralShuffleboard = Shuffleboard.getTab("Coral");

        coralVelocity = this.coralShuffleboard.add("Coral RPM", 0.0).getEntry();
        coralPosition = this.coralShuffleboard.add("Coral Position", 0.0).getEntry();
        coralSupplyCurrent = this.coralShuffleboard.add("Coral Supply Current", 0.0).getEntry();
        coralStatorCurrent = this.coralShuffleboard.add("Coral Stator Current", 0.0).getEntry();
        coralTemp = this.coralShuffleboard.add("Coral Temp", 0.0).getEntry();
        coralCommandedPos = this.coralShuffleboard.add("Coral Commanded Position", 0.0).getEntry();
        coralCommandedSpeed = this.coralShuffleboard.add("Coral Commanded Speed", 0.0).getEntry();
        CANdiPWM1 = this.coralShuffleboard.add("CANdi Coral Beambreak",false).getEntry();//false when there is no object, true when it detects object

        algaeVelocity = this.coralShuffleboard.add("Algae RPM", 0.0).getEntry();
        algaePosition = this.coralShuffleboard.add("Algae Position", 0.0).getEntry();;
        algaeSupplyCurrent = this.coralShuffleboard.add("Algae Supply Current", 0.0).getEntry();
        algaeStatorCurrent = this.coralShuffleboard.add("Algae Stator Current", 0.0).getEntry();
        algaeTemp = this.coralShuffleboard.add("Algae Temp", 0.0).getEntry();
        algaeCommandedPos = this.coralShuffleboard.add("Algae Commanded Position", 0.0).getEntry();
        CANdiPWM2 = this.coralShuffleboard.add("CANdi Algae Beambreak",false).getEntry();//false when there is no object, true when it detects object
  
    }

    @Override
    public void periodic() {
        io.periodic();

        io.updateStats();
          
        UpdateTelemetry();
    }

    private void UpdateTelemetry() {
        coralVelocity.setDouble(io.coral.getVelocity().getValueAsDouble());
        coralPosition.setDouble(io.coralWrist.getPosition().getValueAsDouble());
        coralSupplyCurrent.setDouble(stats.coralSupplyCurrentAmps);
        coralStatorCurrent.setDouble(stats.coralTorqueCurrentAmps);
        coralTemp.setDouble(stats.coralTempCelsius);
        CANdiPWM1.setBoolean(stats.candiPWM1);

        algaeVelocity.setDouble(stats.algaeVelocity);
        algaePosition.setDouble(stats.algaePosition);
        algaeSupplyCurrent.setDouble(stats.algaeSupplyCurrentAmps);
        algaeStatorCurrent.setDouble(stats.algaeTorqueCurrentAmps);
        algaeTemp.setDouble(stats.algaeTempCelsius);
        CANdiPWM2.setBoolean(stats.candiPWM2);
    }

    private FunctionalCommand coralCommand(double position){
        return new FunctionalCommand(
            () -> this.coralCommandedPos.setDouble(position),
            () -> io.setCoralAngleMotorControl(position),
            interrupted -> io.setCoralAngleMotorControl(position), 
            () -> true,
            this
        );
    }

    public Command shoot() {
        return runOnce(() -> {
            coralCommandedSpeed.setDouble(2);
            io.setCoralSpinMotorControl(2);
        });
    }

    public Command intake() {
        return runOnce(() -> {
            coralCommandedSpeed.setDouble(-2);
            io.setCoralSpinMotorControl(-2);
        });    
    }

    public  Command goHorizontal() {
        return coralCommand(0);
    }

    public  Command goVertical() {
        return coralCommand(90);
    }

    public Command zero(){
        return run(() -> io.stop());
    }

    public boolean checkCoralRange(double deadband){
        return (stats.coralPosition >= coralCommandedPos.getDouble(0) - deadband) && 
               (stats.coralPosition <= coralCommandedPos.getDouble(0) + deadband);
    }

    public Command runToPositionAlgae(double position){
        return run(() -> {
            this.algaeCommandedPos.setDouble(position);
            io.setAlgaeMotorControl(position);
        });
      }
    
      private FunctionalCommand algaeCommand(double position){
        return new FunctionalCommand(
          () -> this.algaeCommandedPos.setDouble(position),
          () -> io.setAlgaeMotorControl(position),
          interrupted -> io.setAlgaeMotorControl(position), 
          () -> checkAlgaeRange(.1),
          this);
      }
    
        public Command stop(){
          return run(() -> {
              io.stop();
          });  
        }
    
        public Command hold(){
          return run(() -> {
            io.setAlgaeMotorControl(algaeCommandedPos.getDouble(0));
          });
        }
     
      public boolean checkAlgaeRange(double deadband){
        return (stats.algaePosition >= algaeCommandedPos.getDouble(0) - deadband) && 
               (stats.algaePosition <= algaeCommandedPos.getDouble(0) + deadband);
      }

    @Override
    public void simulationPeriodic() {
        io.periodic(); // Update simulated stats
    }
}