package frc.robot.subsystems.GameSpec.Manipulator;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Arm.ArmConstants;
import frc.robot.subsystems.GameSpec.Manipulator.ManipulatorIO.ManipulatorIOStats;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Manipulator extends SubsystemBase {

    private final ManipulatorIO io;
    public final ManipulatorIOStats stats;
    private final ShuffleboardTab coralShuffleboard;
    private int delay = 0;

    /* Shuffleboard entries */
    private GenericEntry coralVelocity;
    private GenericEntry coralPosition;
    private GenericEntry coralSupplyCurrent;
    private GenericEntry coralStatorCurrent;
    private GenericEntry coralTemp;
    private GenericEntry coralCommandedPos;
    private GenericEntry coralCommandedSpeed;
    private GenericEntry CANdiPWM1;

    private GenericEntry CANdiPWM2;
    private GenericEntry CANdiPWM3;
    private GenericEntry CANrange;

    
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

        CANdiPWM2 = this.coralShuffleboard.add("CANdi Algae Beambreak",false).getEntry();//false when there is no object, true when it detects object
        CANdiPWM3 = this.coralShuffleboard.add("Outer BeamBreak",false).getEntry();//false when there is no object, true when it detects object

        CANrange = this.coralShuffleboard.add("Algae Distance", 0.0).getEntry();

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

        CANdiPWM2.setBoolean(stats.candiPWM2);
        CANdiPWM3.setBoolean(stats.candiPWM3);
    }

    private FunctionalCommand coralCommand(double position){
        return new FunctionalCommand(
            () -> this.coralCommandedPos.setDouble(position),
            () -> 
            {io.setCoralAngleMotorControl(position);}
            ,
            interrupted -> {io.setCoralAngleMotorControl(position);
            }, 
            () -> stats.coralCancoderPosition <= position + .01 && stats.coralCancoderPosition >= position - .01,
            this
        );
    }

    public Command shoot() {
        return runOnce(() -> {
            coralCommandedSpeed.setDouble(-1.5);
            io.setCoralSpinMotorControl(-1.5);
            io.setCoralAngleMotorControl(ManipulatorConstants.coralWristHP);
        });
    }

    public Command intake() {
        return run(() -> {
            coralCommandedSpeed.setDouble(8);
            io.setCoralSpinMotorControl(8);
            io.setCoralAngleMotorControl(ManipulatorConstants.coralWristHP);

        }).onlyWhile(() -> stats.candiPWM1).andThen(Commands.waitSeconds(0.4)).andThen(zero());    
    }

    public Command intakeGround() {
        return run(() -> {
            coralCommandedSpeed.setDouble(8);
            io.setCoralSpinMotorControl(8);
            io.setCoralAngleMotorControl(ManipulatorConstants.coralWristScore);

        });    
    }

    public Command autonIntake() {
        return run(() -> {
            coralCommandedSpeed.setDouble(8);
            io.setCoralSpinMotorControl(8);
            io.setCoralAngleMotorControl(ManipulatorConstants.coralWristHP);

        }).onlyWhile(() -> stats.candiPWM1).andThen(Commands.waitSeconds(0.2)).andThen(zero());    
    }

    public Command L4Spit(){
        return runOnce(() -> io.setCoralSpinMotorControl(-5));
    }

    /** 
     * @apiNote false when it has coral
     */
    public Boolean returnBeamBreak(){
        return stats.candiPWM1;
    }
    public Boolean returnOuterBeamBreak(){
        return stats.candiPWM3;
    }
    
    public  Command goHP() {
        return coralCommand(ManipulatorConstants.coralWristHP);
    }

    public  Command goScore() {
        return coralCommand(ManipulatorConstants.coralWristScore);
    }

    public Command zero(){
        return runOnce(() -> io.stop());
    }
    
    public Command BeamBreak2Stop() {
        return run(() -> {
            if (stats.candiPWM2 == true) {
                coralCommandedSpeed.setDouble(0);
                io.setCoralSpinMotorControl(0);
            }
        });
    }

        
            public boolean checkCoralRange(double deadband){
        return (stats.coralPosition >= coralCommandedPos.getDouble(0) - deadband) && 
               (stats.coralPosition <= coralCommandedPos.getDouble(0) + deadband);
    }
    
        public Command stop(){
          return run(() -> {
              io.stop();
          });  
        }


    @Override
    public void simulationPeriodic() {
        io.periodic(); // Update simulated stats
    }
}