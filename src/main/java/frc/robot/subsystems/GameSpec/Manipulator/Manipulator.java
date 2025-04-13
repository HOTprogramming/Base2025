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

    /* Shuffleboard entries */
    private GenericEntry wristCancoderPosition;
    private GenericEntry wristCommandedPos;
    private GenericEntry CANdiPWM1;
    private GenericEntry CANdiPWM3;
    private GenericEntry wristPosition;

    
    public Manipulator(ManipulatorIO io) {
        this.io = io;
        this.stats = ManipulatorIO.stats;

        this.coralShuffleboard = Shuffleboard.getTab("Coral");

        wristCancoderPosition = this.coralShuffleboard.add("Wrist Cancoder Position", 0.0).getEntry();
        wristPosition = this.coralShuffleboard.add("Wrist Position", 0.0).getEntry();
        wristCommandedPos = this.coralShuffleboard.add("Wrist Commanded Position", 0.0).getEntry();
        CANdiPWM1 = this.coralShuffleboard.add("CANdi Wrist Beambreak",false).getEntry();//false when there is no object, true when it detects object
        CANdiPWM3 = this.coralShuffleboard.add("Outer BeamBreak",false).getEntry();//false when there is no object, true when it detects object

    }

    @Override
    public void periodic() {
        io.periodic();

        io.updateStats();
          
        UpdateTelemetry();
    }

    private void UpdateTelemetry() {
        wristCancoderPosition.setDouble(io.coralCancoder.getAbsolutePosition().getValueAsDouble());
        CANdiPWM1.setBoolean(stats.candiPWM1);
        wristPosition.setDouble(io.coralWrist.getPosition().getValueAsDouble());


        CANdiPWM3.setBoolean(stats.candiPWM3);
    }

    private FunctionalCommand coralCommand(double position){
        return new FunctionalCommand(
            () -> this.wristCommandedPos.setDouble(position/360.0),
            () -> 
            {io.setCoralAngleMotorControl(position);}
            ,
            interrupted -> {io.setCoralAngleMotorControl(position);
            }, 
            () -> stats.wristCancoderPosition <= (position/360.0) + .01 && stats.wristCancoderPosition >= (position/360.0) - .01,
            this
        );
    }

    public Command shoot() {
        return runOnce(() -> {
            io.setCoralSpinMotorControl(-1.5);
            io.setCoralAngleMotorControl(ManipulatorConstants.coralWristHP);
        });
    }

    public Command intake() {
        return run(() -> {
            io.setCoralSpinMotorControl(8);
            io.setCoralAngleMotorControl(ManipulatorConstants.coralWristHP);

        }).onlyWhile(() -> stats.candiPWM1).andThen(Commands.waitSeconds(0.4)).andThen(zero());    
    }

    public Command intakeGround() {
        return run(() -> {
            io.setCoralSpinMotorControl(8);
            io.setCoralAngleMotorControl(ManipulatorConstants.coralWristScore);

        });    
    }

    public Command autonIntake() {
        return run(() -> {
            io.setCoralSpinMotorControl(8);
            io.setCoralAngleMotorControl(ManipulatorConstants.coralWristHP);

        }).onlyWhile(() -> stats.candiPWM1).andThen(Commands.waitSeconds(0.2)).andThen(zero());    
    }

    public Command L4Spit(){
        return runOnce(() -> io.setCoralSpinMotorControl(-5));
    }

    public Command knockAlgaeSlowRight(){
        return runOnce(() -> io.setCoralSpinMotorControl(-8));
    }

    public Command knockAlgaeSlowLeft(){
        return runOnce(() -> io.setCoralSpinMotorControl(4));
    }

    public Command knockAlgaeLeft(){
        return runOnce(() -> io.setCoralSpinMotorControl(9));
    }

    public Command L3Spit(){
        return runOnce(() -> io.setCoralSpinMotorControl(-2));
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

        
            public boolean checkCoralRange(double deadband){
        return (stats.wristPosition >= wristCommandedPos.getDouble(0) - deadband) && 
               (stats.wristPosition <= wristCommandedPos.getDouble(0) + deadband);
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