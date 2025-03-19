package frc.robot.subsystems.GameSpec.Algae;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Algae.AlgaeIO.AlgaeIOStats;
import frc.robot.subsystems.GameSpec.Arm.ArmConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class Algae extends SubsystemBase {

    private final AlgaeIO io;
    public final AlgaeIOStats stats;
    private final ShuffleboardTab algaeShuffleboard;
   

    /* Shuffleboard entries */
    private GenericEntry CANrange;

    
    public Algae(AlgaeIO io) {
        this.io = io;
        this.stats = AlgaeIO.stats;

        this.algaeShuffleboard = Shuffleboard.getTab("Algae");
        CANrange = this.algaeShuffleboard.add("Algae Distance", 0.0).getEntry();

    }

    @Override
    public void periodic() {
        io.periodic();

        io.updateStats();
          
        UpdateTelemetry();
    }

    private void UpdateTelemetry() {

        CANrange.setDouble(stats.algaeDistance);
    }


    /**
     * @return true if algae is farther than the trigger / dont have one
     */
    public Boolean returnAlgaeIn(){
        return stats.algaeDistance > AlgaeConstants.algaeTriggerDistance;
    }


    /**
   * 
   * @return negative voltage if intake, positive voltage if expel
   */
    public Command algaeVoltage(double voltage){
            return runOnce(() -> {
                io.setAlgaeSpinMotorControl(voltage);
            });
    }

    public Command runAlwaysAlgaeVoltage(double voltage){
        return run(() -> {
            io.setAlgaeSpinMotorControl(voltage);
        });
}
     

    @Override
    public void simulationPeriodic() {
        io.periodic(); // Update simulated stats
    }
}