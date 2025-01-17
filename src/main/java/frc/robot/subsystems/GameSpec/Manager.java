package frc.robot.subsystems.GameSpec;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.GameSpec.Arm.Arm;
import frc.robot.subsystems.GameSpec.Arm.ArmIOReal;
import frc.robot.subsystems.GameSpec.Arm.ArmIOSim;
import frc.robot.subsystems.GameSpec.Elevator.Elevator;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOReal;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOSim;

public class Manager extends SubsystemBase{
    private Arm armSubsystem;
    private Elevator elevatorSubsystem;
    
    public Manager() {
        switch (Constants.getRobot()) {
          case COMPBOT -> {
            elevatorSubsystem = new Elevator(new ElevatorIOReal());   
            armSubsystem = new Arm(new ArmIOReal());
          }
          case DEVBOT -> {}
          case SIMBOT -> {    
            elevatorSubsystem = new Elevator(new ElevatorIOSim());
            armSubsystem = new Arm(new ArmIOSim());
    
          }
        }
      }

    public Command place(){
        return runOnce(() -> {elevatorSubsystem.managerElevatorTest();
            armSubsystem.managerArmTest();});
    }

}

