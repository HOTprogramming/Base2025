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



    public Command L1(){
      return run(() -> {
        System.out.println("L1");
        });
    }

    public Command L2(){
      return run(() -> {
          System.out.println("L2");
        });
    }

    public Command L3(){
      return run(() -> {
        System.out.println("L3");
        });
    }

    public Command L4(){
      return run(() -> {
        System.out.println("L4");
        });
    }

    public Command HP(){
      return run(() -> {
        System.out.println("HP");
        });
    }

    public Command Barge(){
      return run(() -> {
        System.out.println("Barge");
        });
    }

    public Command Package(){
      return run(() -> {
        System.out.println("Package");
        });
    }

    // public Command place(){
    //     return run(() -> {
    //         elevatorSubsystem.managerElevatorTest();

    //         if(elevatorSubsystem.elevatorPosition.getDouble(0) > 1){
    //           armSubsystem.managerArmTest();
    //         }
    //       }).until(() -> elevatorSubsystem.checkRange(.1) && armSubsystem.checkRange(.1));
    // }

}

