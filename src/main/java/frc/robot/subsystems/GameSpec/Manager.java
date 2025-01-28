package frc.robot.subsystems.GameSpec;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.GameSpec.Algae.Algae;
import frc.robot.subsystems.GameSpec.Algae.AlgaeIOReal;
import frc.robot.subsystems.GameSpec.Algae.AlgaeIOSim;
import frc.robot.subsystems.GameSpec.Arm.Arm;
import frc.robot.subsystems.GameSpec.Arm.ArmIOReal;
import frc.robot.subsystems.GameSpec.Arm.ArmIOSim;
import frc.robot.subsystems.GameSpec.Climber.Climber;
import frc.robot.subsystems.GameSpec.Climber.ClimberIOReal;
import frc.robot.subsystems.GameSpec.Climber.ClimberIOSim;
import frc.robot.subsystems.GameSpec.Coral.Coral;
import frc.robot.subsystems.GameSpec.Coral.CoralIOReal;
import frc.robot.subsystems.GameSpec.Coral.CoralIOSim;
import frc.robot.subsystems.GameSpec.Elevator.Elevator;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOReal;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOSim;

public class Manager extends SubsystemBase{
    private Arm armSubsystem;
    private Elevator elevatorSubsystem;
    private Coral coralSubsystem;
    private Algae algaeSubsystem;
    private Climber climberSubsystem;
    
    public Manager() {
      if (!Utils.isSimulation()){
        elevatorSubsystem = new Elevator(new ElevatorIOReal());   
        armSubsystem = new Arm(new ArmIOReal());
        coralSubsystem = new Coral(new CoralIOReal());
        algaeSubsystem = new Algae(new AlgaeIOReal());
        climberSubsystem = new Climber(new ClimberIOReal());
      } else {
        elevatorSubsystem = new Elevator(new ElevatorIOSim());
        armSubsystem = new Arm(new ArmIOSim());
        coralSubsystem = new Coral(new CoralIOSim());
        algaeSubsystem = new Algae(new AlgaeIOSim());
        climberSubsystem = new Climber(new ClimberIOSim());
      }
    }

    public Command L1(){
      return Commands.parallel(
        elevatorSubsystem.goToL1(),
        armSubsystem.goToCL1()
      );
    }

    public Command L2(){
      System.out.println("L2");
      return Commands.parallel(elevatorSubsystem.goToL4());
    }

    public Command L3(){
      System.out.println("L3");
      return Commands.parallel(elevatorSubsystem.goToL4());
    }

    public Command L4(){
      System.out.println("L4");

      return Commands.parallel(elevatorSubsystem.goToL1(), armSubsystem.goToCL1())
      .until(() -> (armSubsystem.armPosition(0.3, 0.01)))
      .andThen(
        Commands.parallel(elevatorSubsystem.goToL4(), armSubsystem.goToCL4())
      );
    }

    public Command HP(){
      System.out.println("HP");
      return Commands.parallel(elevatorSubsystem.goToL4());
    }

    public Command Barge(){
      System.out.println("Barge");
      return Commands.parallel(elevatorSubsystem.goToL4());
    }

    public Command Package(){
        System.out.println("Package");
        return Commands.parallel(elevatorSubsystem.goToL4());
        
    }

    // public Command L1(){
    //   return Commands.parallel(
    //     elevatorSubsystem.goToL1(),
    //     Commands.waitUntil(() -> elevatorSubsystem.checkRange(.1)).andThen(armSubsystem.goToCL1())
    //   );
    // }
    


}

