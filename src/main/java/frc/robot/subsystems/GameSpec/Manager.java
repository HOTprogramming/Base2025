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
import frc.robot.subsystems.GameSpec.Intake.Intake;
import frc.robot.subsystems.GameSpec.Intake.IntakeIOReal;
import frc.robot.subsystems.GameSpec.Intake.IntakeIOSim;

public class Manager extends SubsystemBase{
    private Arm armSubsystem;
    private Elevator elevatorSubsystem;
    private Coral coralSubsystem;
    private Algae algaeSubsystem;
    private Climber climberSubsystem;
    private Intake intakeSubsystem;
    
    public Manager() {
      if (!Utils.isSimulation()){
        // elevatorSubsystem = new Elevator(new ElevatorIOReal());   
        // armSubsystem = new Arm(new ArmIOReal());
        coralSubsystem = new Coral(new CoralIOReal());
        algaeSubsystem = new Algae(new AlgaeIOReal());
        // climberSubsystem = new Climber(new ClimberIOReal());
        // intakeSubsystem = new Intake(new IntakeIOReal());
      } else {
        // elevatorSubsystem = new Elevator(new ElevatorIOSim());
        // armSubsystem = new Arm(new ArmIOSim());
        coralSubsystem = new Coral(new CoralIOSim());
        // algaeSubsystem = new Algae(new AlgaeIOSim());
        // climberSubsystem = new Climber(new ClimberIOSim());
      }
    }

    public Command goToPackage(){
      return Commands.sequence(
        armSubsystem.goToPackage(),
        elevatorSubsystem.goToPackage(),
        coralSubsystem.goHorizontal()
      );
    }

    public Command goToL1(){
      return Commands.sequence(
        elevatorSubsystem.goToL1(),
        armSubsystem.goToL1(),
        coralSubsystem.goHorizontal()
      );
    }

    public Command goToL2(){
      return Commands.sequence(
        elevatorSubsystem.goToL2(),
        armSubsystem.goToL2(),
        coralSubsystem.goHorizontal()
      );
    }

    public Command goToL3(){
      return Commands.sequence(
        elevatorSubsystem.goToL3(),
        armSubsystem.goToL3(),
        coralSubsystem.goHorizontal()
      );
    }

    public Command goToL4(){
      return Commands.sequence(
        elevatorSubsystem.goToL4(),
        armSubsystem.goToL4(),
        coralSubsystem.goHorizontal()
      );
    }

    public Command goToFeeder(){
      return Commands.sequence(
        elevatorSubsystem.goToFeeder(),
        armSubsystem.goToFeeder(),
        coralSubsystem.goVertical()
      );
    }

    public Command coralIntake(){
      return coralSubsystem.intake();
    }

    public Command coralShoot(){
      return coralSubsystem.shoot();
    }

    public Command coralZero(){
      return coralSubsystem.zero();
    }
}

