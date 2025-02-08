package frc.robot.subsystems.GameSpec;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.GameSpec.Arm.Arm;
import frc.robot.subsystems.GameSpec.Arm.ArmConstants;
import frc.robot.subsystems.GameSpec.Arm.ArmIOReal;
import frc.robot.subsystems.GameSpec.Arm.ArmIOSim;
import frc.robot.subsystems.GameSpec.Climber.Climber;
import frc.robot.subsystems.GameSpec.Climber.ClimberIOReal;
import frc.robot.subsystems.GameSpec.Climber.ClimberIOSim;
import frc.robot.subsystems.GameSpec.Elevator.Elevator;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorConstants;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOReal;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOSim;
import frc.robot.subsystems.GameSpec.Intake.Intake;
import frc.robot.subsystems.GameSpec.Intake.IntakeIOReal;
import frc.robot.subsystems.GameSpec.Intake.IntakeIOSim;
import frc.robot.subsystems.GameSpec.Manipulator.Manipulator;
import frc.robot.subsystems.GameSpec.Manipulator.ManipulatorIOReal;
import frc.robot.subsystems.GameSpec.Manipulator.ManipulatorIOSim;

public class Manager extends SubsystemBase{
    private Arm armSubsystem;
    private Elevator elevatorSubsystem;
    public Climber climberSubsystem;
    private Intake intakeSubsystem;
    private Manipulator manipulatorSubsystem;
  
    public Manager() {
      if (!Utils.isSimulation()){
        elevatorSubsystem = new Elevator(new ElevatorIOReal());   
        armSubsystem = new Arm(new ArmIOReal());
        manipulatorSubsystem = new Manipulator(new ManipulatorIOReal());
        climberSubsystem = new Climber(new ClimberIOReal());
        intakeSubsystem = new Intake(new IntakeIOReal());
      } else {
        elevatorSubsystem = new Elevator(new ElevatorIOSim());
        armSubsystem = new Arm(new ArmIOSim());
        manipulatorSubsystem = new Manipulator(new ManipulatorIOSim());
        climberSubsystem = new Climber(new ClimberIOSim());
      }
    }

    public Command goToPackage(){
      return Commands.parallel(armSubsystem.goToPackage()).until(() -> (armSubsystem.armGreaterThan(ArmConstants.Intermediate,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToPackage()));
    }

    public Command goToL1(){

      return Commands.parallel(armSubsystem.goToL1());

      // return Commands.sequence(
      //   elevatorSubsystem.goToL1(),
      //   armSubsystem.goToL1()
      // );
    }

    public Command goToL2(){

      return Commands.parallel(armSubsystem.goToL2());

      // return Commands.sequence(
      //   elevatorSubsystem.goToL2(),
      //   armSubsystem.goToL2()
      // );
    }

    public Command goToL3(){
      return Commands.sequence(
        // elevatorSubsystem.goToL1(),
        // armSubsystem.goToL3()
        //elevatorSubsystem.goToPackage(),
        armSubsystem.positiveTest()
      );
    }

    public Command goToL4(){
      return Commands.parallel(elevatorSubsystem.goToL4().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0))), armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L4Height-30.0,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToL4(), armSubsystem.goToL4()));
    }

    public Command L4Score(){
      return Commands.parallel(elevatorSubsystem.goToL4());
      // return Commands.sequence(
      //   armSubsystem.L4Score(),
      //   elevatorSubsystem.L4Score()
      // );
    }

    public Command goToFeeder(){
      return Commands.sequence(
        elevatorSubsystem.goToFeeder(),
        armSubsystem.goToFeeder()
      );
    }

    public Command alignStationIntake(){
      return Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToFeeder(), manipulatorSubsystem.intake());
    }

    //elevator position: 17.0
    //arm position: 52.4

    public Command coralGoHP(){
      return manipulatorSubsystem.goHP();
    }

    public Command coralGoScore(){
      return manipulatorSubsystem.goScore();
    }

    public Command algaeExtend(){
      return manipulatorSubsystem.algaeExtend();
    }

    public Command algaePackage(){
      return manipulatorSubsystem.algaePackage();
    }
   
    public Command Intake(){
      return manipulatorSubsystem.intake();
    }
    
    public Command StopIntake(){
      return manipulatorSubsystem.zero();
    }

}