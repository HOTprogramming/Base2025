package frc.robot.subsystems.GameSpec;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
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
    private final ShuffleboardTab managerShuffleboard;
    private Arm armSubsystem;
    private Elevator elevatorSubsystem;
    public Climber climberSubsystem;
    private Intake intakeSubsystem;
    private Manipulator manipulatorSubsystem;

    public GenericEntry scoringEnum;

    private enum ScoringLevel {
      L1,
      L2,
      L3,
      L4
    }

    private ScoringLevel scoringLevel;
  
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

      scoringLevel = ScoringLevel.L1;

      this.managerShuffleboard = Shuffleboard.getTab("Manager");

      scoringEnum = this.managerShuffleboard.add("manager enum", 0.0).getEntry();
    }

    private void UpdateTelemetry() {
      scoringEnum.setString(scoringLevel.name());
    }

    public Command goToPackage(){
      return Commands.parallel(armSubsystem.goToPackage()).until(() -> (armSubsystem.armGreaterThan(ArmConstants.Intermediate,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToPackage(), Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore())));
    }

    public Command goToL1(){
      return Commands.parallel(Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L1;})
      ,elevatorSubsystem.goToL1().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      , armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L1Height-30.0,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToL1(), armSubsystem.goToL1()), manipulatorSubsystem.goHP()))
      .onlyIf(() -> !manipulatorSubsystem.returnBeambreak());
    }

    public Command goToL2(){
      return Commands.parallel(Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L2;})
      ,elevatorSubsystem.goToL2().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      , armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L2Height-30.0,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToL2(), armSubsystem.goToL2())))
      .onlyIf(() -> !manipulatorSubsystem.returnBeambreak());
    }

    public Command goToL3(){
      return Commands.parallel(Commands.parallel(      
      run(() -> {scoringLevel = ScoringLevel.L3;})
      ,elevatorSubsystem.goToL3().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      , armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L3Height-30.0,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToL3(), armSubsystem.goToL3())))
      .onlyIf(() -> !manipulatorSubsystem.returnBeambreak());
    }

    public Command goToL4(){
      return Commands.parallel(Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L4;})
      ,elevatorSubsystem.goToL4().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      ,armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L4Height-30.0,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToL4(), armSubsystem.goToL4())))
      .onlyIf(() -> !manipulatorSubsystem.returnBeambreak());
    }

    public ScoringLevel getLevel(){
      return scoringLevel;
    }

    @SuppressWarnings({ "unchecked", "rawtypes" })
    public Command shoot(){

      return new SelectCommand(
        Map.of(
          ScoringLevel.L4, Commands.sequence(
            armSubsystem.L4Score(),
            elevatorSubsystem.L4Score())
            .onlyWhile(() -> (!armAbort()))
            .andThen(goToL4().onlyIf(() -> armAbort())),
          ScoringLevel.L3, Commands.sequence(
            armSubsystem.L3Score(),
            elevatorSubsystem.L3Score())
            .onlyWhile(() -> (!armAbort()))
            .andThen(goToL3().onlyIf(() -> (armAbort()))),
          ScoringLevel.L2, Commands.sequence(
            armSubsystem.L2Score(),
            elevatorSubsystem.L2Score())
            .onlyWhile(() -> (!armAbort()))
            .andThen(goToL2().onlyIf(() -> (armAbort()))),
          ScoringLevel.L1, Commands.sequence(
            manipulatorSubsystem.shoot()
            .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
            .andThen(goToL1().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail)))))

          // ScoringLevel.L1, Commands.sequence(
          //   manipulatorSubsystem.shoot().onlyIf(() -> !manipulatorSubsystem.returnBeambreak()),
          //   Commands.waitSeconds(0.5),
          //   manipulatorSubsystem.stop())
          //   .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
          //   .andThen(goToL1().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail))))
        ),
        this::getLevel
      );
    }

    @SuppressWarnings({ "unchecked", "rawtypes" })
    public Command cancelShoot(){

      return new SelectCommand(
        Map.of(
          ScoringLevel.L4, goToL4(),
          ScoringLevel.L3, goToL3(),
          ScoringLevel.L2, goToL2(),
          ScoringLevel.L1, Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goHP())
          ),
        this::getLevel
      );
    }

    public Command goToFeeder(){
      return Commands.sequence(
        elevatorSubsystem.goToFeeder(),
        armSubsystem.goToFeeder()
      );
    }

    public Command alignStationIntake(){
      return Commands.parallel(
        Commands.deadline(manipulatorSubsystem.intake(), armSubsystem.goToFeeder())
        .andThen(armSubsystem.goToPackage()), 
        elevatorSubsystem.goToPackage());
    }

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

    // /**
    //  * @return true if the arm should run normally, false if it should stop because it has coral and the arm overruns
    //  */
    // private boolean armAbort() {
    //   return armSubsystem.armCurrent(ArmConstants.CurrentFail) || manipulatorSubsystem.returnBeambreak();
    // }

    /**
     * @return true if arm should abort
     */
    private boolean armAbort() {
      return !armSubsystem.armCurrent(ArmConstants.CurrentFail) || manipulatorSubsystem.returnBeambreak();
    }
}