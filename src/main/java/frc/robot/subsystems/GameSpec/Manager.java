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
import frc.robot.subsystems.GameSpec.Intake.IntakeConstants;
import frc.robot.subsystems.GameSpec.Intake.IntakeIOReal;
import frc.robot.subsystems.GameSpec.Intake.IntakeIOSim;
import frc.robot.subsystems.GameSpec.Manipulator.Manipulator;
import frc.robot.subsystems.GameSpec.Manipulator.ManipulatorConstants;
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
      L4,
      Algae
    }

    private enum PackageMode{
      climbed,
      notClimbed,
      algaed
    }

    private enum AlgaeIntakeEnum{
      floor,
      pluck
    }

    public boolean doneScoring = false;

    private ScoringLevel scoringLevel;
    private PackageMode packageMode;
    private AlgaeIntakeEnum algaeIntakeEnum;
  
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
      packageMode = PackageMode.notClimbed;
      algaeIntakeEnum = AlgaeIntakeEnum.pluck;

      this.managerShuffleboard = Shuffleboard.getTab("Manager");

      scoringEnum = this.managerShuffleboard.add("manager enum", 0.0).getEntry();
    }

    private void UpdateTelemetry() {
      scoringEnum.setString(scoringLevel.name());
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public Command goToPackage(){

      return new SelectCommand(
        Map.of(
          PackageMode.notClimbed, Commands.parallel(armSubsystem.goToPackage()).until(() -> (armSubsystem.armGreaterThan(ArmConstants.Intermediate,2.0)))
          .andThen(Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToPackage(), Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore()))),
          PackageMode.climbed, Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToPackage())
          ),
        this::getPackageMode
      );
    }

    public Command goToL2Package(){
      return Commands.parallel(elevatorSubsystem.goToPackage())
      .andThen(Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToPackage(), Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore())));
    }

    public Command doneScoring(){
      return runOnce(() -> doneScoring = false);
    }

    public boolean checkDoneScoring(){
      if(doneScoring == false){
        return true;
      }
      else{
        return false;
      }
    }

    public Command goToL1(){
      return Commands.parallel(Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L1;})
      ,elevatorSubsystem.goToL1().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      , armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L1Height-30.0,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToL1(), armSubsystem.goToL1()), manipulatorSubsystem.goHP()))
      .onlyIf(() -> checkDoneScoring());
    }

    public Command goToL2(){
      return Commands.parallel(Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L2;})
      ,elevatorSubsystem.goToL2().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      , armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L2Height-30.0,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToL2(), Commands.sequence(armSubsystem.goToL2().onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), armSubsystem.goToL2Short()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak()))))
      .onlyIf(() -> checkDoneScoring());
    }

    public Command goToL3(){
      return Commands.parallel(Commands.parallel(      
      run(() -> {scoringLevel = ScoringLevel.L3;})
      ,elevatorSubsystem.goToL3().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      , armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L3Height-30.0,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToL3(), Commands.sequence(armSubsystem.goToL3().onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), armSubsystem.goToL3Short()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak()))))
      .onlyIf(() -> checkDoneScoring());
    }

    public Command goToL4(){
      return Commands.parallel(Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L4;})
      ,elevatorSubsystem.goToL4().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      ,armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L4Height-30.0,2.0)))
      .andThen(Commands.parallel(Commands.sequence(elevatorSubsystem.goToL4().onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), elevatorSubsystem.goToL4Long()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak())), Commands.sequence(armSubsystem.goToL4().onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), armSubsystem.goToL4Short()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak())))
      .onlyIf(() -> checkDoneScoring());
    }

    public Command highAlgae(){
      return Commands.sequence(
        runOnce(() -> {algaeIntakeEnum = AlgaeIntakeEnum.pluck;})
        ,Commands.parallel(elevatorSubsystem.goToHighAlgae(), armSubsystem.getAlgaeFromReef())
      );
    }

    public Command lowAlgae(){
      return Commands.sequence(
        runOnce(() -> {algaeIntakeEnum = AlgaeIntakeEnum.pluck;})
        ,Commands.parallel(elevatorSubsystem.goToLowAlgae(), armSubsystem.getAlgaeFromReef())
      );
    }

    public ScoringLevel getLevel(){
      return scoringLevel;
    }

    public PackageMode getPackageMode(){
      return packageMode;
    }

    public AlgaeIntakeEnum getAlgaeIntakeEnum(){
      return algaeIntakeEnum;
    }

    @SuppressWarnings({ "unchecked", "rawtypes" })
    public Command shoot(){

      return new SelectCommand(
        Map.of(
          ScoringLevel.L4, Commands.sequence(Commands.sequence(
            armSubsystem.L4Score(),
            elevatorSubsystem.L4Score())
            .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
            .andThen(goToL4().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail)))),
            Commands.sequence(Commands.parallel(goToPackage().onlyIf(() -> (manipulatorSubsystem.returnBeamBreak()))
            ,(run(() -> {doneScoring = true;}).onlyIf(() -> (manipulatorSubsystem.returnBeamBreak()))))),    
            goToL4().onlyIf(() -> (!manipulatorSubsystem.returnBeamBreak()))),
          ScoringLevel.L3, Commands.sequence(Commands.sequence(
            armSubsystem.L3Score(),
            elevatorSubsystem.L3Score())
            .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
            .andThen(goToL3().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail)))),
            Commands.sequence(Commands.parallel(goToPackage().onlyIf(() -> (manipulatorSubsystem.returnBeamBreak()))
            ,(run(() -> {doneScoring = true;}).onlyIf(() -> (manipulatorSubsystem.returnBeamBreak()))))),    
            goToL3().onlyIf(() -> (!manipulatorSubsystem.returnBeamBreak()))),
          ScoringLevel.L2, Commands.sequence(Commands.sequence(
            armSubsystem.L2Score(),
            elevatorSubsystem.L2Score())
            .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
            .andThen(goToL2().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail)))),
            Commands.sequence(Commands.parallel(Commands.parallel(armSubsystem.L2Score(), elevatorSubsystem.L2Score()).onlyIf(() -> (manipulatorSubsystem.returnBeamBreak()))
            ,(run(() -> {doneScoring = true;}).onlyIf(() -> (manipulatorSubsystem.returnBeamBreak()))))),    
            goToL2().onlyIf(() -> (!manipulatorSubsystem.returnBeamBreak()))),
          ScoringLevel.L1, Commands.sequence(
            manipulatorSubsystem.shoot()
            .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
            .andThen(goToL1().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail))))),
          ScoringLevel.Algae, manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeExpelVoltage)
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
          ScoringLevel.L1, Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goHP()),
          ScoringLevel.Algae, manipulatorSubsystem.algaeVoltage(0.0)
          ),
        this::getLevel
      );
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public Command autonShoot(){
      return new SelectCommand(
        Map.of(
          ScoringLevel.L4, Commands.sequence(
            armSubsystem.L4Score(), elevatorSubsystem.L4Score()
          ),
            ScoringLevel.L3, Commands.sequence(
            armSubsystem.L3Score(), elevatorSubsystem.L3Score()
          )
        ),
        this::getLevel
      );
    }


    public Command alignStationIntake(){
      return Commands.parallel(
        Commands.deadline(manipulatorSubsystem.intake(), armSubsystem.goToFeeder())
        .andThen(armSubsystem.goToPackage()), 
        elevatorSubsystem.goToFeeder());
    }

    public Command coralGoHP(){
      return manipulatorSubsystem.goHP();
    }

    public Command coralGoScore(){
      return manipulatorSubsystem.goScore();
    }

    public Command lockFingers(){
      return climberSubsystem.servoLock();
    }    
    public Command OpenFingers(){
      return climberSubsystem.servoOpen();
    }

    //deploys the climber
    public Command climberOut(){
      return Commands.sequence(runOnce(() -> { packageMode = PackageMode.climbed;}), armSubsystem.horizontal(),
      run(() -> climberSubsystem.setPower(3.0)).onlyWhile(() -> climberSubsystem.checkClimberDeployed()).andThen(runOnce(() -> climberSubsystem.setPower(0.0)))
      ,elevatorSubsystem.climbDown());
    }

    //picks up an algae from the ground
    public Command alignFloorIntake(){
      return Commands.sequence(
      runOnce(() -> {algaeIntakeEnum = AlgaeIntakeEnum.floor;})
      ,Commands.parallel(elevatorSubsystem.goToFloorIntake()
      ,armSubsystem.horizontal())
      ,intakeSubsystem.intakeAlgaeGround()
      ,Commands.parallel(armSubsystem.intakeAlgae(), elevatorSubsystem.goToFloorIntakeGrab())
      );
    }

    public Command floorIntakePackage(){
      return Commands.sequence(elevatorSubsystem.goToFloorIntake()
      ,armSubsystem.horizontal()
      ,intakeSubsystem.goToPackage()
      ,Commands.parallel(armSubsystem.goToPackage(), elevatorSubsystem.goToPackage()));
    }

    public Command alignProcessor(){
      return Commands.sequence(runOnce(() -> {scoringLevel = ScoringLevel.Algae;}),
        Commands.parallel(elevatorSubsystem.goToProcessor())
        ,Commands.parallel(armSubsystem.processor()));
    }

    public Command barge(){
      return Commands.sequence(
        Commands.parallel(elevatorSubsystem.goToBarge(), runOnce(() -> {scoringLevel = ScoringLevel.Algae;}))
        ,armSubsystem.horizontal());
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public Command algaeIntake(){

      return new SelectCommand(
        Map.of(
          AlgaeIntakeEnum.pluck, manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeIntakeVoltage),
          AlgaeIntakeEnum.floor, Commands.parallel(
           manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeIntakeVoltage)
          ,intakeSubsystem.intakeRollerVoltage(IntakeConstants.rollerIntakeVoltage))
          ),
        this::getAlgaeIntakeEnum
      );
    }

    public Command algaeStopIntake(){
      return Commands.parallel(
      manipulatorSubsystem.algaeVoltage(0.0)
      ,intakeSubsystem.intakeRollerVoltage(0.0));

    }
}