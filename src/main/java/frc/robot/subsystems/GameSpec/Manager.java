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
import frc.robot.subsystems.Lights.Lights;

public class Manager extends SubsystemBase{
    private Arm armSubsystem;
    private Elevator elevatorSubsystem;
    public Climber climberSubsystem;
    private Intake intakeSubsystem;
    private Manipulator manipulatorSubsystem;
    private Lights lightsSubsystem;

    public GenericEntry scoringEnum;

    private enum ScoringLevel {
      L1,
      L2,
      L3,
      L4,
      Algae,
      Barge
    }

    private enum AlgaeIntakeEnum{
      floor,
      pluck
    }

    public boolean doneScoring = false;

    private ScoringLevel scoringLevel;
    private AlgaeIntakeEnum algaeIntakeEnum;
  
    public Manager() {
      if (!Utils.isSimulation()){
        elevatorSubsystem = new Elevator(new ElevatorIOReal());   
        armSubsystem = new Arm(new ArmIOReal());
        manipulatorSubsystem = new Manipulator(new ManipulatorIOReal());
        climberSubsystem = new Climber(new ClimberIOReal());
        intakeSubsystem = new Intake(new IntakeIOReal());
        lightsSubsystem = new Lights();
      } else {
        elevatorSubsystem = new Elevator(new ElevatorIOSim());
        armSubsystem = new Arm(new ArmIOSim());
        manipulatorSubsystem = new Manipulator(new ManipulatorIOSim());
        climberSubsystem = new Climber(new ClimberIOSim());
        lightsSubsystem = new Lights();
        intakeSubsystem = new Intake(new IntakeIOSim());

      } 

      scoringLevel = ScoringLevel.L1;
      algaeIntakeEnum = AlgaeIntakeEnum.pluck;

    }


    public Command goToPackage(){
      return Commands.parallel(armSubsystem.goToPackage()).until(() -> (armSubsystem.armGreaterThan(ArmConstants.Intermediate,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToPackage(), Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore())));
    }

    public Command goToL2Package(){
      return Commands.parallel(elevatorSubsystem.goToPackage())
      .andThen(Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToPackage(), Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore())));
    }

    public Command gotoL4Package(){
      return Commands.parallel(
        elevatorSubsystem.goToPackage(),
        armSubsystem.L4Score()
      ).until(() -> !elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L4Height-6.0, 0.1))
      .andThen(Commands.parallel(
        armSubsystem.goToPackage(),
        elevatorSubsystem.goToPackage()
      ));
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
      return Commands.parallel(
      Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L1;})
      ,elevatorSubsystem.goToL1().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      ,armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L1Height-30.0,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToL1(), armSubsystem.goToL1()), manipulatorSubsystem.goHP()))
      .onlyIf(() -> checkDoneScoring());
    }

    public Command goToL2(){
      return Commands.parallel(
      manipulatorSubsystem.goScore().withTimeout(0.1)
      ,Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L2;})
      ,elevatorSubsystem.goToL2().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      ,armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L2Height-30.0,2.0)))
      .andThen(Commands.parallel(Commands.sequence(elevatorSubsystem.goToL2()
      .onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), elevatorSubsystem.goToL2Long()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak()
      )), 
      Commands.sequence(armSubsystem.goToL2()
      .onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), armSubsystem.goToL2Short()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak()
      )))
      .onlyIf(() -> checkDoneScoring());
    }

    public Command goToL3(){
      return Commands.parallel(
      manipulatorSubsystem.goScore().withTimeout(0.1)
      ,Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L3;})
      ,elevatorSubsystem.goToL3().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      ,armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L3Height-30.0,2.0)))
      .andThen(Commands.parallel(Commands.sequence(elevatorSubsystem.goToL3()
      .onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), elevatorSubsystem.goToL3Long()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak()
      )), 
      Commands.sequence(armSubsystem.goToL3()
      .onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), armSubsystem.goToL3Short()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak()
      )))
      .onlyIf(() -> checkDoneScoring());
    }

    public Command goToL4(){
      return Commands.parallel(
      manipulatorSubsystem.goScore().withTimeout(0.1)
      ,Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L4;})
      ,elevatorSubsystem.goToL4().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      ,armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L4Height-20.0,2.0)))
      .andThen(Commands.parallel(Commands.sequence(elevatorSubsystem.goToL4()
      .onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), elevatorSubsystem.goToL4Long()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak()
      )), 
      Commands.sequence(armSubsystem.goToL4()
      .onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), armSubsystem.goToL4Short()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak()
      )))
      .onlyIf(() -> checkDoneScoring());
    }

    public Command highAlgae(){
      return Commands.sequence(
        runOnce(() -> {algaeIntakeEnum = AlgaeIntakeEnum.pluck;})
        ,Commands.parallel(elevatorSubsystem.goToHighAlgae()
        ,armSubsystem.getAlgaeFromReef()
        ,
        Commands.sequence(
        manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeIntakeVoltage)
        .onlyWhile(() -> manipulatorSubsystem.returnAlgaeIn())
        ,manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeHoldVoltage)
        .onlyWhile(() -> !manipulatorSubsystem.returnAlgaeIn()))
      )
      );
    }

    public Command lowAlgae(){
      return Commands.sequence(
        runOnce(() -> {algaeIntakeEnum = AlgaeIntakeEnum.pluck;})
        ,Commands.parallel(elevatorSubsystem.goToLowAlgae()
        ,armSubsystem.getAlgaeFromReef()
        ,
        Commands.sequence(
        manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeIntakeVoltage)
        .onlyWhile(() -> manipulatorSubsystem.returnAlgaeIn())
        ,manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeHoldVoltage)
        .onlyWhile(() -> !manipulatorSubsystem.returnAlgaeIn()))
      )
      );
    }

    public ScoringLevel getLevel(){
      return scoringLevel;
    }

    public AlgaeIntakeEnum getAlgaeIntakeEnum(){
      return algaeIntakeEnum;
    }

    @SuppressWarnings({ "unchecked", "rawtypes" })
    public Command shoot(){

      return new SelectCommand(
        Map.of(
          ScoringLevel.L4, Commands.sequence(
            runOnce(() -> {doneScoring = true;}),
            armSubsystem.L4Score().withTimeout(0.75),
            Commands.parallel(
            manipulatorSubsystem.L4Spit(),
            gotoL4Package()),
            manipulatorSubsystem.zero()
            ),
          ScoringLevel.L3, Commands.sequence(
            runOnce(() -> {doneScoring = true;}),
            Commands.sequence(
            armSubsystem.L3Score()
            ,elevatorSubsystem.L3Score()
            ,manipulatorSubsystem.goScore().withTimeout(0.1))
            .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
            .andThen(goToL3().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail)))),
            Commands.sequence(Commands.parallel(goToPackage().onlyIf(() -> (manipulatorSubsystem.returnBeamBreak())))),    
            goToL3().onlyIf(() -> (!manipulatorSubsystem.returnBeamBreak()))),
          ScoringLevel.L2, Commands.sequence(
            runOnce(() -> {doneScoring = true;}),
            Commands.sequence(
            armSubsystem.L2Score()
            ,elevatorSubsystem.L2Score()
            ,manipulatorSubsystem.goScore().withTimeout(0.1))
            .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
            .andThen(goToL2().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail)))),
            Commands.sequence(Commands.parallel(Commands.parallel(armSubsystem.L2Score(), elevatorSubsystem.L2Score()).onlyIf(() -> (manipulatorSubsystem.returnBeamBreak())))),    
            goToL2().onlyIf(() -> (!manipulatorSubsystem.returnBeamBreak()))),
          ScoringLevel.L1, Commands.sequence(
            manipulatorSubsystem.shoot()
            .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
            .andThen(goToL1().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail))))),
          ScoringLevel.Algae, manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeExpelVoltage),
          ScoringLevel.Barge, manipulatorSubsystem.algaeVoltage(16.0)
        ),
        this::getLevel
      );
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public Command autonShoot(){
      return new SelectCommand(
        Map.of(
            ScoringLevel.L4, Commands.parallel(
            armSubsystem.L4Score(), elevatorSubsystem.L4MiniScore().onlyWhile(() -> armSubsystem.armLessThan(-30.0, 0))
          ),
            ScoringLevel.L3, Commands.sequence(
            Commands.parallel(armSubsystem.L3Score(), manipulatorSubsystem.goScore()), elevatorSubsystem.L3Score()
          )
          // .onlyWhile(() -> elevatorSubsystem.elevatorGreaterThan(elevator working pose, 0))
          //shut up im coding
        ),
        this::getLevel
      );
    }

    public Command autonL4() {
      return Commands.sequence(
        Commands.parallel(runOnce(() -> {scoringLevel = ScoringLevel.L4;}), elevatorSubsystem.goToL4()), 
        manipulatorSubsystem.goScore().withTimeout(0.3)
       );
    }

    public Command autonIntake() {
      return Commands.parallel(Commands.deadline(manipulatorSubsystem.autonIntake(), armSubsystem.goToFeeder()), elevatorSubsystem.goToFeeder());
    }

    public Command autonFinishIntake() {
      return armSubsystem.goToPackage();
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

    public Command climberOut(){
      return Commands.sequence(
      lockFingers()
      ,elevatorSubsystem.goToFloorIntake()
      ,armSubsystem.horizontal()
      //,intakeSubsystem.intakeClimberOut()
      ,unlatchServo()
      ,Commands.waitSeconds(0.2)
      ,climberDeploy()
      ,latchServo()
      ,elevatorSubsystem.climbDown()
      //,intakeSubsystem.intakeVert()
      );
    }

    public Command climberDeploy(){
    return run(() -> climberSubsystem.setPower(4.0))
      .onlyWhile(() -> climberSubsystem.checkClimberDeployed())
      .andThen(runOnce(() -> climberSubsystem.setPower(0.0)));
    }

    public Command unlatchServo(){
      return climberSubsystem.ratchetServoPositionClimber(0.38, -3.0)
      .andThen(Commands.waitSeconds(0.2));
    }

    public Command latchServo(){
      return climberSubsystem.ratchetServoPosition(0.59)
      .andThen(Commands.waitSeconds(0.2));
    }

    public Command testRatchetServoIn2(){
      return climberSubsystem.ratchetServoPosition(0.59);
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
      ,Commands.parallel(armSubsystem.goToPackage()
      ,elevatorSubsystem.goToPackage()));
    }

    public Command bargePackage(){
      return Commands.parallel(
        elevatorSubsystem.goToPackage(),
        armSubsystem.goToPackage(),
        cancelAlgaeHolding());
    }

    public Command algaePackage(){
      return Commands.parallel(armSubsystem.goToPackage(), cancelAlgaeHolding());
    }

    //does the normal package like in coral mode, but stops the holding voltage for the algae instead of moving the wrist.
    public Command algaePackageElevator(){
      return Commands.parallel(armSubsystem.goToPackage()).until(() -> (armSubsystem.armGreaterThan(ArmConstants.Intermediate,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToPackage(), cancelAlgaeHolding()));
    }

    public Command cancelAlgaeHolding(){
      return manipulatorSubsystem.algaeVoltage(0.0).onlyIf(() -> manipulatorSubsystem.returnAlgaeIn());
    }

    public Command alignProcessor(){
      return Commands.sequence(runOnce(() -> {scoringLevel = ScoringLevel.Algae;}),
        Commands.parallel(elevatorSubsystem.goToProcessor())
        ,Commands.parallel(armSubsystem.processor()));
    }

    public Command barge(){
      return Commands.parallel(
        Commands.parallel(elevatorSubsystem.goToBarge(), runOnce(() -> {scoringLevel = ScoringLevel.Barge;}))
        ,armSubsystem.barge());
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public Command algaeIntake(){

      return new SelectCommand(
        Map.of(
          AlgaeIntakeEnum.pluck,
            manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeIntakeVoltage)
            .until(() -> manipulatorSubsystem.returnAlgaeIn())
            .andThen(manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeHoldVoltage))
            ,
          AlgaeIntakeEnum.floor, Commands.parallel(
           manipulatorSubsystem.algaeVoltage(ManipulatorConstants.algaeIntakeVoltage)
          )
          ),
        this::getAlgaeIntakeEnum
      );
    }

    public Command algaeStopIntake(){
      return Commands.parallel(
      manipulatorSubsystem.algaeVoltage(0.0)
      ,intakeSubsystem.intakeRollerVoltage(0.0));

    }

    public Command setLightsCoral() {
      return lightsSubsystem.setYellow();
    }
    
    public Command setLightsShoot(){
      return lightsSubsystem.setPurple();
    }

    public Command setLightsAlgae() {
      return lightsSubsystem.setTeal();
    }

    public Command setLightsClimb() {
      return lightsSubsystem.setRed();
    }

    public Command setLightsGood() {
      return lightsSubsystem.setGreen();
    }

    public Command setLightsOk() {
      return lightsSubsystem.setYellow();
    }

    public Command setLightsBad() {
      return lightsSubsystem.setRed();
    }

    public Command setLightsAlign() {
      return lightsSubsystem.setGreen();
    }

    // public Command setLightsOff() {
    //   return lightsSubsystem.set
    // }
}