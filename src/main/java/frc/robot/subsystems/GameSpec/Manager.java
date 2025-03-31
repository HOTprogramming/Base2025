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
import frc.robot.subsystems.GameSpec.Algae.Algae;
import frc.robot.subsystems.GameSpec.Algae.AlgaeConstants;
import frc.robot.subsystems.GameSpec.Algae.AlgaeIO;
import frc.robot.subsystems.GameSpec.Algae.AlgaeIOReal;
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
import frc.robot.subsystems.GameSpec.Lights.Lights;
import frc.robot.subsystems.GameSpec.Lights.Lights.AnimationTypes;
import frc.robot.subsystems.GameSpec.Manipulator.Manipulator;
import frc.robot.subsystems.GameSpec.Manipulator.ManipulatorConstants;
import frc.robot.subsystems.GameSpec.Manipulator.ManipulatorIOReal;
import frc.robot.subsystems.GameSpec.Manipulator.ManipulatorIOSim;

public class Manager extends SubsystemBase{
    public Arm armSubsystem;
    public Elevator elevatorSubsystem;
    public Climber climberSubsystem;
    public Intake intakeSubsystem;
    private Manipulator manipulatorSubsystem;
    private Lights lightsSubsystem;
    public Algae algaeSubsystem;

    public GenericEntry scoringEnum;

    private enum ScoringLevel {
      L1,
      L2,
      L3,
      L4,
      Algae,
      Barge
    }

    public boolean doneScoring = false;

    private ScoringLevel scoringLevel;
  
    public Manager() {
      if (!Utils.isSimulation()){
        elevatorSubsystem = new Elevator(new ElevatorIOReal());   
        armSubsystem = new Arm(new ArmIOReal());
        manipulatorSubsystem = new Manipulator(new ManipulatorIOReal());
        climberSubsystem = new Climber(new ClimberIOReal());
        intakeSubsystem = new Intake(new IntakeIOReal());
        lightsSubsystem = new Lights();
        algaeSubsystem = new Algae(new AlgaeIOReal());
      } else {
        elevatorSubsystem = new Elevator(new ElevatorIOSim());
        armSubsystem = new Arm(new ArmIOSim());
        manipulatorSubsystem = new Manipulator(new ManipulatorIOSim());
        climberSubsystem = new Climber(new ClimberIOSim());
        lightsSubsystem = new Lights();
        intakeSubsystem = new Intake(new IntakeIOSim());
        algaeSubsystem = new Algae(new AlgaeIOReal());
      } 

      scoringLevel = ScoringLevel.L1;
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
        armSubsystem.goToPackage(),
        elevatorSubsystem.goToPackage()
      );
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
      manipulatorSubsystem.goScore()
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
      manipulatorSubsystem.goScore()
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
      return Commands.parallel(
        elevatorSubsystem.goToHighAlgae()
        ,armSubsystem.getAlgaeFromReef()
        ,algaeSubsystem.runAlwaysAlgaeVoltage(AlgaeConstants.algaeIntakeVoltage)
        .onlyWhile(() -> algaeSubsystem.returnAlgaeIn())
        );
    }

    public Command lowAlgae(){
      return Commands.parallel(elevatorSubsystem.goToLowAlgae()
        ,armSubsystem.getAlgaeFromReef()
        ,algaeSubsystem.runAlwaysAlgaeVoltage(AlgaeConstants.algaeIntakeVoltage)
        .onlyWhile(() -> algaeSubsystem.returnAlgaeIn())
      );
    }

    public ScoringLevel getLevel(){
      return scoringLevel;
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
          ScoringLevel.L3, 
            Commands.sequence(
            runOnce(() -> {doneScoring = true;}),
            armSubsystem.L3Score(),
            manipulatorSubsystem.L3Spit(),
            elevatorSubsystem.L3Score(),
            goToPackage()
            )
            ,
          ScoringLevel.L2,
            Commands.parallel(
            elevatorSubsystem.goToL2(),
            Commands.sequence(
            runOnce(() -> {doneScoring = true;}),
            armSubsystem.L2Score(),
            manipulatorSubsystem.L3Spit())
            ),
          ScoringLevel.L1, Commands.sequence(
            manipulatorSubsystem.shoot()
            .onlyWhile(() -> (armSubsystem.armCurrent(ArmConstants.CurrentFail)))
            .andThen(goToL1().onlyIf(() -> (!armSubsystem.armCurrent(ArmConstants.CurrentFail))))),
          ScoringLevel.Algae, algaeSubsystem.runAlwaysAlgaeVoltage(AlgaeConstants.algaeExpelVoltage),
          ScoringLevel.Barge, algaeSubsystem.runAlwaysAlgaeVoltage(AlgaeConstants.algaeExpelVoltage)
        ),
        this::getLevel
      );
    }

    /**
     * @apinote full shooting to package
     */
    public Command autonShoot() {
      return 
      Commands.sequence(
            runOnce(() -> {doneScoring = true;}),
            armSubsystem.L4Score().withTimeout(0.75),
            Commands.parallel(
            manipulatorSubsystem.L4Spit(),
            gotoL4Package()),
            manipulatorSubsystem.zero()
            );
    }


    /**
     * @apinote full shooting to floor intake
     */
    public Command autonShootIntake() {
      return 
      Commands.sequence(
            runOnce(() -> {doneScoring = true;}),
            armSubsystem.L4Score().withTimeout(0.75),
            Commands.parallel(
              manipulatorSubsystem.L4Spit()
                .andThen(Commands.waitSeconds(.5))
                .andThen(manipulatorSubsystem.zero())
                .andThen(manipulatorSubsystem.goScore()),
              armSubsystem.horizontal(),
              intakeSubsystem.deploy(),
              elevatorSubsystem.intakeCoral()
            ).until(() -> intakeSubsystem.getBeamBreak())
            );
    }

    /**
     * @apinote Half shoot for fast driving
     */
    public Command autonShootStart() {
      return 
      Commands.sequence(
            runOnce(() -> {doneScoring = true;}),
            armSubsystem.L4Score().withTimeout(0.75)
            );
    }

    /**
     * @apinote finish the half shoot
     */
    public Command autonShootFinish() {
      return 
      Commands.sequence(
            Commands.parallel(
            manipulatorSubsystem.L4Spit(),
            gotoL4Package()),
            manipulatorSubsystem.zero()
            );
    }

    /**
     * @deprecated not used just use autonShoot
     */
    public Command autonFinishShoot() {
      return
      Commands.sequence(
        Commands.parallel(
          manipulatorSubsystem.L4Spit(),
          gotoL4Package()
        ),
        manipulatorSubsystem.zero());
    }

    public Command autonL4(){
      return Commands.parallel(
      manipulatorSubsystem.goScore().withTimeout(0.1)
      ,Commands.parallel(
      run(() -> {scoringLevel = ScoringLevel.L4;})
      ,elevatorSubsystem.autonGoToL4().unless(() -> (armSubsystem.armLessThan(ArmConstants.Intermediate, 2.0)))
      ,armSubsystem.goToPackage())
      .until(() -> (elevatorSubsystem.elevatorGreaterThan(ElevatorConstants.L4Height-20.0,2.0)))
      .andThen(Commands.parallel(Commands.sequence(elevatorSubsystem.autonGoToL4()
      .onlyWhile(() -> manipulatorSubsystem.returnOuterBeamBreak()), elevatorSubsystem.goToL4Long()).onlyWhile(() -> !manipulatorSubsystem.returnOuterBeamBreak()
      ))));
    }

    public Command autonIntake() {
      return 
      Commands.parallel(
        Commands.deadline(
          manipulatorSubsystem.autonIntake(), 
          armSubsystem.goToFeeder(),
          intakeSubsystem.handoff()), 
        elevatorSubsystem.goToFeeder());
    }


    public Command autonFinishIntake() {
      return 
      Commands.parallel(
        armSubsystem.goToPackage(),
        intakeSubsystem.clearance());
    }

    public Command autonHalfL4() {
      return elevatorSubsystem.halfHeight();
    }

    public Command alignStationIntake(){
      return Commands.parallel(
        Commands.deadline(manipulatorSubsystem.intake(), armSubsystem.goToFeeder(), intakeSubsystem.handoff())
        .andThen(armSubsystem.goToPackage(), intakeSubsystem.clearance()), 
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
      return Commands.parallel(
      intakeSubsystem.climb(),
      Commands.sequence(
      lockFingers()
      ,Commands.parallel(
      elevatorSubsystem.initialClimbHeight()
      ,armSubsystem.horizontal())
      ,unlatchServo()
      ,Commands.waitSeconds(0.2)
      ,climberDeploy()
      ,latchServo()
      ,elevatorSubsystem.climbDown()
      ));
    }

    public Command autoPackageClimber(){
      return run(() -> climberSubsystem.setPower(-2.0))
      .onlyWhile(() -> climberSubsystem.checkClimberClimbed())
      .andThen(runOnce(() -> climberSubsystem.setPower(0.0)));
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

  //handoff code for teleop
  public Command floorIntakeDeploy(){
    return Commands.sequence(
    elevatorSubsystem.goToPackage(),
    Commands.parallel(
        armSubsystem.horizontal(),
        intakeSubsystem.deploy(),
        manipulatorSubsystem.goScore(),
        elevatorSubsystem.intakeCoral()
        )
      .until(() -> intakeSubsystem.getBeamBreak())
      .andThen(Commands.sequence(
      Commands.waitSeconds(0.2),
      Commands.parallel(
        armSubsystem.horizontal(),
        Commands.sequence(Commands.waitSeconds(0.02), intakeSubsystem.handoffAndSpin()),
        manipulatorSubsystem.intakeGround(),
        elevatorSubsystem.intakeCoral()
        ))
      .until(() -> !manipulatorSubsystem.returnBeamBreak()) //coral beambreak true/false is flipped from intake beambreak
      .andThen(
      Commands.sequence(
      Commands.waitSeconds(0.1),
      Commands.parallel(
        armSubsystem.goToPackage(),
        elevatorSubsystem.intakeCoral(),
        Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore()),
        intakeSubsystem.handoff())
        .until(() -> armSubsystem.returnArmPos() < ArmConstants.Horizontal-5.0)
        .andThen(
        Commands.parallel(
        armSubsystem.goToPackage(),
        elevatorSubsystem.goToPackage(),
        Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore()),
        intakeSubsystem.handoff())
        ),
      intakeSubsystem.clearance())
      )));
  }

    /**
     * @apiNote ends with a coral in da grippa
     */
    public Command autonFloorIntakeStart() {
      return Commands.parallel(
        armSubsystem.horizontal(),
        intakeSubsystem.deploy(),
        manipulatorSubsystem.goScore(),
        elevatorSubsystem.intakeCoral()
        ).until(() -> intakeSubsystem.getBeamBreak());
    }

    /**
     * @apiNote handoff
     */
    public Command autonFloorIntakeEnd() {
      return Commands.sequence(
        Commands.waitSeconds(0.0),
        Commands.parallel(
          armSubsystem.horizontal(),
          Commands.sequence(Commands.waitSeconds(0.07), intakeSubsystem.handoffAndSpin()),
          manipulatorSubsystem.intakeGround(),
          elevatorSubsystem.intakeCoral()
          ))
        .until(() -> !manipulatorSubsystem.returnBeamBreak()) //coral beambreak true/false is flipped from intake beambreak
        .andThen(
        Commands.sequence(
        Commands.waitSeconds(0.1),
        Commands.parallel(
          armSubsystem.goToPackage(),
          elevatorSubsystem.intakeCoral(),
          Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore()),
          intakeSubsystem.handoff())
          .until(() -> armSubsystem.returnArmPos() < ArmConstants.Horizontal-5.0)
          .andThen(
          Commands.parallel(
          armSubsystem.goToPackage(),
          elevatorSubsystem.goToPackage(),
          Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore()),
          intakeSubsystem.handoff())
          ),
        intakeSubsystem.clearance())
        );
    }

    /**
     * @apiNote handoff straight to L4
     */
    public Command autonFloorIntakeEndFast() {
      return Commands.sequence(
        Commands.waitSeconds(0.0),
        Commands.parallel(
          armSubsystem.horizontal(),
          Commands.sequence(Commands.waitSeconds(0.07), intakeSubsystem.handoffAndSpin()),
          manipulatorSubsystem.intakeGround(),
          elevatorSubsystem.intakeCoral()
          ))
        .until(() -> !manipulatorSubsystem.returnBeamBreak()) //coral beambreak true/false is flipped from intake beambreak
        .andThen(
        Commands.sequence(
        Commands.waitSeconds(0.1),
        Commands.parallel(
          Commands.deadline(
            elevatorSubsystem.autonGoToL4(), 
            armSubsystem.goToPackage()),
          intakeSubsystem.clearance()
          )
        )
        );
    }

    /**
     * @deprecated full sequence
     */
    public Command floorIntakeAutonDeployFull(){
      return 
      Commands.parallel(
          armSubsystem.horizontal(),
          intakeSubsystem.deploy(),
          manipulatorSubsystem.goScore(),
          elevatorSubsystem.intakeCoral()
          )
        .until(() -> intakeSubsystem.getBeamBreak())
        .andThen(Commands.sequence(
        Commands.waitSeconds(0.0),
        Commands.parallel(
          armSubsystem.horizontal(),
          Commands.sequence(Commands.waitSeconds(0.07), intakeSubsystem.handoffAndSpin()),
          manipulatorSubsystem.intakeGround(),
          elevatorSubsystem.intakeCoral()
          ))
        .until(() -> !manipulatorSubsystem.returnBeamBreak()) //coral beambreak true/false is flipped from intake beambreak
        .andThen(
        Commands.sequence(
        Commands.waitSeconds(0.1),
        Commands.parallel(
          armSubsystem.goToPackage(),
          elevatorSubsystem.intakeCoral(),
          Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore()),
          intakeSubsystem.handoff())
          .until(() -> armSubsystem.returnArmPos() < ArmConstants.Horizontal-5.0)
          .andThen(
          Commands.parallel(
          armSubsystem.goToPackage(),
          elevatorSubsystem.goToPackage(),
          Commands.sequence(manipulatorSubsystem.zero(), manipulatorSubsystem.goScore()),
          intakeSubsystem.handoff())
          ),
        intakeSubsystem.clearance())
        ));
    }

    //packages the floor intake after a deploy
    public Command floorIntakeClearance(){
      return Commands.parallel(
        armSubsystem.goToPackage(), 
        intakeSubsystem.clearance(), 
        manipulatorSubsystem.zero(),
        elevatorSubsystem.goToPackage());
    }

    public Command bargePackage(){
      return Commands.parallel(
        elevatorSubsystem.goToPackage(),
        armSubsystem.goToPackage());
    }

    public Command algaePackage(){
      return Commands.parallel(armSubsystem.goToPackage());
    }

    //does the normal package like in coral mode, but stops the holding voltage for the algae instead of moving the wrist.
    public Command algaePackageElevator(){
      return Commands.parallel(armSubsystem.goToPackage()).until(() -> (armSubsystem.armGreaterThan(ArmConstants.Intermediate,2.0)))
      .andThen(Commands.parallel(elevatorSubsystem.goToPackage(), armSubsystem.goToPackage()));
    }

    public Command alignProcessor(){
      return Commands.sequence(runOnce(() -> {scoringLevel = ScoringLevel.Algae;}),
        Commands.parallel(elevatorSubsystem.goToProcessor())
        ,Commands.parallel(armSubsystem.processor()));
    }

    public Command barge(){
      return Commands.sequence(
      Commands.parallel(
        Commands.parallel(elevatorSubsystem.goToBarge(), runOnce(() -> {scoringLevel = ScoringLevel.Barge;}))
        ,armSubsystem.barge()),
        algaeSubsystem.runAlwaysAlgaeVoltage(AlgaeConstants.algaeExpelVoltage).withTimeout(0.5),
        bargePackage());
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

    public Command setLightsAligning() {
      return lightsSubsystem.setGreen();
    }

    public Command setLightsAlignGood() {
      return Commands.sequence(lightsSubsystem.changeAnimation(AnimationTypes.AutoAlign), lightsSubsystem.animate());
    }

    public Command setFancyLights() {
      return Commands.sequence(lightsSubsystem.changeAnimation(AnimationTypes.Twinkle), lightsSubsystem.animate());
    }

    public Command setOneLights(int light, Boolean good) {
      return lightsSubsystem.setOne(light, good);
    }
}