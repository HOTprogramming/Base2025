// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.derive;
import static frc.robot.subsystems.Drivetrain.DriveConstants.slowModeMultiplier;

import java.io.IOException;
import java.util.Map;
import java.util.jar.Attributes.Name;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.Camera.Camera.CameraPositions;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveSim;
import frc.robot.subsystems.GameSpec.Manager;
import frc.robot.subsystems.GameSpec.Algae.AlgaeConstants;
import frc.robot.subsystems.GameSpec.Climber.Climber;
import frc.robot.subsystems.GameSpec.Intake.Intake;
import frc.robot.subsystems.GameSpec.Lights.Lights;
import frc.robot.subsystems.Drivetrain.DriveKraken;


public class RobotContainer {

  private SendableChooser<String> chooser = new SendableChooser<>();
  private Command autoCommand;
  private String autoString;

  private Drive drivetrain;
  private Camera cameraSubsystem;
  private Manager gamespecManager;

  private enum Mode {
    coral,
    algae,
    climb
  }



  private Mode mode;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.

    if(!Utils.isSimulation()){
        drivetrain = new Drive(new DriveKraken());
        cameraSubsystem = new Camera(drivetrain);
    } else {
      drivetrain = new Drive(new DriveSim());
      cameraSubsystem = new Camera(drivetrain);

    }

    gamespecManager = new Manager();

    chooser.setDefaultOption("Auto", "Auto");
    
    chooser.addOption("RedR4", "RedR4");
    chooser.addOption("RedL4", "RedL4");
    chooser.addOption("BlueR4", "BlueR4"); 
    chooser.addOption("BlueL4", "BlueL4"); 
    chooser.addOption("TESTING", "TESTING"); 
    chooser.addOption("Copy of StatesRedR4", "Copy of StatesRedR4"); 
    chooser.addOption("Practice1", "Practice1"); 
    chooser.addOption("Test", "Test"); 
    chooser.addOption("lolipop", "lolipop"); 
    chooser.addOption("lolipop test", "lolipop test"); 

    // chooser.addOption("RedL3", "RedL3");
    // chooser.addOption("BlueR3", "BlueR3");
    // chooser.addOption("BlueL3", "BlueL3");
    // chooser.addOption("Copy of BlueL3", "Copy of BlueL3");

     

    // chooser.addOption("Complex Auto", "m_complexAuto");
    
    NamedCommands.registerCommand("L1", gamespecManager.goToL1());
    NamedCommands.registerCommand("L2", gamespecManager.goToL2());
    NamedCommands.registerCommand("L3", gamespecManager.goToL3());
    NamedCommands.registerCommand("L4", gamespecManager.goToL4());
    NamedCommands.registerCommand("Package", gamespecManager.goToPackage());
    NamedCommands.registerCommand("Coral HP", gamespecManager.coralGoHP());
    NamedCommands.registerCommand("Coral Score", gamespecManager.coralGoScore());
    NamedCommands.registerCommand("align station intake", gamespecManager.alignStationIntake());
    NamedCommands.registerCommand("shoot", gamespecManager.shoot());
    NamedCommands.registerCommand("lock fingers", gamespecManager.lockFingers()); 
    NamedCommands.registerCommand("open fingers", gamespecManager.OpenFingers());
    NamedCommands.registerCommand("done scoring", gamespecManager.doneScoring());
    NamedCommands.registerCommand("climb", gamespecManager.climberOut());
    NamedCommands.registerCommand("high algae", gamespecManager.highAlgae());
    NamedCommands.registerCommand("low algae", gamespecManager.lowAlgae());
    NamedCommands.registerCommand("align processor", gamespecManager.alignProcessor());
    NamedCommands.registerCommand("barge", Commands.parallel(gamespecManager.barge(), drivetrain.runOnce(() -> drivetrain.teleopDrive(0, 0, 0))));
    NamedCommands.registerCommand("L2 Package", gamespecManager.goToL2Package());
    NamedCommands.registerCommand("Algae Package", gamespecManager.algaePackage());
    NamedCommands.registerCommand("Barge Package", gamespecManager.bargePackage());

    NamedCommands.registerCommand("Lights Coral", gamespecManager.setLightsCoral());
    NamedCommands.registerCommand("Lights Algae", gamespecManager.setLightsAlgae());
    NamedCommands.registerCommand("Lights Climb", gamespecManager.setLightsClimb());
    NamedCommands.registerCommand("Lights Shoot", gamespecManager.setLightsShoot());


    NamedCommands.registerCommand("Lights Auto Bad", gamespecManager.setLightsBad());
    NamedCommands.registerCommand("Lights Auto Ok", gamespecManager.setLightsOk());
    NamedCommands.registerCommand("Lights Auto Good", gamespecManager.setLightsGood());


    NamedCommands.registerCommand("Align Reef Left",  drivetrain.autonAlignReefCommand(0));
    NamedCommands.registerCommand("Align Reef Center",  drivetrain.autonAlignReefCommand(1));
    NamedCommands.registerCommand("Align Reef Right",  drivetrain.autonAlignReefCommand(2));

    NamedCommands.registerCommand("Auton Shoot",  gamespecManager.autonShoot());
    NamedCommands.registerCommand("Auton Finish Shoot",  gamespecManager.autonFinishShoot());
    NamedCommands.registerCommand("Auton Intake Start", gamespecManager.autonIntake());
    NamedCommands.registerCommand("Auton Finish Intake", gamespecManager.autonFinishIntake());
    NamedCommands.registerCommand("AL4", gamespecManager.autonL4());
    NamedCommands.registerCommand("Intake Clearence", gamespecManager.intakeSubsystem.clearance());
    NamedCommands.registerCommand("Intake Bump", gamespecManager.intakeSubsystem.bump());
    NamedCommands.registerCommand("Half Height", gamespecManager.autonHalfL4());
    NamedCommands.registerCommand("Floor Intake Deploy", gamespecManager.floorIntakeDeploy());
    NamedCommands.registerCommand("Auton Floor Intake Start", gamespecManager.autonFloorIntakeStart());
    NamedCommands.registerCommand("Auton Floor Intake End Fast", gamespecManager.autonFloorIntakeEndFast());

    NamedCommands.registerCommand("Auton Fast Shoot Start", gamespecManager.autonShootStart());
    NamedCommands.registerCommand("Auton Fast Shoot End", gamespecManager.autonShootFinish());
    NamedCommands.registerCommand("Auton Shoot Intake", gamespecManager.autonShootIntake());


    
    NamedCommands.registerCommand("Auton Floor Intake End", gamespecManager.autonFloorIntakeEnd());
    NamedCommands.registerCommand("Stop Drive", drivetrain.runOnce(() -> drivetrain.teleopDrive(0, 0, 0)));

    NamedCommands.registerCommand("Chase Object", drivetrain.run(() -> drivetrain.chaseSlow()).until(() -> gamespecManager.intakeSubsystem.getBeamBreak()).onlyIf(() -> drivetrain.targetSeen)); // gamespecManager.intakeSubsystem.getBeamBreak() ||
    
    NamedCommands.registerCommand("Chase Auton", drivetrain.run(() -> drivetrain.chaseAuton()).until(() -> gamespecManager.intakeSubsystem.getBeamBreak()).onlyIf(() -> drivetrain.targetSeen)); // gamespecManager.intakeSubsystem.getBeamBreak() ||
   // NamedCommands.registerCommand("Chase Object", drivetrain.run(() -> drivetrain.chaseSlow()).until(() -> (gamespecManager.intakeSubsystem.getBeamBreak()))); // gamespecManager.intakeSubsystem.getBeamBreak() ||

    //new EventTrigger("Package").whileTrue(gamespecManager.goToPackage());


    // must make a runonce command using a functional command interface
    // NamedCommands.registerCommand("Auton Align Left",  drivetrain.run(() -> drivetrain.alignReef(0)));
    // NamedCommands.registerCommand("Auton Align Right", drivetrain.run(() -> drivetrain.alignReef(1)));

    mode = Mode.coral;

    SmartDashboard.putData(chooser);

    configureBindings();
  }

  private void configureBindings() {   

    // drivetrain.setDefaultCommand
    //   (drivetrain.run(() -> {
    //         drivetrain.headingControl(
    //           Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0, 
    //           Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0);
    //       }
    //   ));    

      // driver.axisLessThan(0, -0.00)
      //   .or(driver.axisGreaterThan(0, 0.00))
      //   .or(driver.axisLessThan(1, -0.00))
      //   .or(driver.axisGreaterThan(1, 0.00))
      //   .and(driver.y().negate())
      //   .whileTrue

      gamespecManager.intakeSubsystem.setDefaultCommand(
        Commands.either(gamespecManager.intakeSubsystem.bump(), gamespecManager.intakeSubsystem.clearance(), 
        () -> (((gamespecManager.armSubsystem.returnArmCommandedPos() < -90.0) && (gamespecManager.elevatorSubsystem.returnElevatorPos() <= 28.0)) 
        || (((gamespecManager.armSubsystem.returnArmPos() < -100.0) && (gamespecManager.armSubsystem.returnArmCommandedPos() <= -90.0)) && (gamespecManager.elevatorSubsystem.returnElevatorCommandedPos() <= 28.0))
        || (gamespecManager.armSubsystem.returnArmCommandedPos() > 5.0 && gamespecManager.elevatorSubsystem.returnElevatorCommandedPos() < 20.0)
        || (gamespecManager.armSubsystem.returnArmPos() > -20.0 && gamespecManager.elevatorSubsystem.returnElevatorPos() < 20.0)))
        .unless(this::isClimb) 
      );

      gamespecManager.algaeSubsystem.setDefaultCommand(  
      Commands.either(gamespecManager.algaeSubsystem.algaeVoltage(0.0), 
      gamespecManager.algaeSubsystem.algaeVoltage(AlgaeConstants.algaeHoldVoltage),
      () -> gamespecManager.algaeSubsystem.returnAlgaeInWhileHolding())
      );

      drivetrain.setDefaultCommand
      (drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.015 ? -driver.getRightX() : 0);
        }
      ));
      // .onFalse(Commands.race(Commands.waitSeconds(0.15), drivetrain.run(() -> {
      //   drivetrain.teleopDrive(
      //     Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
      //     Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
      //     Math.abs(driver.getRightX()) >= 0.015 ? -driver.getRightX() : 0);
      //   })));

      // driver.leftBumper().whileTrue(drivetrain.run(() -> {
      //   drivetrain.teleopDriveSlow(
      //     Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
      //     Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
      //     Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
      // }))
      // .onFalse(Commands.race(Commands.waitSeconds(0.15), drivetrain.run(() -> {
      //   drivetrain.teleopDrive(
      //     Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
      //     Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
      //     Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
      //   })));

      driver.a().whileTrue(
        drivetrain.run(() -> {drivetrain.alignObjectTeleop(
          (Math.abs(driver.getLeftX()) >= 0.1 ? driver.getLeftX() : 0) * 0.5,
          (Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0) * 0.5,
          (Math.abs(driver.getRightX()) >= 0.015 ? -driver.getRightX() : 0) * 0.5);
        }
      ));


      driver.y()
      .and(driver.axisLessThan(4, -0.15).or(driver.axisGreaterThan(4, 0.15))
      .or(driver.axisLessThan(5, -0.15)).or(driver.axisGreaterThan(5, 0.15)))
      .whileTrue
      (drivetrain.run(() -> {
        drivetrain.lockReefManual(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.1 ? -driver.getRightX() : 0,
          Math.abs(driver.getRightY()) >= 0.1 ? -driver.getRightY() : 0);
        }
      ));
      // b right y middle x left
      // driver.rightBumper().whileTrue(Commands.sequence(Commands.parallel(drivetrain.runOnce(() -> drivetrain.updateReefTarget(2)), gamespecManager.setLightsAligning()), drivetrain.resetControllers(), drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)))).onFalse(refreshLights());
      // driver.y().whileTrue(Commands.sequence(Commands.parallel(drivetrain.runOnce(() -> drivetrain.updateReefTarget(1)), gamespecManager.setLightsAligning()), drivetrain.resetControllers(), drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)))).onFalse(refreshLights());
      // driver.leftBumper().whileTrue(Commands.sequence(Commands.parallel(drivetrain.runOnce(() -> drivetrain.updateReefTarget(0)), gamespecManager.setLightsAligning()), drivetrain.resetControllers(), drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)))).onFalse(refreshLights());

      driver.rightBumper().onTrue(Commands.sequence(drivetrain.runOnce(() -> drivetrain.updateReefTarget(2)), gamespecManager.setLightsAligning(), drivetrain.resetControllers()));
      driver.y().onTrue(Commands.sequence(drivetrain.runOnce(() -> drivetrain.updateReefTarget(1)), gamespecManager.setLightsAligning(), drivetrain.resetControllers()));
      driver.leftBumper().onTrue(Commands.sequence(drivetrain.runOnce(() -> drivetrain.updateReefTarget(0)), gamespecManager.setLightsAligning(), drivetrain.resetControllers()));

      driver.rightBumper().whileTrue(drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)));
      driver.y().whileTrue(drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)));
      driver.leftBumper().whileTrue(drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)));

      driver.rightBumper().onFalse(refreshLights());
      driver.y().onFalse(refreshLights());
      driver.leftBumper().onFalse(refreshLights());

      
      driver.rightBumper().and(() -> drivetrain.getAutoAlignGood()).onTrue(gamespecManager.setLightsAlignGood());
      driver.y().and(() -> drivetrain.getAutoAlignGood()).onTrue(gamespecManager.setLightsAlignGood());
      driver.leftBumper().and(() -> drivetrain.getAutoAlignGood()).onTrue(gamespecManager.setLightsAlignGood());
      // driver.b().whileTrue(Commands.sequence(drivetrain.runOnce(() -> drivetrain.updateReefTarget(1)), drivetrain.run(() -> drivetrain.alignReefFieldcentric())));      
      // driver.x().whileTrue(Commands.sequence(drivetrain.runOnce(() -> drivetrain.updateReefTarget(0)), drivetrain.run(() -> drivetrain.alignReefFieldcentric())));      
    
      new Trigger(() -> cameraSubsystem.getCameraAlive(CameraPositions.RIGHT)).onTrue(gamespecManager.setOneLights(0, true)).onFalse(gamespecManager.setOneLights(0, false).ignoringDisable(true));
      new Trigger(() -> cameraSubsystem.getCameraAlive(CameraPositions.LEFT)).onTrue(gamespecManager.setOneLights(3, true)).onFalse(gamespecManager.setOneLights(3, false).ignoringDisable(true));
      new Trigger(() -> cameraSubsystem.getCameraAlive(CameraPositions.TOP)).onTrue(gamespecManager.setOneLights(5, true)).onFalse(gamespecManager.setOneLights(5, false).ignoringDisable(true));


      new Trigger(() -> gamespecManager.climberSubsystem.returnReadyToClimb()).and(this::isClimb).onTrue(gamespecManager.setLightsGood()).onFalse(gamespecManager.setLightsClimb());
      // driver.a().onTrue(gamespecManager.setOneLights(0, true)).onFalse(gamespecManager.setOneLights(0, false).ignoringDisable(true));
      // driver.b().onTrue(gamespecManager.setOneLights(3, true)).onFalse(gamespecManager.setOneLights(3, false).ignoringDisable(true));
      // driver.x().onTrue(gamespecManager.setOneLights(5, true)).onFalse(gamespecManager.setOneLights(5, false).ignoringDisable(true));


      // driver.b().whileTrue(NamedCommands.getCommand("Align Reef Left"));
      // driver.x().whileTrue(NamedCommands.getCommand("Align Reef Right"));
      // driver.b().onTrue(NamedCommands.getCommand("expel"));
      driver.rightTrigger().onTrue(NamedCommands.getCommand("shoot")
      .onlyIf(operator.b().or(operator.a()).or(operator.x()).or(operator.y())));

      driver.start().onTrue(drivetrain.resetPidgeon()).onFalse(drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
        }));

       operator.leftBumper().onTrue(Commands.parallel(gamespecManager.runOnce(() -> mode = Mode.coral), NamedCommands.getCommand("Lights Coral")));
       operator.rightBumper().onTrue(Commands.parallel(gamespecManager.runOnce(() -> mode = Mode.algae), NamedCommands.getCommand("Lights Algae")));
       operator.start().onTrue(Commands.parallel(gamespecManager.runOnce(() -> mode = Mode.climb), NamedCommands.getCommand("Lights Climb")));
       operator.back().onTrue(Commands.parallel(gamespecManager.runOnce(() -> mode = Mode.climb), NamedCommands.getCommand("Lights Climb")));

      operator.a().and(this::isCoral).onTrue(NamedCommands.getCommand("L2")).onFalse(Commands.parallel(NamedCommands.getCommand("L2 Package"), NamedCommands.getCommand("done scoring")));
      operator.b().and(this::isCoral).onTrue(NamedCommands.getCommand("L3")).onFalse(Commands.parallel(NamedCommands.getCommand("Package"), NamedCommands.getCommand("done scoring")));
      operator.x().and(this::isCoral).onTrue(NamedCommands.getCommand("L1")).onFalse(Commands.parallel(NamedCommands.getCommand("Package"), NamedCommands.getCommand("done scoring")));
      operator.y().and(this::isCoral).onTrue(NamedCommands.getCommand("L4")).onFalse(Commands.parallel(NamedCommands.getCommand("Package"), NamedCommands.getCommand("done scoring")));
      operator.rightTrigger().and(this::isCoral).whileTrue(NamedCommands.getCommand("align station intake")).onFalse(Commands.parallel(NamedCommands.getCommand("Package"))); //, NamedCommands.getCommand("Stop Intake")));

      operator.a().and(this::isAlgae).whileTrue(NamedCommands.getCommand("low algae")).onFalse(Commands.parallel(NamedCommands.getCommand("Algae Package")));
      operator.b().and(this::isAlgae).whileTrue(NamedCommands.getCommand("high algae")).onFalse(Commands.parallel(NamedCommands.getCommand("Algae Package")));
      operator.x().and(this::isAlgae).onTrue(NamedCommands.getCommand("align processor")).onFalse(gamespecManager.algaePackageElevator());

      operator.y()
      .and(this::isAlgae)
      .and(new Trigger(() -> drivetrain.returnAutoBarge()))
      .onTrue(NamedCommands.getCommand("barge"));

      operator.a().and(this::isClimb).onTrue(NamedCommands.getCommand("climb"));      
      operator.y().and(this::isClimb).onTrue(NamedCommands.getCommand("lock fingers"));
      operator.x().and(this::isClimb).onTrue(NamedCommands.getCommand("open fingers"));

      //auto climb code
      new Trigger(() -> gamespecManager.climberSubsystem.returnReadyToClimb())
      .debounce(1.0)
      .and(operator.b().and(this::isClimb))
      .onTrue(gamespecManager.autoPackageClimber());

      new Trigger(() -> gamespecManager.climberSubsystem.returnReadyToClimb())
      .and(operator.b().and(this::isClimb))
      .onTrue(NamedCommands.getCommand("open fingers"));

      operator.leftTrigger().or(driver.leftTrigger()).or(() -> gamespecManager.returnIntakeState())
      .whileTrue(gamespecManager.floorIntakeDeploy()).onFalse(gamespecManager.floorIntakeClearance());

      operator.povUp().or(operator.povLeft().or(operator.povDown().or(operator.povRight()))).onTrue(NamedCommands.getCommand("Package").unless(this::isClimb));

      operator.axisLessThan(5, -0.05).or(operator.axisGreaterThan(5, 0.05)).and(this::isClimb).whileTrue(
      Commands.sequence(
        gamespecManager.climberSubsystem.run(
        () -> gamespecManager.climberSubsystem.setPower(-operator.getRightY())
      ).onlyWhile(() -> gamespecManager.climberSubsystem.checkClimberSoftStop())
      ,gamespecManager.climberSubsystem.run(
        () -> gamespecManager.climberSubsystem.setPower(0.0)
      ).onlyIf(() -> !gamespecManager.climberSubsystem.checkClimberSoftStop())))
      .whileFalse(gamespecManager.climberSubsystem.runOnce(
        () -> gamespecManager.climberSubsystem.setPower(0.0)));

      operator.leftTrigger().or(operator.rightTrigger()).onFalse(NamedCommands.getCommand("Coral Zero"));

       NamedCommands.registerCommand("OTF", drivetrain.generateOnTheFly());
      NamedCommands.registerCommand("R_OTF", drivetrain.runOnTheFly());

      new EventTrigger("OTF").onTrue(Commands.runOnce(() -> drivetrain.generateOnTheFly()));

  }

  public boolean isCoral() {
    return mode == Mode.coral;
  }
  public boolean isAlgae() {
    return mode == Mode.algae;
  }
  public boolean isClimb() {
    return mode == Mode.climb;
  }


        //      operator.leftTrigger().and(operator.y())
      //      .whileTrue(gamespecManager.L3());

  public void updateAutonCommand() {
    if (autoString != null) {
      if (chooser.getSelected() != autoString) {
        autoString = chooser.getSelected();
        autoCommand = new PathPlannerAuto(autoString);
        try {
          drivetrain.setAutonStartPose(PathPlannerAuto.getPathGroupFromAutoFile(autoString).get(0).getStartingDifferentialPose());
        } catch (IOException e) {
          System.err.println("No Auto File");
        } catch (ParseException e) {
          System.err.println("idk good luck");
        } catch (IndexOutOfBoundsException e) {
          System.err.println("no auto Paths");
        }
      }
    } else {
      autoString = chooser.getSelected();
    }
  }

  public Command refreshLights() {
    return gamespecManager.setLightsAlgae().onlyIf(() -> isAlgae())
    .andThen(gamespecManager.setLightsClimb().onlyIf(() -> isClimb()))
    .andThen(gamespecManager.setLightsCoral().onlyIf(() -> isCoral()));
  }

  public Command lightsDisable() {
    return gamespecManager.setFancyLights();
  }

  public void resetLeds() {
    switch (this.mode) {
      case coral:
      NamedCommands.getCommand("Lights Coral").schedule();
        break;
    
      case algae:
      NamedCommands.getCommand("Lights Algae").schedule();
        break;

      case climb:
      NamedCommands.getCommand("Lights Climb").schedule();
        break;
    }
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}