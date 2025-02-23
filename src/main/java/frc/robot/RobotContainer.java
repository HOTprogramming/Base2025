// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.derive;

import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveSim;
import frc.robot.subsystems.GameSpec.Manager;
import frc.robot.subsystems.GameSpec.Climber.Climber;
import frc.robot.subsystems.Lights.Lights;
import frc.robot.subsystems.Drivetrain.DriveKraken;


public class RobotContainer {

  private SendableChooser<String> chooser = new SendableChooser<>();

  private Drive drivetrain;
  private Camera cameraSubsystem;
  private Manager gamespecManager;
  private Lights m_Lights;

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
        m_Lights = new Lights();
    } else {
      drivetrain = new Drive(new DriveSim());
      m_Lights = new Lights();
      cameraSubsystem = new Camera(drivetrain);

    }

    gamespecManager = new Manager();

    chooser.setDefaultOption("Auto", "Auto");
    
    chooser.addOption("RedR3", "RedR3"); 
    chooser.addOption("RedL3", "RedL3");
    chooser.addOption("BlueR3", "BlueR3");
    chooser.addOption("BlueL3", "BlueL3");

     

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
    NamedCommands.registerCommand("cancel shoot", gamespecManager.cancelShoot());
    NamedCommands.registerCommand("done scoring", gamespecManager.doneScoring());
    NamedCommands.registerCommand("climb", gamespecManager.climberOut());
    NamedCommands.registerCommand("high algae", gamespecManager.highAlgae());
    NamedCommands.registerCommand("low algae", gamespecManager.lowAlgae());
    NamedCommands.registerCommand("align floor intake", gamespecManager.alignFloorIntake());
    NamedCommands.registerCommand("align processor", gamespecManager.alignProcessor());
    NamedCommands.registerCommand("barge", gamespecManager.barge());
    NamedCommands.registerCommand("intake", gamespecManager.algaeIntake());
    NamedCommands.registerCommand("stop intake", gamespecManager.algaeStopIntake());
    NamedCommands.registerCommand("L2 Package", gamespecManager.goToL2Package());
    NamedCommands.registerCommand("Floor Intake Package", gamespecManager.floorIntakePackage());


    NamedCommands.registerCommand("Align Reef Left",  drivetrain.autonAlignReefCommand(0));
    NamedCommands.registerCommand("Align Reef Right",  drivetrain.autonAlignReefCommand(1));

    NamedCommands.registerCommand("Auton Shoot",  gamespecManager.autonShoot());
    NamedCommands.registerCommand("Auton Intake Start", gamespecManager.autonIntake());
    NamedCommands.registerCommand("Auton Finish Intake", gamespecManager.autonFinishIntake());
    NamedCommands.registerCommand("AL4", gamespecManager.autonL4());


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

      driver.leftTrigger().whileTrue
      (drivetrain.run(() -> {
        drivetrain.lockReef(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0);
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
      driver.b().whileTrue(Commands.sequence(Commands.parallel(drivetrain.runOnce(() -> drivetrain.updateReefTargetWBall(2)), cameraSubsystem.setIgnore()), drivetrain.run(() -> drivetrain.alignReefFieldcentric()))).onFalse(cameraSubsystem.setUnIgnore());
      driver.y().whileTrue(Commands.sequence(Commands.parallel(drivetrain.runOnce(() -> drivetrain.updateReefTargetWBall(1)), cameraSubsystem.setIgnore()), drivetrain.run(() -> drivetrain.alignReefFieldcentric()))).onFalse(cameraSubsystem.setUnIgnore());
      driver.x().whileTrue(Commands.sequence(Commands.parallel(drivetrain.runOnce(() -> drivetrain.updateReefTargetWBall(0)), cameraSubsystem.setIgnore()), drivetrain.run(() -> drivetrain.alignReefFieldcentric()))).onFalse(cameraSubsystem.setUnIgnore());

      // driver.b().whileTrue(Commands.sequence(drivetrain.runOnce(() -> drivetrain.updateReefTarget(1)), drivetrain.run(() -> drivetrain.alignReefFieldcentric())));      
      // driver.x().whileTrue(Commands.sequence(drivetrain.runOnce(() -> drivetrain.updateReefTarget(0)), drivetrain.run(() -> drivetrain.alignReefFieldcentric())));      


      // driver.b().whileTrue(NamedCommands.getCommand("Align Reef Left"));
      // driver.x().whileTrue(NamedCommands.getCommand("Align Reef Right"));
      // driver.b().onTrue(NamedCommands.getCommand("expel"));
      driver.rightTrigger().onTrue(NamedCommands.getCommand("shoot")
      .onlyIf(operator.b().or(operator.a()).or(operator.x()).or(operator.y())))
      .onFalse(NamedCommands.getCommand("cancel shoot")
      .onlyIf(operator.b().or(operator.a()).or(operator.x()).or(operator.y())));
      // driver.leftTrigger().onTrue(NamedCommands.getCommand("Intake"));

      driver.start().onTrue(drivetrain.resetPidgeon()).onFalse(drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
        }));

       operator.leftBumper().onTrue(gamespecManager.runOnce(() -> mode = Mode.coral));
       operator.rightBumper().onTrue(gamespecManager.runOnce(() -> mode = Mode.algae));
       operator.start().onTrue(gamespecManager.runOnce(() -> mode = Mode.climb));
       operator.back().onTrue(gamespecManager.runOnce(() -> mode = Mode.climb));

      operator.a().and(this::isCoral).onTrue(NamedCommands.getCommand("L2")).onFalse(Commands.parallel(NamedCommands.getCommand("L2 Package"), NamedCommands.getCommand("done scoring")));
      operator.b().and(this::isCoral).onTrue(NamedCommands.getCommand("L3")).onFalse(Commands.parallel(NamedCommands.getCommand("Package"), NamedCommands.getCommand("done scoring")));
      operator.x().and(this::isCoral).onTrue(NamedCommands.getCommand("L1")).onFalse(Commands.parallel(NamedCommands.getCommand("Package"), NamedCommands.getCommand("done scoring")));
      operator.y().and(this::isCoral).onTrue(NamedCommands.getCommand("L4")).onFalse(Commands.parallel(NamedCommands.getCommand("Package"), NamedCommands.getCommand("done scoring")));
      operator.leftTrigger().and(this::isCoral).whileTrue(NamedCommands.getCommand("align floor intake")); 
      operator.rightTrigger().and(this::isCoral).whileTrue(NamedCommands.getCommand("align station intake")).onFalse(Commands.parallel(NamedCommands.getCommand("Package"))); //, NamedCommands.getCommand("Stop Intake")));

      operator.a().and(this::isAlgae).onTrue(NamedCommands.getCommand("low algae")).onFalse(Commands.parallel(NamedCommands.getCommand("Package")));
      operator.b().and(this::isAlgae).onTrue(NamedCommands.getCommand("high algae")).onFalse(Commands.parallel(NamedCommands.getCommand("Package")));
      operator.x().and(this::isAlgae).onTrue(NamedCommands.getCommand("align processor")).onFalse(Commands.parallel(NamedCommands.getCommand("Package")));
      operator.y().and(this::isAlgae).onTrue(NamedCommands.getCommand("barge")).onFalse(Commands.parallel(NamedCommands.getCommand("Package")));
      operator.leftTrigger().and(this::isAlgae).whileTrue(NamedCommands.getCommand("align floor intake")).onFalse(NamedCommands.getCommand("Floor Intake Package"));
      operator.rightTrigger().and(this::isAlgae).whileTrue(NamedCommands.getCommand("intake")).onFalse(NamedCommands.getCommand("stop intake"));

      operator.a().and(this::isClimb).onTrue(NamedCommands.getCommand("climb"));      
      operator.x().and(this::isClimb).onTrue(NamedCommands.getCommand("lock fingers"));
      operator.y().and(this::isClimb).onTrue(NamedCommands.getCommand("open fingers"));

      operator.axisLessThan(5, -0.05).or(operator.axisGreaterThan(5, 0.05)).and(this::isClimb).whileTrue(
        gamespecManager.climberSubsystem.run(
          () -> gamespecManager.climberSubsystem.setPower(-operator.getRightY())
        )
      ).onFalse(gamespecManager.climberSubsystem.run(
        () -> gamespecManager.climberSubsystem.setPower(0.0)
      ));
      operator.leftTrigger().or(operator.rightTrigger()).onFalse(NamedCommands.getCommand("Coral Zero"));
      System.out.println("c");

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

  public Command getAutonomousCommand() {
    Timer timer = new Timer();
    timer.start();
    String autoName = chooser.getSelected();
    timer.stop();
    System.err.println(timer.get());
    return new PathPlannerAuto(autoName).finallyDo(() -> System.out.println("ENDED AUTO COMMAND"));
  }
}