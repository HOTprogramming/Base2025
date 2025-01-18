// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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
    }

    gamespecManager = new Manager();

    chooser.setDefaultOption("Auto", "Auto");
    // chooser.addOption("Complex Auto", "m_complexAuto");
    
    NamedCommands.registerCommand("L1", gamespecManager.goToL1());
    NamedCommands.registerCommand("L2", gamespecManager.goToL2());
    NamedCommands.registerCommand("L3", gamespecManager.goToL3());
    NamedCommands.registerCommand("L4", gamespecManager.goToL4());
    NamedCommands.registerCommand("Package", gamespecManager.goToPackage());
    NamedCommands.registerCommand("Feeder", gamespecManager.goToFeeder());
    NamedCommands.registerCommand("Coral HP", gamespecManager.coralGoHP());
    NamedCommands.registerCommand("Coral Score", gamespecManager.coralGoScore());
    NamedCommands.registerCommand("Intake", gamespecManager.Intake());
    NamedCommands.registerCommand("Stop Intake", gamespecManager.StopIntake());
    NamedCommands.registerCommand("align station intake", gamespecManager.alignStationIntake());
    NamedCommands.registerCommand("shoot", gamespecManager.shoot());


    mode = Mode.coral;

    SmartDashboard.putData(chooser);

    configureBindings();
  }

  private void configureBindings() {    

    drivetrain.setDefaultCommand
      (drivetrain.run(() -> {
            drivetrain.headingControl(
              Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0, 
              Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0);
          }
      ).unless(this::isClimb));    

      driver.axisLessThan(4, -0.05)
        .or(driver.axisGreaterThan(4, 0.05))
        .and(driver.y().negate())
        .whileTrue
      (drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
        }
      )).onFalse(Commands.race(Commands.waitSeconds(0.15), drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
        })));

      driver.a().whileTrue
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
  
      driver.leftBumper().onTrue(drivetrain.run(() -> drivetrain.alignReef(1)));  
      driver.rightBumper().onTrue(drivetrain.run(() -> drivetrain.alignReef(-1))); 
      driver.b().onTrue(NamedCommands.getCommand("expel"));
      driver.rightTrigger().onTrue(NamedCommands.getCommand("shoot"));
      driver.leftTrigger().onTrue(NamedCommands.getCommand("Intake"));

      driver.start().onTrue(drivetrain.resetPidgeon());

      operator.leftBumper().onTrue(gamespecManager.runOnce(() -> mode = Mode.coral));
      operator.rightBumper().onTrue(gamespecManager.runOnce(() -> mode = Mode.algae));

      operator.start().onTrue(gamespecManager.runOnce(() -> mode = Mode.climb));
      operator.back().onTrue(gamespecManager.runOnce(() -> mode = Mode.climb));

      operator.a().and(this::isCoral).onTrue(NamedCommands.getCommand("L2")).onFalse(Commands.parallel(NamedCommands.getCommand("Package")));
      operator.b().and(this::isCoral).onTrue(NamedCommands.getCommand("L3")).onFalse(Commands.parallel(NamedCommands.getCommand("Package")));
      operator.x().and(this::isCoral).onTrue(NamedCommands.getCommand("L1")).onFalse(Commands.parallel(NamedCommands.getCommand("Package")));
      operator.y().and(this::isCoral).onTrue(NamedCommands.getCommand("L4")).onFalse(Commands.parallel(NamedCommands.getCommand("Package")));
      operator.leftTrigger().and(this::isCoral).whileTrue(NamedCommands.getCommand("align floor intake")); 
      operator.rightTrigger().and(this::isCoral).whileTrue(NamedCommands.getCommand("align station intake")).onFalse(Commands.parallel(NamedCommands.getCommand("Package"), NamedCommands.getCommand("Stop Intake")));

      operator.a().and(this::isAlgae).onTrue(NamedCommands.getCommand("Package"));
      operator.b().and(this::isAlgae).onTrue(NamedCommands.getCommand("L3"));
      operator.x().and(this::isAlgae).onTrue(NamedCommands.getCommand("processer"));
      operator.y().and(this::isAlgae).onTrue(NamedCommands.getCommand("barge"));
      operator.leftTrigger().and(this::isAlgae).whileTrue(NamedCommands.getCommand("align floor intake"));
      operator.rightTrigger().and(this::isAlgae).whileTrue(NamedCommands.getCommand("align processor"));

      operator.a().and(this::isClimb).onTrue(NamedCommands.getCommand("climb"));      
      operator.x().and(this::isClimb).onTrue(NamedCommands.getCommand("lock fingers"));

      operator.axisLessThan(5, -0.05).or(operator.axisGreaterThan(5, 0.05)).and(this::isClimb).whileTrue(
        gamespecManager.climberSubsystem.run(
          () -> gamespecManager.climberSubsystem.setPower(-operator.getRightY())
        )
      ).onFalse(gamespecManager.climberSubsystem.run(
        () -> gamespecManager.climberSubsystem.setPower(0.0)
      ));

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

  public Command getAutonomousCommand() {
    String autoName = chooser.getSelected();
    return new PathPlannerAuto(autoName).finallyDo(() -> System.out.println("ENDED AUTO COMMAND"));
  }
}