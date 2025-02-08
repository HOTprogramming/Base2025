// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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
import frc.robot.subsystems.Drivetrain.DriveKraken;


public class RobotContainer {

  private SendableChooser<String> chooser = new SendableChooser<>();

  private Drive drivetrain;
  private Camera cameraSubsystem;
  private Manager gamespecManager;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public enum Mode{
    CORAL,
    ALGAE,
    CLIMB
  }

  public static Mode mode = Mode.CORAL;

  public RobotContainer() {
    RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.

    if(!Utils.isSimulation()){
        drivetrain = new Drive(new DriveKraken());
        cameraSubsystem = new Camera(drivetrain);
    } else {
      drivetrain = new Drive(new DriveSim());
    }

    gamespecManager = new Manager();

    chooser.setDefaultOption("Auto", "Auto");
    chooser.addOption("goods", "REDR4Place&Pickup");
    // chooser.addOption("Complex Auto", "m_complexAuto");
    // NamedCommands.registerCommand("Coral Intake", gamespecManager.coralIntake());
    // NamedCommands.registerCommand("Coral Shoot", gamespecManager.coralShoot());
    // NamedCommands.registerCommand("Coral Zero", gamespecManager.coralZero());

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
      ));    

      driver.rightBumper().whileTrue
      (drivetrain.run(() -> {
        drivetrain.robotCentricTeleopDrive(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
        }
      ));

      driver.axisLessThan(4, -0.15)
        .or(driver.axisGreaterThan(4, 0.15))
        .and(driver.rightBumper().negate())
        .and(driver.leftBumper().negate())
        .and(driver.y().negate())
        .whileTrue
      (drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
        }
      )).onFalse(Commands.race(Commands.waitSeconds(0.2), drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
        })));

      driver.povUp().whileTrue
      (drivetrain.run(() -> {
        drivetrain.lockRotation(
          Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
          Rotation2d.fromDegrees(0));
        }
      ));

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


      driver.povUp().onTrue(drivetrain.run(() -> drivetrain.alignReef(0)));    
      driver.povLeft().onTrue(drivetrain.run(() -> drivetrain.alignReef(1)));  
      driver.povRight().onTrue(drivetrain.run(() -> drivetrain.alignReef(-1))); 

      driver.start().onTrue(drivetrain.resetPidgeon());

      // operator.a().onTrue(NamedCommands.getCommand("L1"));
      // operator.b().onTrue(NamedCommands.getCommand("L2"));
      // operator.x().onTrue(NamedCommands.getCommand("L3"));
      // operator.y().onTrue(NamedCommands.getCommand("L4"));
      // operator.leftBumper().onTrue(NamedCommands.getCommand("Package"));
      // operator.rightBumper().onTrue(NamedCommands.getCommand("Feeder"));
      operator.leftTrigger().or(operator.rightTrigger()).onFalse(gamespecManager.coralZero());
      operator.a().onTrue(gamespecManager.goToPackage());
      operator.b().onTrue(gamespecManager.goToL4());
      operator.x().onTrue(gamespecManager.goToL3());
      operator.y().onTrue(gamespecManager.L4Score());
      operator.leftTrigger().whileTrue(gamespecManager.coralIntake());
      operator.rightTrigger().whileTrue(gamespecManager.coralShoot());

      NamedCommands.registerCommand("OTF", drivetrain.generateOnTheFly());
      NamedCommands.registerCommand("R_OTF", drivetrain.runOnTheFly());


      new EventTrigger("OTF").onTrue(Commands.runOnce(() -> drivetrain.generateOnTheFly()));
  }

        //      operator.leftTrigger().and(operator.y())
      //      .whileTrue(gamespecManager.L3());


  public Command getAutonomousCommand() {
    String autoName = chooser.getSelected();
    return new PathPlannerAuto(autoName).finallyDo(() -> System.out.println("ENDED AUTO COMMAND"));
  }

}