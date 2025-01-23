// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveSim;
import frc.robot.subsystems.GameSpec.Manager;
import frc.robot.subsystems.Drivetrain.DriveKraken;


public class RobotContainer {
  private Drive drivetrain;
  private Camera cameraSubsystem;
  private Manager gamespecManager;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        drivetrain = new Drive(new DriveKraken());
        cameraSubsystem = new Camera(drivetrain);
      }
      case DEVBOT -> {
        drivetrain = new Drive(new DriveKraken());
        cameraSubsystem = new Camera(drivetrain);
      }
      case SIMBOT -> {
        drivetrain = new Drive(new DriveSim());
      }
    }

    gamespecManager = new Manager();

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

      driver.b().whileTrue
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

      // driver.povLeft().onTrue(drivetrain.run(() -> drivetrain.alignReefLeft()));
      // driver.povRight().onTrue(drivetrain.run(() -> drivetrain.alignReefRight()));
      // driver.povDown().onTrue(drivetrain.run(() -> drivetrain.chaseObject()));    
      driver.povLeft().onTrue(drivetrain.run(() -> drivetrain.chaseObjectLeft()));  
      driver.povRight().onTrue(drivetrain.run(() -> drivetrain.chaseObjectRight()));  

      driver.start().onTrue(drivetrain.resetPidgeon());

      //four positions (l1, l2, l3, l4), human player, Barge, package

      operator.a().onTrue(gamespecManager.L1());
      operator.b().onTrue(gamespecManager.L2());
      operator.y().onTrue(gamespecManager.L3());
      operator.x().onTrue(gamespecManager.L4());
      operator.rightBumper().onTrue(gamespecManager.Barge());
      operator.rightTrigger().onTrue(gamespecManager.HP());
      
      //      operator.leftTrigger().and(operator.y())
      //      .whileTrue(gamespecManager.L3());

  }


  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Auto");
  }

}
