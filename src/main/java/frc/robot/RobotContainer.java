// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveSim;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Elevator.ElevatorIOReal;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.Elevator;


public class RobotContainer {
  public Drive drivetrain;

  private Elevator elevatorSubsystem = null;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        this.drivetrain = new Drive(new DriveKraken());
        this.elevatorSubsystem = new Elevator(new ElevatorIOReal());
      }
      case DEVBOT -> {}
      case SIMBOT -> {
        this.drivetrain = new Drive(new DriveSim());
        elevatorSubsystem = new Elevator(new ElevatorIOSim());
      }
    }


    configureBindings();
  }

  private void configureSimpleBindings() {
    drivetrain.setDefaultCommand(
      drivetrain.run(() -> drivetrain.teleopDrive( 
        Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
        Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
        Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0)
      )
    );
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

      driver.povLeft().onTrue(drivetrain.run(() -> drivetrain.alignReefLeft()));
      driver.povRight().onTrue(drivetrain.run(() -> drivetrain.alignReefRight()));

      driver.start().onTrue(drivetrain.resetPidgeon());


      elevatorSubsystem.setDefaultCommand(elevatorSubsystem.stop());

      driver.a().whileTrue(elevatorSubsystem.runToPosition(.5));
      driver.x().whileTrue(elevatorSubsystem.runToPosition(1));
      driver.y().whileTrue(elevatorSubsystem.runToPosition(1.5));
      driver.b().whileTrue(elevatorSubsystem.runToPosition(2));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
  }

}
