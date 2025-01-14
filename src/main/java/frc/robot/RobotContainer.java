// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveSim;
import frc.robot.subsystems.GameSpec.Arm.Arm;
import frc.robot.subsystems.GameSpec.Arm.ArmIOReal;
import frc.robot.subsystems.GameSpec.Arm.ArmIOSim;
import frc.robot.subsystems.GameSpec.Elevator.Elevator;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOReal;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveIO;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Drivetrain.DriveSim;

public class RobotContainer {
  private Elevator elevatorSubsystem;
  public Drive drivetrain;
  private Arm armSubsystem;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        elevatorSubsystem = new Elevator(new ElevatorIOReal());   
        drivetrain = new Drive(new DriveKraken());
        armSubsystem = new Arm(new ArmIOReal());
      }
      case DEVBOT -> {}
      case SIMBOT -> {
        elevatorSubsystem = new Elevator(new ElevatorIOSim());
        drivetrain = new Drive(new DriveSim());
        armSubsystem = new Arm(new ArmIOSim());
      }
    }

    NamedCommands.registerCommand("Elevator Algae Intake",  elevatorSubsystem.goToIntakeAlgae());
    NamedCommands.registerCommand("Elevator Coral Intake", elevatorSubsystem.goToIntakeCoral());
    NamedCommands.registerCommand("Elevator Net", elevatorSubsystem.goToNet());
    NamedCommands.registerCommand("Elevator L4", elevatorSubsystem.goToL4());
    NamedCommands.registerCommand("Elevator L3", elevatorSubsystem.goToL3());
    NamedCommands.registerCommand("Elevator L2", elevatorSubsystem.goToL2());
    NamedCommands.registerCommand("Elevator L1", elevatorSubsystem.goToL1());

    configureBindings();
  }

  private void configureBindings() {    
//
    operator.x().whileTrue(armSubsystem.goToPackage());
    operator.rightBumper().whileTrue(armSubsystem.goToCFeederIntake());
    operator.leftBumper().whileTrue(armSubsystem.goToCFloorIntake());
    operator.povUp().whileTrue(armSubsystem.goToCL4());
    operator.povRight().whileTrue(armSubsystem.goToCL3());
    operator.povDown().whileTrue(armSubsystem.goToCL3());
    operator.povDownLeft().whileTrue(armSubsystem.goToCL1());
    operator.y().whileTrue(armSubsystem.goToAPlace());
    operator.b().whileTrue(armSubsystem.goToAIntakeL3());
    operator.a().whileTrue(armSubsystem.goToAIntakeL2());

    driver.a().whileTrue(NamedCommands.getCommand("Elevator L4"));
    driver.b().whileTrue(NamedCommands.getCommand("Elevator L3"));
    driver.x().whileTrue(NamedCommands.getCommand("Elevator L2"));
    driver.y().whileTrue(NamedCommands.getCommand("Elevator L1"));
    driver.povUp().whileTrue(NamedCommands.getCommand("Elevator Net"));
    driver.povRight().whileTrue(NamedCommands.getCommand("Elevator Coral Intake"));
    driver.povLeft().whileTrue(NamedCommands.getCommand("Elevator Algae Intake"));

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

      driver.start().onTrue(drivetrain.resetPidgeon());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Auto");
  }

}
