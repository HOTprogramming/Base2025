// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator.ElevatorIOKraken;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.Elevator;


public class RobotContainer {

  private Elevator m_elevatorSubsystem = null;


  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  FunctionalCommand highCommand;

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPBOT -> {

        m_elevatorSubsystem = new Elevator(new ElevatorIOKraken());
        
      }
      case DEVBOT -> {}
      case SIMBOT -> {
        m_elevatorSubsystem = new Elevator(new ElevatorIOSim());
      }
    }

    configureBindings();
  }

  /* Driver Controller */
  private void configureBindings() {

    m_elevatorSubsystem.setDefaultCommand(
      (m_elevatorSubsystem.run(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.ZERO);;
      }))
    );

    driver.a().whileTrue(
      (m_elevatorSubsystem.run(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L1);
      }))
    );

    driver.b().whileTrue(
      (m_elevatorSubsystem.run(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L2);
      }))
    );

    driver.x().whileTrue(
      (m_elevatorSubsystem.run(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L3);
      }))
    );

    driver.y().whileTrue(
      (m_elevatorSubsystem.run(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L4);
      }))
    );

  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      (m_elevatorSubsystem.run(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L4);
      })).until(() -> m_elevatorSubsystem.checkInRange(.1)),
      new WaitCommand(5),
      (m_elevatorSubsystem.run(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L3);
      })).until(() -> m_elevatorSubsystem.checkInRange(.1)),
      new WaitCommand(5),
      (m_elevatorSubsystem.run(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L2);
      })).until(() -> m_elevatorSubsystem.checkInRange(.1)),
      new WaitCommand(5),
      (m_elevatorSubsystem.run(() -> {
        m_elevatorSubsystem.setElevatorState(ElevatorState.L1);
      }))
    );
  }

}
