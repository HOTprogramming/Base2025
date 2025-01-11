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
import frc.robot.subsystems.Elevator.ElevatorIOReal;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.Elevator;


public class RobotContainer {
  private Elevator elevatorSubsystem = null;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  FunctionalCommand highCommand;

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPBOT -> {

        elevatorSubsystem = new Elevator(new ElevatorIOReal());
        
      }
      case DEVBOT -> {}
      case SIMBOT -> {
        elevatorSubsystem = new Elevator(new ElevatorIOSim());
      }
    }

    configureBindings();
  }

  /* Driver Controller */
  private void configureBindings() {
    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.stop());

    driver.a().whileTrue(elevatorSubsystem.runToPosition(1));
  }

  public Command getAutonomousCommand() {
    return null;
  }

}
