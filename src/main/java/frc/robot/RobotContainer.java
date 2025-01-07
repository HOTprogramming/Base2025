// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator.ElevatorIOKraken;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.Elevator;


public class RobotContainer {

  private Elevator m_elevatorSubsystem = null;


  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

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


  }

}
