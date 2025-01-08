// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Drivetrain.DriveSim;

public class RobotContainer {
  public Drive drivetrain;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        this.drivetrain = new Drive(new DriveKraken());
      }
      case DEVBOT -> {}
      case SIMBOT -> {
        this.drivetrain = new Drive(new DriveSim());
      }
    }


    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
