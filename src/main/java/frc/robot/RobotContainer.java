// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.io.IOException;


import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveSim;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Drivetrain.DriveKraken;


public class RobotContainer {

  private SendableChooser<String> chooser = new SendableChooser<>();
  private Command autoCommand;
  private String autoString;

  private Drive drivetrain;
  private Elevator elevator;
  
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    //RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.

    if(!Utils.isSimulation()){
        drivetrain = new Drive(new DriveKraken());
    } else {
        drivetrain = new Drive(new DriveSim());
    }

    elevator = new Elevator();

    chooser.setDefaultOption("Auto", "Auto");
    SmartDashboard.putData(chooser);

    configureBindings();
  }

  private void configureBindings() {   
      drivetrain.setDefaultCommand
      (drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.015 ? -driver.getRightX() : 0);
        }
      ));
      
      operator.a().onTrue(elevator.goToPackage());
      operator.b().onTrue(elevator.goToL3());
  }

  /**
   * Pre-generates auton path and pulls auton starting position. 
   * This prevents a small delay at the start of auton where the robot has to read in the auto file.
   */
  public void updateAutonCommand() {
    if (autoString != null) {
      if (chooser.getSelected() != autoString) {
        autoString = chooser.getSelected();
        autoCommand = new PathPlannerAuto(autoString);
        try {
          drivetrain.setAutonStartPose(
            PathPlannerAuto
              .getPathGroupFromAutoFile(autoString)
              .get(0)
              .getStartingDifferentialPose()
          );
        } catch (IOException e) {
          System.err.println("No Auto File");
        } catch (ParseException e) {
          System.err.println("Auton File Corrupted");
        } catch (IndexOutOfBoundsException e) {
          System.err.println("no auto Paths");
        }
      }
    } else {
      autoString = chooser.getSelected();
    }
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}