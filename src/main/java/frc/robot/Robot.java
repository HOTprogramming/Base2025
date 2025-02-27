// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj.DataLogManager;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private ShuffleboardTab tab;

  private GenericEntry matchTimeEntry;
  private GenericEntry voltsEntry;
  private GenericEntry ampsEntry;

  private boolean ppConfigured = false;

  private Command initAuto;

  public Robot() {
    m_robotContainer = new RobotContainer();

    // DataLogManager.start();
    // DriverStation.startDataLog(DataLogManager.getLog(), true);

    
    tab = Shuffleboard.getTab("tab");
    matchTimeEntry = tab.add("Match time",0.0).getEntry();
    voltsEntry = tab.add("Volts",0.0).getEntry();
    ampsEntry = tab.add("Amps",0.0).getEntry();

    initAuto = new PathPlannerAuto("initAuto").ignoringDisable(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    matchTimeEntry.setDouble(DriverStation.getMatchTime());
    voltsEntry.setDouble(RoboRioDataJNI.getVInVoltage());
    ampsEntry.setDouble(RoboRioDataJNI.getVInCurrent());
  }

  @Override
  public void disabledInit() {
    if (!ppConfigured) {
      initAuto.schedule();
      ppConfigured = true;
    }

    
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.updateAutonCommand();
    m_robotContainer.updateLights();
  }

  @Override
  public void disabledExit() {
    initAuto.cancel();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = Commands.waitSeconds(0.01).andThen(m_robotContainer.getAutonomousCommand());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.resetLeds();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
