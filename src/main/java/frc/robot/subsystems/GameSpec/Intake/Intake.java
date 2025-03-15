// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GameSpec.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Intake.IntakeIO.IntakeIOStats;
import frc.robot.subsystems.GameSpec.Manipulator.ManipulatorConstants;

public class Intake extends SubsystemBase {
  public FunctionalCommand testCommand;

  private final IntakeIO io;
  public final IntakeIOStats stats;
  private final ShuffleboardTab intakeShuffleboard;

  /* Shuffleboard entrys */
  public GenericEntry intakePosition;
  public GenericEntry intakeDirection;
  private GenericEntry intakeVelocity;
  private GenericEntry intakeSupplyCurrent;
  private GenericEntry intakeStatorCurrent;
  private GenericEntry intakeTemp;
  private GenericEntry blackRollerVoltage;
  private GenericEntry orangeRollerVoltage;
  private GenericEntry intakeCancoderPosition;
  private GenericEntry IntakeCancoderVelocity;
  public GenericEntry intakeCommandedPos;
  public GenericEntry beamBreakStatus;
  
  public Intake(IntakeIO io) {
    this.io = io; 
    this.stats = IntakeIO.stats;

    this.intakeShuffleboard = Shuffleboard.getTab("Intake");

    intakeVelocity = this.intakeShuffleboard.add("Intake RPM", 0.0).getEntry();
    intakePosition = this.intakeShuffleboard.add("Intake Position", 0.0).getEntry();;
    intakeSupplyCurrent = this.intakeShuffleboard.add("Intake Supply Current", 0.0).getEntry();
    intakeStatorCurrent = this.intakeShuffleboard.add("Intake Stator Current", 0.0).getEntry();
    intakeTemp = this.intakeShuffleboard.add("Intake Temp", 0.0).getEntry();
    blackRollerVoltage = this.intakeShuffleboard.add("Black Roller Voltage", 0.0).getEntry();
    orangeRollerVoltage = this.intakeShuffleboard.add("Orange Roller Voltage", 0.0).getEntry();
    intakeCancoderPosition = this.intakeShuffleboard.add("Intake Cancoder Position", 0.0).getEntry();
    IntakeCancoderVelocity = this.intakeShuffleboard.add("Intake Cancoder Speed", 0.0).getEntry();
    intakeCommandedPos = this.intakeShuffleboard.add("Arm Commanded Position", 0.0).getEntry();
    beamBreakStatus = this.intakeShuffleboard.add("BeamBreak", false).getEntry();
  }


  @Override
  public void periodic() {
    io.periodic();

    io.updateStats();
      
    UpdateTelemetry();
  }

  private void UpdateTelemetry() {
    intakeVelocity.setDouble(stats.intakeVelocity);
    intakePosition.setDouble(stats.intakePosition);
    intakeSupplyCurrent.setDouble(stats.SupplyCurrentAmps);
    intakeStatorCurrent.setDouble(stats.TorqueCurrentAmps);
    intakeTemp.setDouble(stats.TempCelsius);
    intakeCancoderPosition.setDouble(stats.intakeCancoderPosition);
    IntakeCancoderVelocity.setDouble(stats.intakeCancoderVelocity);
    beamBreakStatus.setBoolean(io.beambreak.get());
  }

  public FunctionalCommand intakeCommand(double position, double orangeVoltage, double blackVoltage){
    return new FunctionalCommand(
      () ->{
      this.intakeCommandedPos.setDouble(position);
      this.orangeRollerVoltage.setDouble(orangeVoltage);
      this.blackRollerVoltage.setDouble(blackVoltage);
      },
      () -> {
      io.setIntakeMotorControl(position);
      io.setIntakeSpinMotorControl(0.0, 0.0);
      
      },
      interrupted -> {
      io.setIntakeMotorControl(position);
      io.setIntakeSpinMotorControl(orangeVoltage, blackVoltage);
      io.setIntakeSpinMotorControl(0.0, 0.0);
      }, 
      () -> checkRange(5),
      this);
  }

  public Command goToPackage(){
    return intakeCommand(IntakeConstants.intakePackage, 0.0, 0.0);
  }

  public Command clearance(){
    return intakeCommand(IntakeConstants.intakeClearance, 0.0, 0.0);
  }

  public Command bump(){
    return intakeCommand(IntakeConstants.intakeBump, 0.0, 0.0);
  }

  public Command deploy(){
    return run(() -> {
      intakeCommandedPos.setDouble(IntakeConstants.intakeGround);
      io.setIntakeMotorControl(IntakeConstants.intakeGround);
      io.setIntakeSpinMotorControl(8, 10);
     }); 
  }

  /**
   * @return: false if nothing in the beambreak, true if it detects
   */

  public boolean getBeamBreak(){
    return io.beambreak.get();
  }

  public boolean checkRange(double deadband){
    return (stats.intakePosition >= intakePosition.getDouble(0) - deadband) && 
           (stats.intakePosition <= intakePosition.getDouble(0) + deadband);
  }

  @Override
  public void simulationPeriodic() {
  }
}