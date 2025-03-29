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
  public GenericEntry intakeVelocity;
  public GenericEntry intakeDirection;
  public GenericEntry intakeCommandedPos;
  public GenericEntry beamBreakStatus;
  
  public Intake(IntakeIO io) {
    this.io = io; 
    this.stats = IntakeIO.stats;

    this.intakeShuffleboard = Shuffleboard.getTab("Intake");

    intakePosition = this.intakeShuffleboard.add("Intake Position", 0.0).getEntry();
    intakeVelocity = this.intakeShuffleboard.add("Intake Orange Velocity",0.0 ).getEntry();
    intakeCommandedPos = this.intakeShuffleboard.add("Intake Commanded Position", 0.0).getEntry();
    beamBreakStatus = this.intakeShuffleboard.add("BeamBreak", false).getEntry();
  }


  @Override
  public void periodic() {
    io.periodic();

    io.updateStats();
      
    UpdateTelemetry();
  }

  private void UpdateTelemetry() {
    intakePosition.setDouble(stats.intakePosition);
    beamBreakStatus.setBoolean(io.beambreak.get());
    intakeVelocity.setDouble(stats.intakeVelocity);
    
  }

  public FunctionalCommand intakeCommand(double position, double orangeVelocity, double blackVelocity){
    return new FunctionalCommand(
      () ->{
      this.intakeCommandedPos.setDouble(position);
      },
      () -> {
      io.setIntakeMotorControl(position);
      io.setIntakeSpinVelocityControl(orangeVelocity, blackVelocity);
      },
      interrupted -> {
      io.setIntakeMotorControl(position);
      io.setIntakeSpinVelocityControl(orangeVelocity, blackVelocity);
      }, 
      () -> checkRange(5),
      this);
  }

  public Command goToHandoff(){
    return intakeCommand(IntakeConstants.intakeHandoff, 0.0, 0.0);
  }

  public Command clearance(){
    return intakeCommand(IntakeConstants.intakeClearance, 0.0, 0.0);
  }

  public Command handoffAndSpin(){
    return intakeCommand(IntakeConstants.intakeHandoff, -2.0, -2.0);
  }

  public Command bump(){
    return intakeCommand(IntakeConstants.intakeBump, 0.0, 0.0);
  }

  public Command climb(){
    return intakeCommand(IntakeConstants.intakeGround, 0.0, 0.0);
  }

  public Command deploy(){
    return run(() -> {
      intakeCommandedPos.setDouble(IntakeConstants.intakeGround);
      io.setIntakeMotorControl(IntakeConstants.intakeGround);
      // io.setIntakeSpinMotorControl(8, 10);
      io.setIntakeSpinVelocityControl(85.0, 100);
     }); 
  }

  public Command deployAuton(){
    return runOnce(() -> {
      intakeCommandedPos.setDouble(IntakeConstants.intakeGround);
      io.setIntakeMotorControl(IntakeConstants.intakeGround);
      io.setIntakeSpinVelocityControl(85.0, 100);
     }); 
  }


  /**
   * @return: false if nothing in the beambreak, true if it detects
   */

  public boolean getBeamBreak(){
    return io.beambreak.get();
  }

  public boolean checkRange(double deadband){
    return (stats.intakePosition >= intakeCommandedPos.getDouble(0) - deadband) && 
           (stats.intakePosition <= intakeCommandedPos.getDouble(0) + deadband);
  }

  @Override
  public void simulationPeriodic() {
  }
}