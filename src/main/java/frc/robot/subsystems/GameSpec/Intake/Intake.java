// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GameSpec.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Intake.IntakeIO.IntakeIOStats;

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
  private GenericEntry intakeRotationPos;
  private GenericEntry intakeRollerSpeed;
  private GenericEntry intakeCancoderPosition;
  private GenericEntry IntakeCancoderVelocity;
  public GenericEntry intakeCommandedPos;
  
  public Intake(IntakeIO io) {
    this.io = io; 
    this.stats = IntakeIO.stats;

    this.intakeShuffleboard = Shuffleboard.getTab("Intake");

    intakeVelocity = this.intakeShuffleboard.add("Intake RPM", 0.0).getEntry();
    intakePosition = this.intakeShuffleboard.add("Intake Position", 0.0).getEntry();;
    intakeSupplyCurrent = this.intakeShuffleboard.add("Intake Supply Current", 0.0).getEntry();
    intakeStatorCurrent = this.intakeShuffleboard.add("Intake Stator Current", 0.0).getEntry();
    intakeTemp = this.intakeShuffleboard.add("Intake Temp", 0.0).getEntry();
    intakeRotationPos = this.intakeShuffleboard.add("Intake Rotation Position", 0.0).getEntry();
    intakeRollerSpeed = this.intakeShuffleboard.add("Intake Roller Speed", 0.0).getEntry();
    intakeCancoderPosition = this.intakeShuffleboard.add("Intake Cancoder Position", 0.0).getEntry();
    IntakeCancoderVelocity = this.intakeShuffleboard.add("Intake Cancoder Speed", 0.0).getEntry();
    intakeCommandedPos = this.intakeShuffleboard.add("Arm Commanded Position", 0.0).getEntry();
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
  }

  public FunctionalCommand intakeCommand(double position){
    return new FunctionalCommand(
      () -> this.intakeCommandedPos.setDouble(position),
      () -> io.setIntakeMotorControl(position),
      interrupted -> io.setIntakeMotorControl(position), 
      () -> checkRange(5),
      this);
  }

  public Command intakeAlgaeGround(){
    return runOnce(() -> {
      intakeCommandedPos.setDouble(IntakeConstants.intakeGround);
      io.setIntakeMotorControl(IntakeConstants.intakeGround);
  });
  }

  public Command intakeClimberOut(){
    return intakeCommand(IntakeConstants.climberOut);
  }

  public Command runIntakeAlgae(){
    return runOnce(() -> {
      intakeRollerSpeed.setDouble(IntakeConstants.rollerIntakeVoltage);
      io.setIntakeSpinMotorControl(IntakeConstants.rollerIntakeVoltage);
  });
  }

  public Command intakeRollerVoltage(double voltage){
    return runOnce(() -> {
      intakeRollerSpeed.setDouble(voltage);
      io.setIntakeSpinMotorControl(voltage);
  });
  }

  public Command processor(){
    // return intakeCommand(IntakeConstants.intakeGround);
    return runOnce(() -> {
      intakeCommandedPos.setDouble(IntakeConstants.intakeGround);
      intakeRollerSpeed.setDouble(IntakeConstants.rollerIntakeVoltage);
      
      io.setIntakeSpinMotorControl(IntakeConstants.rollerIntakeVoltage);
      io.setIntakeMotorControl(IntakeConstants.intakeGround);
  });
  }

  public Command goToPackage(){
    return runOnce(() -> {
      intakeCommandedPos.setDouble(IntakeConstants.intakePackage);
      io.setIntakeMotorControl(IntakeConstants.intakePackage);
  });
  }

  public Command stop(){
    return run(() -> {
        io.stop();
    });  
  }

  public boolean checkRange(double deadband){
    return (stats.intakePosition >= intakeRotationPos.getDouble(0) - deadband) && 
           (stats.intakePosition <= intakeRotationPos.getDouble(0) + deadband);
  }

  @Override
  public void simulationPeriodic() {
  }
}