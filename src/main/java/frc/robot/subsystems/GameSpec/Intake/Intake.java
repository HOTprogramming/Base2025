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
  private GenericEntry intakeCommandedPos;
  
  public Intake(IntakeIO io) {
    this.io = io; 
    this.stats = IntakeIO.stats;

    this.intakeShuffleboard = Shuffleboard.getTab("Intake");

    intakeVelocity = this.intakeShuffleboard.add("Intake RPM", 0.0).getEntry();
    intakePosition = this.intakeShuffleboard.add("Intake Position", 0.0).getEntry();;
    intakeSupplyCurrent = this.intakeShuffleboard.add("Intake Supply Current", 0.0).getEntry();
    intakeStatorCurrent = this.intakeShuffleboard.add("Intake Stator Current", 0.0).getEntry();
    intakeTemp = this.intakeShuffleboard.add("Intake Temp", 0.0).getEntry();
    intakeCommandedPos = this.intakeShuffleboard.add("Intake Commanded Position", 0.0).getEntry();
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
  }

  public Command runToPosition(double position){
    return run(() -> {
        this.intakeCommandedPos.setDouble(position);
        io.setIntakeMotorControl(position);
    });
  }

  private FunctionalCommand intakeCommand(double position){
    return new FunctionalCommand(
      () -> this.intakeCommandedPos.setDouble(position),
      () -> io.setIntakeMotorControl(position),
      interrupted -> io.setIntakeMotorControl(position), 
      () -> checkRange(.1),
      this);
  }

  public Command goToL4(){
    return intakeCommand(IntakeConstants.l4Height);
  }

  public Command goToL3(){
    return intakeCommand(IntakeConstants.l3Height);
  }

  public Command goToL2(){
    return intakeCommand(IntakeConstants.l2Height);
  }

  public Command goToL1(){
    return intakeCommand(IntakeConstants.l1Height);
  }

  public Command goToNet(){
    return intakeCommand(IntakeConstants.netHeight);
  }

  public Command goToIntakeCoral(){
    return intakeCommand(IntakeConstants.intakeCoralHeight);
  }

  public Command goToIntakeAlgae(){
    return intakeCommand(IntakeConstants.intakeAlgaeHeight);
  }

  public Command managerIntakeTest(){
    System.out.println("intakeworks");
    return runOnce(() -> {});
  }

  public Command stop(){
    return run(() -> {
        io.stop();
    });  
  }

  public Command hold(){
    return run(() -> {
      io.setIntakeMotorControl(intakeCommandedPos.getDouble(0));
    });
  }
 
  public boolean checkRange(double deadband){
    return (stats.intakePosition >= intakeCommandedPos.getDouble(0) - deadband) && 
           (stats.intakePosition <= intakeCommandedPos.getDouble(0) + deadband);
  }

  @Override
  public void simulationPeriodic() {
  }
}