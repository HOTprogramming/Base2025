// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GameSpec.Climber;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Climber.ClimberIO.ClimberIOStats;

public class Climber extends SubsystemBase {
  public FunctionalCommand testCommand;

  private final ClimberIO io;
  public final ClimberIOStats stats;
  private final ShuffleboardTab climberShuffleboard;

  /* Shuffleboard entrys */
  public GenericEntry climberPosition;
  public GenericEntry climberDirection;
  private GenericEntry climberVelocity;
  private GenericEntry climberSupplyCurrent;
  private GenericEntry climberStatorCurrent;
  private GenericEntry climberTemp;
  private GenericEntry climberCommandedPos;
  
  public Climber(ClimberIO io) {
    this.io = io; 
    this.stats = ClimberIO.stats;

    this.climberShuffleboard = Shuffleboard.getTab("climber");

    climberVelocity = this.climberShuffleboard.add("climber RPM", 0.0).getEntry();
    climberPosition = this.climberShuffleboard.add("climber Position", 0.0).getEntry();;
    climberSupplyCurrent = this.climberShuffleboard.add("climber Supply Current", 0.0).getEntry();
    climberStatorCurrent = this.climberShuffleboard.add("climber Stator Current", 0.0).getEntry();
    climberTemp = this.climberShuffleboard.add("climber Temp", 0.0).getEntry();
    climberCommandedPos = this.climberShuffleboard.add("climber Commanded Position", 0.0).getEntry();
  }


  @Override
  public void periodic() {
    io.periodic();

    io.updateStats();
      
    UpdateTelemetry();
  }

  private void UpdateTelemetry() {
    climberVelocity.setDouble(stats.climberVelocity);
    climberPosition.setDouble(stats.climberPosition);
    climberSupplyCurrent.setDouble(stats.SupplyCurrentAmps);
    climberStatorCurrent.setDouble(stats.TorqueCurrentAmps);
    climberTemp.setDouble(stats.TempCelsius);
  }

  public Command runToPosition(double position){
    return run(() -> {
        this.climberCommandedPos.setDouble(position);
        io.setClimberMotorControl(position);
    });
  }

  private FunctionalCommand climberCommand(double position){
    return new FunctionalCommand(
      () -> this.climberCommandedPos.setDouble(position),
      () -> io.setClimberMotorControl(position),
      interrupted -> io.setClimberMotorControl(position), 
      () -> checkRange(.1),
      this);
  }


  public Command stop(){
    return run(() -> {
        io.stop();
    });  
  }

  public Command hold(){
    return run(() -> {
      io.setClimberMotorControl(climberCommandedPos.getDouble(0));
    });
  }
 
  public boolean checkRange(double deadband){
    return (stats.climberPosition >= climberCommandedPos.getDouble(0) - deadband) && 
           (stats.climberPosition <= climberCommandedPos.getDouble(0) + deadband);
  }

  @Override
  public void simulationPeriodic() {
  }
}