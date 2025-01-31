// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GameSpec.Algae;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Algae.AlgaeIO.AlgaeIOStats;

public class Algae extends SubsystemBase {
  public FunctionalCommand testCommand;

  private final AlgaeIO io;
  public final AlgaeIOStats stats;
  private final ShuffleboardTab algaeShuffleboard;
  protected CANcoder algaeCancoder; //algae extend

  /* Shuffleboard entrys */
  public GenericEntry algaePosition;
  public GenericEntry algaeDirection;
  private GenericEntry algaeVelocity;
  private GenericEntry algaeSupplyCurrent;
  private GenericEntry algaeStatorCurrent;
  private GenericEntry algaeTemp;
  private GenericEntry algaeCommandedPos;
  
  public Algae(AlgaeIO io) {
    this.io = io; 
    this.stats = AlgaeIO.stats;

    this.algaeShuffleboard = Shuffleboard.getTab("Algae");

    algaeVelocity = this.algaeShuffleboard.add("Algae RPM", 0.0).getEntry();
    algaePosition = this.algaeShuffleboard.add("Algae Position", 0.0).getEntry();;
    algaeSupplyCurrent = this.algaeShuffleboard.add("Algae Supply Current", 0.0).getEntry();
    algaeStatorCurrent = this.algaeShuffleboard.add("Algae Stator Current", 0.0).getEntry();
    algaeTemp = this.algaeShuffleboard.add("Algae Temp", 0.0).getEntry();
    algaeCommandedPos = this.algaeShuffleboard.add("Algae Commanded Position", 0.0).getEntry();
  }


  @Override
  public void periodic() {
    io.periodic();

    io.updateStats();
      
    UpdateTelemetry();
  }

  private void UpdateTelemetry() {
    algaeVelocity.setDouble(stats.algaeVelocity);
    algaePosition.setDouble(stats.algaePosition);
    algaeSupplyCurrent.setDouble(stats.SupplyCurrentAmps);
    algaeStatorCurrent.setDouble(stats.TorqueCurrentAmps);
    algaeTemp.setDouble(stats.TempCelsius);
  }

  public Command runToPosition(double position){
    return run(() -> {
        this.algaeCommandedPos.setDouble(position);
        io.setAlgaeMotorControl(position);
    });
  }

  private FunctionalCommand algaeCommand(double position){
    return new FunctionalCommand(
      () -> this.algaeCommandedPos.setDouble(position),
      () -> io.setAlgaeMotorControl(position),
      interrupted -> io.setAlgaeMotorControl(position), 
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
      io.setAlgaeMotorControl(algaeCommandedPos.getDouble(0));
    });
  }
 
  public boolean checkRange(double deadband){
    return (stats.algaePosition >= algaeCommandedPos.getDouble(0) - deadband) && 
           (stats.algaePosition <= algaeCommandedPos.getDouble(0) + deadband);
  }

  @Override
  public void simulationPeriodic() {
  }
}