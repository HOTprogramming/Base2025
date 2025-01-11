// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOStats;

public class Elevator extends SubsystemBase {
  public enum ElevatorState{
    L4,
    L3,
    L2,
    L1,
    ZERO
  }

  public FunctionalCommand testCommand;
  public ElevatorState elevatorState;

  private final ElevatorIO io;
  public final ElevatorIOStats stats;
  private final ShuffleboardTab elevatorShuffleboard;

  /* Shuffleboard entrys */
  private GenericEntry elevatorVelocity;
  private GenericEntry elevatorSupplyCurrent;
  private GenericEntry elevatorStatorCurrent;
  private GenericEntry elevatorTemp;

  private GenericEntry shuffKP;
  private GenericEntry shuffKS;
  private GenericEntry shuffKV;
  public GenericEntry elevatorPosition;
  public GenericEntry elevatorDirection;
  public double simelevatorPosition = 0.0;
  public double elevatorCommandedPos;
  

  public Elevator(ElevatorIO io) {
    this.io = io; 

    this.elevatorState = ElevatorState.ZERO;

    this.stats = ElevatorIO.stats;

    this.elevatorShuffleboard = Shuffleboard.getTab("Elevator");

    elevatorVelocity = this.elevatorShuffleboard.add("Elevator RPM", 0.0).getEntry();
    elevatorPosition = this.elevatorShuffleboard.add("Elevator Position", 0.0).getEntry();;
    elevatorSupplyCurrent = this.elevatorShuffleboard.add("Elevator Supply Current", 0.0).getEntry();
    elevatorStatorCurrent = this.elevatorShuffleboard.add("Elevator Stator Current", 0.0).getEntry();
    elevatorTemp = this.elevatorShuffleboard.add("Elevator Temp", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    io.updateStats();
      
    UpdateTelemetry();
  }

  private void UpdateTelemetry() {
    elevatorVelocity.setDouble(stats.elevatorVelocity);
    elevatorPosition.setDouble(stats.elevatorPosition);
    elevatorSupplyCurrent.setDouble(stats.SupplyCurrentAmps);
    elevatorStatorCurrent.setDouble(stats.TorqueCurrentAmps);
    elevatorTemp.setDouble(stats.TempCelsius);
  }

  public Command runToPosition(double position){
    return run(() -> {
        io.setElevatorMotorControl(position);
    });
  }

  public Command stop(){
    return run(() -> {
        io.stop();
    });  
}

  public Command setPID() {
    return runOnce(() -> {io.setPID(
      this.shuffKP.getDouble(0.0),
      this.shuffKS.getDouble(0.0), 
      this.shuffKV.getDouble(0.0));});
  }

  @Override
  public void simulationPeriodic() {
    io.periodic();
  }
}