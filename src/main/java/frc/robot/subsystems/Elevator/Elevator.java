// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private GenericEntry velocityRpmCancoder;
  private GenericEntry positionCancoder;

  private GenericEntry goalPose;
  private GenericEntry stateName;

  private GenericEntry f_fusedSensorOutOfSync;
  private GenericEntry sf_fusedSensorOutOfSync;
  private GenericEntry f_remoteSensorInvalid;
  private GenericEntry sf_remoteSensorInvalid;

  
  private GenericEntry shuffKP;
  private GenericEntry shuffKS;
  private GenericEntry shuffKV;
  public GenericEntry elevatorPosition;
  public GenericEntry detectedelevatorPos;
  public GenericEntry elevatorDirection;
  public double simelevatorPosition = 0.0;
  public double elevatorCommandedPos;
  public double detectedPos;
  public double getelevatorPos;
  

  public Elevator(ElevatorIO io) {
    this.io = io; 

    this.elevatorState = ElevatorState.ZERO;

    this.stats = ElevatorIO.ioStats;

    this.elevatorShuffleboard = Shuffleboard.getTab("Elevator");

    elevatorVelocity = this.elevatorShuffleboard.add("Elevator RPM", 0.0).getEntry();
    elevatorPosition = this.elevatorShuffleboard.add("Elevator Position", 0.0).getEntry();;
    elevatorSupplyCurrent = this.elevatorShuffleboard.add("Elevator Supply Current", 0.0).getEntry();
    elevatorStatorCurrent = this.elevatorShuffleboard.add("Elevator Stator Current", 0.0).getEntry();
    elevatorTemp = this.elevatorShuffleboard.add("Elevator Temp", 0.0).getEntry();

    velocityRpmCancoder = this.elevatorShuffleboard.add("Elevator RPM Cancoder", 0.0).getEntry();
    positionCancoder = this.elevatorShuffleboard.add("Elevator Pos Cancoder", 0.0).getEntry();

    f_fusedSensorOutOfSync = this.elevatorShuffleboard.add("f_fusedSensorOutOfSync", false).getEntry();
    sf_fusedSensorOutOfSync = this.elevatorShuffleboard.add("sf_fusedSensorOutOfSync", false).getEntry();
    f_remoteSensorInvalid = this.elevatorShuffleboard.add("f_remoteSensorInvalid", false).getEntry();
    sf_remoteSensorInvalid = this.elevatorShuffleboard.add("sf_remoteSensorInvalid", false).getEntry();

    testCommand = new FunctionalCommand(
      this::zero,
      () -> io.reachGoal(1), 
      interrupted -> this.zero(),
      () -> this.checkInRange(.1),
      this);
  }


  @Override
  public void periodic() {

    
    if(elevatorState == ElevatorState.ZERO){
      zero();
    } else if (elevatorState == ElevatorState.L1){
      elevatorCommandedPos = .5;
    } else if (elevatorState == ElevatorState.L2){
      elevatorCommandedPos = 1;
    } else if (elevatorState == ElevatorState.L3){
      elevatorCommandedPos = 1.5;
    } else if (elevatorState == ElevatorState.L4){
      elevatorCommandedPos = 2;
    } else {
      zero();
    }

    if(elevatorState != ElevatorState.ZERO || elevatorState == null){
      io.reachGoal(elevatorCommandedPos);
    }

    io.updateStats(stats);

    SmartDashboard.putNumber("State", elevatorCommandedPos);
      
    UpdateTelemetry();
  }

  public boolean checkInRange(double deadband){
    return (stats.elevatorPosition >= elevatorCommandedPos - deadband) && (stats.elevatorPosition <= elevatorCommandedPos + deadband);
  }

  public void setElevatorState(ElevatorState elevatorState){
    this.elevatorState = elevatorState;
  }

  public void powerElevator(double power){
    io.setElevatorMotorControl(power);
  }

  private void UpdateTelemetry() {
    elevatorVelocity.setDouble(stats.elevatorVelocity);
    elevatorPosition.setDouble(stats.elevatorPosition);
    // detectedelevatorPos.setDouble(detectedPos);
    elevatorSupplyCurrent.setDouble(stats.SupplyCurrentAmps);
    elevatorStatorCurrent.setDouble(stats.TorqueCurrentAmps);
    elevatorTemp.setDouble(stats.TempCelsius);

    velocityRpmCancoder.setDouble(stats.cancoderVelocity);
    positionCancoder.setDouble(stats.cancoderPosition);

    f_fusedSensorOutOfSync.setBoolean(stats.f_fusedSensorOutOfSync);
    sf_fusedSensorOutOfSync.setBoolean(stats.sf_fusedSensorOutOfSync);
    f_remoteSensorInvalid.setBoolean(stats.f_remoteSensorInvalid);
    sf_remoteSensorInvalid.setBoolean(stats.sf_remoteSensorInvalid);
  }

  public void zero(){
    io.setElevatorMotorControl(0);
  }

  public Command setPID() {
    return runOnce(() -> {io.setPID(
      this.shuffKP.getDouble(0.0),
      this.shuffKS.getDouble(0.0), 
      this.shuffKV.getDouble(0.0));});
  }

  @Override
  public void simulationPeriodic() {
    io.simStuff();
  }
}