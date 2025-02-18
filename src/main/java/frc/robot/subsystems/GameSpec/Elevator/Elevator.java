// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GameSpec.Elevator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Elevator.ElevatorIO.ElevatorIOStats;

public class Elevator extends SubsystemBase {
  public FunctionalCommand testCommand;

  private final ElevatorIO io;
  public final ElevatorIOStats stats;
  private final ShuffleboardTab elevatorShuffleboard;

  /* Shuffleboard entrys */
  public GenericEntry elevatorPosition;
  public GenericEntry elevatorDirection;
  private GenericEntry elevatorVelocity;
  private GenericEntry elevatorSupplyCurrent;
  private GenericEntry elevatorStatorCurrent;
  private GenericEntry elevatorTemp;
  private GenericEntry elevatorCommandedPos;
  
  public Elevator(ElevatorIO io) {
    this.io = io; 
    this.stats = ElevatorIO.stats;

    this.elevatorShuffleboard = Shuffleboard.getTab("Elevator");

    elevatorVelocity = this.elevatorShuffleboard.add("Elevator RPM", 0.0).getEntry();
    elevatorPosition = this.elevatorShuffleboard.add("Elevator Position", 0.0).getEntry();;
    elevatorSupplyCurrent = this.elevatorShuffleboard.add("Elevator Supply Current", 0.0).getEntry();
    elevatorStatorCurrent = this.elevatorShuffleboard.add("Elevator Stator Current", 0.0).getEntry();
    elevatorTemp = this.elevatorShuffleboard.add("Elevator Temp", 0.0).getEntry();
    elevatorCommandedPos = this.elevatorShuffleboard.add("Elevator Commanded Position", 0.0).getEntry();
  }


  @Override
  public void periodic() {
    io.periodic();

    io.updateStats();
      
    UpdateTelemetry();
  }

  private void UpdateTelemetry() {
    elevatorVelocity.setDouble(stats.elevatorVelocity);
    elevatorPosition.setDouble(io.elevator.getPosition().getValueAsDouble());
    elevatorSupplyCurrent.setDouble(stats.SupplyCurrentAmps);
    elevatorStatorCurrent.setDouble(stats.TorqueCurrentAmps);
    elevatorTemp.setDouble(stats.TempCelsius);
  }

  public Command runToPosition(double position){
    return run(() -> {
        this.elevatorCommandedPos.setDouble(position);
        io.setElevatorMotorControl(position);
    });
  }

  private FunctionalCommand elevatorCommand(double position){
    return new FunctionalCommand(
      () -> this.elevatorCommandedPos.setDouble(position),
      () -> io.setElevatorMotorControl(position),
      interrupted -> io.setElevatorMotorControl(position), 
      () -> checkRange(1.0),
      this);
  }

  public Command goToL4(){
    return elevatorCommand(ElevatorConstants.L4Height);
  }

  public Command goToL4Long(){
    return elevatorCommand(ElevatorConstants.L4LongHeight);
  }

  public Command goToL3(){
    return elevatorCommand(ElevatorConstants.L3Height);
    // return elevatorCommand(30.0);
  }

  public Command goToL2(){
    return elevatorCommand(ElevatorConstants.L2Height);
  }

  public Command goToL1(){
    return elevatorCommand(ElevatorConstants.L1Height);
  }

  public Command L4Score(){
    return elevatorCommand(ElevatorConstants.L4ScoreHeight);
  }

  public Command L3Score(){
    return elevatorCommand(ElevatorConstants.L3ScoreHeight);
  }

  public Command L2Score(){
    return elevatorCommand(ElevatorConstants.L2ScoreHeight);
  }

  public Command goToFeeder(){
    return elevatorCommand(ElevatorConstants.FeederHeight);
  }

  public Command goToPackage(){
    return elevatorCommand(ElevatorConstants.PackageHeight);
    // return elevatorCommand(25.0);
  }

  public Command goToHighAlgae(){
    return elevatorCommand(ElevatorConstants.highAlgae);
  }

  public Command goToLowAlgae(){
    return elevatorCommand(ElevatorConstants.lowAlgae);
  }

  public Command goToHP(){
    return elevatorCommand(ElevatorConstants.HPHeight);
  }

  public Command goToBarge(){
    return elevatorCommand(ElevatorConstants.BargeHeight);
  }

  public Command climbDown(){
    return elevatorCommand(ElevatorConstants.climbHeight);
  }

  public boolean elevatorClimbHeight(){
    if(stats.elevatorPosition < ElevatorConstants.L1Height - 1.0){
      return true;
    }
    else{
      return false;
    }
  }
  

  public Command managerElevatorTest(){
    System.out.println("elevatorworks");
    return runOnce(() -> {});
  }

  public Command stop(){
    return run(() -> {
        io.stop();
    });  
  }

  public Command hold(){
    return run(() -> {
      io.setElevatorMotorControl(elevatorCommandedPos.getDouble(0));
    });
  }
 
  public boolean checkRange(double deadband){
    return (stats.elevatorPosition >= elevatorCommandedPos.getDouble(0) - deadband) && 
           (stats.elevatorPosition <= elevatorCommandedPos.getDouble(0) + deadband);
  }

  public boolean elevatorGreaterThan(double desiredPos, double threshHold){
    if(stats.elevatorPosition > desiredPos - Math.abs(threshHold)){
      System.out.println(true);
      return true;
    }
    else{
      System.out.println(false);
      return false;
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}