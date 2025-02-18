// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GameSpec.Arm;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Arm.ArmIO.ArmIOStats;

public class Arm extends SubsystemBase {
  //arm range 150 to -150
  public FunctionalCommand testCommand;
  private final ArmIO io;
  public final ArmIOStats stats;
  private final ShuffleboardTab armShuffleboard;

  /* Shuffleboard entrys */
  public GenericEntry armPosition;
  public GenericEntry armDirection;
  private GenericEntry armVelocity;
  private GenericEntry armSupplyCurrent;
  private GenericEntry armStatorCurrent;
  private GenericEntry armTemp;
  public GenericEntry armCommandedPos;
  
  public Arm(ArmIO io) {
    this.io = io; 

    this.stats = ArmIO.stats;

    this.armShuffleboard = Shuffleboard.getTab("Arm");
    
    armVelocity = this.armShuffleboard.add("Arm RPM", 0.0).getEntry();
    armPosition = this.armShuffleboard.add("Arm Position", 0.0).getEntry();;
    armSupplyCurrent = this.armShuffleboard.add("Arm Supply Current", 0.0).getEntry();
    armStatorCurrent = this.armShuffleboard.add("Arm Stator Current", 0.0).getEntry();
    armTemp = this.armShuffleboard.add("Arm Temp", 0.0).getEntry();
    armCommandedPos = this.armShuffleboard.add("Arm Commanded Position", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    io.updateStats();
      
    UpdateTelemetry();
  }

  private void UpdateTelemetry() {
    armVelocity.setDouble(stats.armVelocity);
    armPosition.setDouble(io.arm.getPosition().getValueAsDouble());
    armSupplyCurrent.setDouble(stats.SupplyCurrentAmps);
    armStatorCurrent.setDouble(stats.TorqueCurrentAmps);
    armTemp.setDouble(stats.TempCelsius);
  }

  public FunctionalCommand armCommand(double position){
    return new FunctionalCommand(
      () -> this.armCommandedPos.setDouble(position),
      () -> io.setArmMotorControl(position),
      interrupted -> io.setArmMotorControl(position), 
      () -> checkRange(5),
      this);
  }

  public Command goToPackage(){
    return armCommand(ArmConstants.PackageAngle);
  }

  public Command goToL1(){
    return armCommand(ArmConstants.L1Angle);
  }
  
  public Command goToL2(){
    return armCommand(ArmConstants.L2Angle);
  }

  public Command goToL3(){
    return armCommand(ArmConstants.L3Angle);
  }

  public Command goToL3Short(){
    return armCommand(ArmConstants.L3short);
  }
  
  public Command goToL4Short(){
    return armCommand(ArmConstants.L4Short);
  }

  public Command goToL2Short(){
    return armCommand(ArmConstants.L2Short);
  }

  public Command goToL4(){
    return armCommand(ArmConstants.L4Angle);
  }

  public Command L4Score(){
    return armCommand(ArmConstants.L4Score);
  }

  public Command L3Score(){
    return armCommand(ArmConstants.L3Score);
  }

  public Command L2Score(){
    return armCommand(ArmConstants.L2Score);
  }

  public Command goToFeeder(){
    return armCommand(ArmConstants.FeederAngle);
  }

  public Command horizontal(){
    return armCommand(ArmConstants.Horizontal);
  }

  public Command intakeAlgae(){
    return armCommand(ArmConstants.IntakeAlgae);
  }

  public boolean checkRange(double deadband){
    return (stats.armPosition >= armCommandedPos.getDouble(0) - deadband) && 
           (stats.armPosition <= armCommandedPos.getDouble(0) + deadband);
  }

  public boolean armGreaterThan(double desiredPos, double threshHold){
    if(stats.armPosition > desiredPos - Math.abs(threshHold)){
      System.out.println(true);
      return true;
    }
    else{
      System.out.println(false);
      return false;
    }
  }

  /**
   * 
   * @param tripCurrent
   * @return false if bad
   */
  public boolean armCurrent(double tripCurrent){
    return !(stats.SupplyCurrentAmps > tripCurrent);

  }

  public boolean armLessThan(double desiredPos, double threshHold){
    if(stats.armPosition < desiredPos - Math.abs(threshHold)){
      return true;
    }
    else{
      return false;
    }

  }

  public Command runToPosition(double position){
    return run(() -> {
        io.setArmMotorControl(position);
    });
  }

  public Command stop(){
    return run(() -> {
        io.stop();
    });  
  }

  @Override
  public void simulationPeriodic() {
    io.periodic();
  }
}