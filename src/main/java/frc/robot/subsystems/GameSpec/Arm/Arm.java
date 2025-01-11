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
  private static final double Package = Math.toRadians(30);

  private static final double coral_Feader_Intake = Math.toRadians(30);
  private static final double coral_Floor_Intake = Math.toRadians(40);
  private static final double coral_Level1 = Math.toRadians(50);
  private static final double coral_Level2 = Math.toRadians(60);
  private static final double coral_Level3 = Math.toRadians(70);
  private static final double coral_Level4 = Math.toRadians(80);
  
  private static final double algae_Place = Math.toRadians(90);
  private static final double algae_Intake_Level2 = Math.toRadians(100);
  private static final double algea_Intake_Level3 = Math.toRadians(110);

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
  private GenericEntry armCommandedPos;
  
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
    armPosition.setDouble(stats.armPosition);
    armSupplyCurrent.setDouble(stats.SupplyCurrentAmps);
    armStatorCurrent.setDouble(stats.TorqueCurrentAmps);
    armTemp.setDouble(stats.TempCelsius);
  }

  public FunctionalCommand armCommand(double position){
    return new FunctionalCommand(
      () -> this.armCommandedPos.setDouble(position),
      () -> io.setArmMotorControl(position),
      interrupted -> io.setArmMotorControl(position), 
      () -> checkRange(.25),
      this);
  }

  public Command goToL4(){
    return armCommand(1);
  }


  public boolean checkRange(double deadband){
    // return (stats.armPosition >= armCommandedPos.getDouble(0) - deadband) && 
    //        (stats.armPosition <= armCommandedPos.getDouble(0) + deadband);
    return false;
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