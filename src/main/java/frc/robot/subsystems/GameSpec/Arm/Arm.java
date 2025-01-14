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
  private static final double Package = Math.toRadians(0);

  private static final double coral_Feader_Intake = Math.toRadians(50);
  private static final double coral_Floor_Intake = Math.toRadians(20);
  private static final double coral_Level1 = Math.toRadians(25);
  private static final double coral_Level2 = Math.toRadians(35);
  private static final double coral_Level3 = Math.toRadians(40);
  private static final double coral_Level4 = Math.toRadians(50);
  
  private static final double algae_Place = Math.toRadians(40);
  private static final double algae_Intake_Level2 = Math.toRadians(180);
  private static final double algea_Intake_Level3 = Math.toRadians(-180);

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
      () -> checkRange(.1),
      this);
  }
//
  public Command goToPackage(){
    return armCommand(Package);
  }
  public Command goToCFeederIntake(){
    return armCommand(coral_Feader_Intake);
  }
  public Command goToCFloorIntake(){
    return armCommand(coral_Floor_Intake);
  }
  public Command goToCL4(){
    return armCommand(coral_Level4);
  }
  public Command goToCL3(){
    return armCommand(coral_Level3);
  }
  public Command goToCL2(){
    return armCommand(coral_Level2);
  }
  public Command goToCL1(){
    return armCommand(coral_Level1);
  }
  public Command goToAIntakeL2(){
    return armCommand(algae_Intake_Level2);
  }
  public Command goToAIntakeL3(){
    return armCommand(algea_Intake_Level3);
  }
  public Command goToAPlace(){
    return armCommand(algae_Place);
  }

  public boolean checkRange(double deadband){
    return (stats.armPosition >= armCommandedPos.getDouble(0) - deadband) && 
           (stats.armPosition <= armCommandedPos.getDouble(0) + deadband);
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