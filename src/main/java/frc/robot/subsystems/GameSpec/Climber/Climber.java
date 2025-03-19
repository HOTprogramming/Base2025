// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GameSpec.Climber;

import java.io.PrintStream;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
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
  public GenericEntry climber;
  public GenericEntry climberPosition;
  public GenericEntry climberDirection;
  private GenericEntry climberVelocity;
  private GenericEntry climberSupplyCurrent;
  private GenericEntry climberStatorCurrent;
  private GenericEntry climberTemp;
  private GenericEntry servoVelocity;
  private GenericEntry servoClampCommandedPos;
  private GenericEntry climberVoltage;
  
  public Climber(ClimberIO io) {
    this.io = io; 
    this.stats = ClimberIO.stats;

    this.climberShuffleboard = Shuffleboard.getTab("climber");
    
    climberPosition = this.climberShuffleboard.add("climber Position", 0.0).getEntry();
    climberSupplyCurrent = this.climberShuffleboard.add("climber Supply Current", 0.0).getEntry();
  }


  @Override
  public void periodic() {

    io.periodic();

    io.updateStats();
      
    UpdateTelemetry();
  }

  private void UpdateTelemetry() {
    climberPosition.setDouble(stats.climberPosition);
    climberSupplyCurrent.setDouble(stats.supplyCurrentAmps);
  }

    public void setPower(Double supplier){
        io.setPower(supplier);
    }

    public Command servoLock(){
      return runOnce(() -> {
       io.climberServo.set(ClimberConstants.climberServoLockPos);
       io.climberServo2.set(1 - ClimberConstants.climberServoLockPos);
      });
    }

    public Command ratchetServoPositionClimber(double position, double climberVoltage){
      return runOnce(() -> {
        io.ratchetServo.set(position);
        io.setPower(climberVoltage);
      });
    }

    public Command ratchetServoPosition(double position){
      return runOnce(() -> {
        io.ratchetServo.set(position);
      });
    }

    public Command servoOpen(){
      return run(() -> {
      io.climberServo.set(ClimberConstants.climberServoOpenPos);
      io.climberServo2.set(1.0 - ClimberConstants.climberServoOpenPos);
      });
    }

    /**
   * 
   * @return false if the climber is deployed, true if it isn't
   */
  public boolean checkClimberDeployed(){
    if(stats.climberPosition > ClimberConstants.targetClicks){
      return false;
    }
    else{
      return true;
    }
  }

  /**
   * @return false if climber is at or past softstop
   */
  public boolean checkClimberSoftStop(){
    return stats.climberPosition > ClimberConstants.softStopClicks;
  }

  public Boolean checkClimberPackaged(){
    if(stats.climberPosition < ClimberConstants.packageClicks && stats.climberPosition > 1.0){
      return false;
    }
    else{
      return true;
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}