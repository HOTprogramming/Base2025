// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private final TurretStats ioStats = new TurretStats();
  private final ShuffleboardTab turretShuffleboard;
  private final TurretKraken m_TurretKraken = TurretKraken.getInstance();
 
  private double currentPosition; 
  private double targetRotations; 
  
  
  /* Shuffleboard entrys */
  private GenericEntry turretPosition;
  private GenericEntry turretVelocity;
  private GenericEntry turretAppliedVolts;
  private GenericEntry turretSupplyCurrent;
  private GenericEntry turretTorqueCurrent;
  private GenericEntry turretTemp;
  private GenericEntry turretEncVelocity; 
  private GenericEntry turretEncPosition; 
  private GenericEntry turretReqPosition; 
  private GenericEntry turretPositioninDegrees;
  private GenericEntry turretState; 
  

  /* a Generic class for handling the IO stats generated in the Kraken module  */  
  class TurretStats {
    //Stats for the turret/Expel Motor  
    public boolean MotorConnected = true;    
    public double turretPositionRads = 0.0;
    public double turretVelocityRps = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretSupplyCurrentAmps = 0.0;
    public double turretTorqueCurrentAmps = 0.0;
    public double turretTempCelsius = 0.0;      
    public double turretCanCoderPositionRads = 0.0;
    public double turretCanCoderVelocityRps = 0.0;
}

  /** Constructor - Creates a new turretSubsystem. */
  public Turret() {

    //Define Suffleboard Tab for this Subsystem 
    this.turretShuffleboard = Shuffleboard.getTab("Turret");
    turretVelocity = this.turretShuffleboard.add("Velocity-Motor (rot per s)", 0.0).getEntry();
    turretPosition = this.turretShuffleboard.add("Position-Motor (rot)", 0.0).getEntry();
    turretPositioninDegrees = this.turretShuffleboard.add("Position-Turret (degree)", 0.0).getEntry();
    turretSupplyCurrent = this.turretShuffleboard.add("Supply Current", 0.0).getEntry();
    turretTorqueCurrent = this.turretShuffleboard.add("Torque Current", 0.0).getEntry();
    turretTemp = this.turretShuffleboard.add("Motor Temp", 0.0).getEntry();
    turretAppliedVolts = this.turretShuffleboard.add("Applied Volts",0.0).getEntry();
    turretReqPosition = this.turretShuffleboard.add("Position-Target (rot):",0.0).getEntry();
    turretEncVelocity = this.turretShuffleboard.add("Velocity-Enc (rot per s)", 0.0).getEntry();
    turretEncPosition = this.turretShuffleboard.add("Position-Enc (rot)", 0.0).getEntry();
    turretState = this.turretShuffleboard.add("State", "").getEntry();
  }


  @Override  
  /* This method will be called once per scheduler run */
  public void periodic() {
    //go update the signal data 
    m_TurretKraken.updateStats(ioStats);
     
    //Update Dashboard with Telementry Data 
    UpdateTelemetry();
  
  }

  /*********************************************************
   * Update the Shuffleboard with Motor/Cancoder Statistics 
   ********************************************************/
  private void UpdateTelemetry() {
    turretVelocity.setDouble(roundTo1Decimal(ioStats.turretVelocityRps));
    turretPosition.setDouble(roundTo1Decimal(ioStats.turretPositionRads));
    turretSupplyCurrent.setDouble(roundTo1Decimal(ioStats.turretSupplyCurrentAmps));
    turretTorqueCurrent.setDouble(roundTo1Decimal(ioStats.turretTorqueCurrentAmps));
    turretAppliedVolts.setDouble(roundTo1Decimal(ioStats.turretAppliedVolts));
    turretTemp.setDouble(roundTo1Decimal(ioStats.turretTempCelsius));
    turretEncPosition.setDouble(roundTo1Decimal(ioStats.turretCanCoderPositionRads));
    turretEncVelocity.setDouble(roundTo1Decimal(ioStats.turretCanCoderVelocityRps)); 
    turretPositioninDegrees.setDouble(roundTo1Decimal(ioStats.turretCanCoderPositionRads*TurretConstants.kDegreePerRotation));
  
    //save to use later 
    currentPosition = ioStats.turretCanCoderPositionRads;

    if(checkOnTarget()) {
       turretState.setString("On Target");
    } else {
      turretState.setString("Moving");
    }
  }

  //Utility function to round to 1 decimal 
  private double roundTo1Decimal(double number) {
    return Math.round(number * 10.0)/10.0;
  }

   /**
   * Check to see if turret is aligned with target 
   *
   * @return boolean - true on target
   */
  public boolean checkOnTarget() { 
    boolean retFlag = false;  
    double tolerance = 0.5; 
    
    double lowerBound = targetRotations - tolerance;
    double upperBound = targetRotations + tolerance; 
  
    //Test to see if position is within tolerance 
    if(currentPosition >= lowerBound && currentPosition <= upperBound) {
      retFlag = true; 
    } 

    return retFlag;  
  }

 /**
   * Move turret a requested number of rotations 
   *
   * @return nothing 
   */
  private void zeroTurretPosition() {
    m_TurretKraken.runPosition(0.0);
    turretReqPosition.setDouble(0.0);
  }

   /**
   * Move turret a requested number of rotations 
   *
   * @return nothing 
   */
  private void moveTurretPosition(double requestedRotations) {
    //Calculate how much to move turret based on current position  
    //  start with where we are and add how far we want to move   
    targetRotations = currentPosition + requestedRotations;

    //if targetRotations will move past the softlimit - reverse direction 180 degrees 
    //first - determine which direction we are moving 
    if(targetRotations < 0.0) {
      if(targetRotations <= TurretConstants.kLeftSoftLimit) {
        //move will go past softlimit - change move to spin 360 degrees the other way
         targetRotations += TurretConstants.kFullRotation; 
      }  
    } else {
      if(targetRotations >= TurretConstants.kRightSoftLimit ) {
        //move will go past softlimit - change move to spin 360 degress the other way
        targetRotations -= TurretConstants.kFullRotation; 
      }
    }

    m_TurretKraken.runPosition(targetRotations);
    turretReqPosition.setDouble(targetRotations);
  }

   /**
   * Move turret by requested number of degrees 
   *
   * @return nothing 
   */
  private void moveTurretbyDegrees(double requestedDegrees) {
    //Convert degrees into rotations   
    double requestedRotations = requestedDegrees * TurretConstants.kRotationPerDegree;
    //Then call normal routine using rotations 
    moveTurretPosition(requestedRotations);
  }


  /**************************************************************
  * Commands 
  **************************************************************/
  /**
   * Set command to turret
   *
   * @return a command
   */
  public Command moveByRotations(double requestedRotations) {
    return runOnce(
        () -> {
          /* one-time action goes here */
          moveTurretPosition(requestedRotations);
        });
  }

  /**
   * Set command to turret
   *
   * @return a command
   */
  public Command moveByDegrees(double requestedDegrees) {
    return runOnce(
        () -> {
          /* one-time action goes here */
          moveTurretbyDegrees(requestedDegrees);
        });
  }

  /**
   * Set command to turret
   *
   * @return a command
   */
  public Command zeroTurret() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          zeroTurretPosition();
        });
  }


}