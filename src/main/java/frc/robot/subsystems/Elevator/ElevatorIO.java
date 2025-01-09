package frc.robot.subsystems.Elevator;

public interface ElevatorIO {
        
        class ElevatorIOStats {
                public boolean elevatorMotorConnected = true;
                public boolean f_fusedSensorOutOfSync;
                public boolean sf_fusedSensorOutOfSync;
                public boolean f_remoteSensorInvalid;
                public boolean sf_remoteSensorInvalid;
                public double elevatorPosition = 0.0;
                public double krakenElevatorPosition = 0.0;
                public double elevatorVelocity = 0.0;
                public double cancoderPosition = 0.0;
                public double cancoderVelocity = 0.0;
                public double elevatorRotorPos = 0.0;
                public double elevatorAppliedVolts = 0.0;
                public double elevatorCurrentAmps = 0.0;

                public double SupplyCurrentAmps = 0.0;
                public double TorqueCurrentAmps = 0.0;
                public double TempCelsius = 0.0;
              }

        ElevatorIOStats ioStats = new ElevatorIOStats();

        /** Update stats */
        default void updateStats(ElevatorIOStats stats) {}

        //apply motion magic control mode
        default void setElevatorMotorControl(double commandedPosition){ 
        }

        default void runVolts(double Volts) {
        }
    
        default void runVelocity(double Rpm, double Feedforward) {
        }
  
        default void setPID(double kP, double kI, double kD) {
        }
    
        default void stop() {
        }
    
        default void runCharacterization(double input) {
        }

        default void reachGoal(double goal){}

        default void simStuff(){}
       
}