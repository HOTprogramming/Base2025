package frc.util.SubsystemExamples.Sensor;

import com.ctre.phoenix6.CANBus;

import frc.robot.Constants;

public final class ExampleConstants {
    // CAN identification
    public static final int kExampleSensorID = 1;
    public static final CANBus canbus = new CANBus(Constants.canName);

    // Expected Value Range
    public static final double kMaxReading = 2;
    public static final double kMineading = -2;
}
