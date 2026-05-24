package frc.robot.systems.efi.sensors;

import frc.lib.hardware.HardwareRecords.CANRangeConfiguration;

public class SensorConstants {
    public static final int leftCANRangeID = 49;
    public static final int centerCANRangeID = 48;
    public static final int rightCANRangeID = 47;
    
    public static final double kPositionTolerance = 0.002;
    public static final double kFuelCutoff = 0.04;

    public static CANRangeConfiguration leftCANRangeConfiguration = new CANRangeConfiguration(leftCANRangeID, kFuelCutoff, kPositionTolerance);
    public static CANRangeConfiguration centerCANRangeConfiguration = new CANRangeConfiguration(centerCANRangeID, kFuelCutoff, kPositionTolerance);
    public static CANRangeConfiguration rightCANRangeConfiguration = new CANRangeConfiguration(rightCANRangeID, kFuelCutoff, kPositionTolerance);
}