
package frc.robot.systems.efi.sensors;

import org.littletonrobotics.junction.AutoLog;

/** The intake subsystem's hardware interface. */
public interface SensorIO {
  @AutoLog
  public static class SensorInputs {
    public double distanceFromSensor = 0.0;
    public double distanceFromSensorStdDev = 0.0;
    public boolean hasObject = false;
    public boolean isSensorConnected = false;
    public double ambience = 0.0;
    public double signalStrength = 0.0;
    public boolean isMeasurementHealthGood = false;
    public double measurementTime = 0.0;
  }

  public default void updateInputs(SensorInputs inputs) {};

  public default boolean getCANRangeValue() { return false;};

  public default void setSensorConfigs(double pTolerance, double pCutoff) {}
}