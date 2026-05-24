// REBELLION 10014

package frc.robot.systems.climb;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ClimbIO {
    @AutoLog
    public static class ClimbInputs {
      public boolean iIsClimbConnected = false;
      public double iClimbMotorVolts = 0.0;
      public double iClimbSupplyCurrentAmps = 0.0;
      public double iClimbStatorCurrentAmps = 0.0;
      public double iClimbTempCelsius = 0.0;
      public double iClimbPositionMeters = 0.0;
      public double iClimbReferenceValue = 0.0;
    }

    public default void updateInputs(ClimbInputs pInputs) {}

    public default void setMotorVolts(double pVolts) {}

    public default void setMotorPosition(double pPositionM, double pFeedforward) {}

    public default void setPDConstants(double pKP, double pKD) {}

    public default void stopMotor() {}

    public default void changeClimbNeutralMode(NeutralModeValue pNeutralMode) {}

    
}
