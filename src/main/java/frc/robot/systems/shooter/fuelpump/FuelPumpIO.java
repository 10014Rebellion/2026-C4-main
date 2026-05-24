package frc.robot.systems.shooter.fuelpump;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface FuelPumpIO {
    @AutoLog
    public static class FuelPumpInputs {
      public boolean iIsFuelPumpLeader = true;
      public boolean iIsFuelPumpConnected = false;
      public String iFuelPumpControlMode = "";
      public Rotation2d iFuelPumpVelocityRPS = Rotation2d.kZero;
      public double iFuelPumpAccelerationRPSS = 0.0;
      public double iFuelPumpMotorVolts = 0.0;
      public double iFuelPumpSupplyCurrentAmps = 0.0;
      public double iFuelPumpStatorCurrentAmps = 0.0;
      public double iFuelPumpTempCelsius = 0.0;
      public Rotation2d iFuelPumpVelocityGoal = Rotation2d.kZero;
    }

    public default void setPDConstants(int pSlot, double pKP, double pKD) {}

    public default void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {}

    public default void enforceFollower() {}

    public default void updateInputs(FuelPumpInputs pInputs) {}

    public default void setMotorVelocity(Rotation2d pVelocityRPS, double pFeedforward) {}

    public default void setMotorVolts(double pVolts) {}

    public default void stopMotor() {}

}
