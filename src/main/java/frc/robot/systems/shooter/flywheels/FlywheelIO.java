// REBELLION 10014

package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelInputs {
      public boolean iIsFlywheelConnected = false;
      public boolean iIsLeader = true;
      public String iFlywheelControlMode = "";
      public Rotation2d iFlywheelRotorVelocityRPS = Rotation2d.kZero;
      public Rotation2d iFlywheelRotorAccelerationRPSS = Rotation2d.kZero;
      public double iFlywheelMotorVolts = 0.0;
      public double iFlywheelSupplyCurrentAmps = 0.0;
      public double iFlywheelStatorCurrentAmps = 0.0;
      public double iFlywheelTempCelsius = 0.0;
      public Rotation2d iFlywheelClosedLoopReference = Rotation2d.kZero;
      public Rotation2d iFlywheelClosedLoopReferenceSlope = Rotation2d.kZero;
    }

    public default void updateInputs(FlywheelInputs pInputs) {}

    public default void setPDConstants(double pKP, double pKD) {}

    public default void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {}

    public default void setMotorVelAndAccel(double pVelocityRPS, double pAccelerationRPSS, double pFeedforward) {}

    public default void setMotorVolts(double pVolts) {}

    public default void setMotorAmperage(double pAmps) {}

    public default void stopMotor() {}

    public default void enforceFollower() {}
}