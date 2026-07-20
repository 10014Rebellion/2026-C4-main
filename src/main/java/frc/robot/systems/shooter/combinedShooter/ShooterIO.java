// REBELLION 10014

package frc.robot.systems.shooter.combinedShooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterIO {
    @AutoLog
    public static class ShooterInputs {
      public boolean iIsShooterConnected = false;
      public boolean iIsLeader = true;
      public String iShooterControlMode = "";
      public Rotation2d iShooterRotorVelocityRPS = Rotation2d.kZero;
      public Rotation2d iShooterRotorAccelerationRPSS = Rotation2d.kZero;
      public double iShooterMotorVolts = 0.0;
      public double iShooterSupplyCurrentAmps = 0.0;
      public double iShooterStatorCurrentAmps = 0.0;
      public double iShooterTempCelsius = 0.0;
      public Rotation2d iShooterClosedLoopReference = Rotation2d.kZero;
      public Rotation2d iShooterClosedLoopReferenceSlope = Rotation2d.kZero;
    }

    public default void updateInputs(ShooterInputs pInputs) {}

    public default void setPDConstants(double pKP, double pKD) {}

    public default void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {}

    public default void setMotorVelAndAccel(double pVelocityRPS, double pAccelerationRPSS, double pFeedforward) {}

    public default void setMotorVolts(double pVolts) {}

    public default void setMotorAmperage(double pAmps) {}

    public default void stopMotor() {}

    public default void enforceFollower() {}
}