package frc.robot.systems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
  @AutoLog
  public static class HoodInputs {
    public boolean iIsHoodConnected = false;
    public String iHoodControlMode = "";
    public Rotation2d iHoodAngle = Rotation2d.kZero;
    public Rotation2d iHoodVelocityRotPS = Rotation2d.kZero;
    public Rotation2d iHoodAccelerationRPSS = Rotation2d.kZero;
    public double iHoodMotorVolts = 0.0;
    public double iHoodSupplyCurrentAmps = 0.0;
    public double iHoodStatorCurrentAmps = 0.0;
    public double iHoodTempCelsius = 0.0;
    public Rotation2d iHoodReferenceValue = Rotation2d.kZero;
    public Rotation2d iHoodReferenceValueSlope = Rotation2d.kZero;

  }

  public default void setPDConstants(double pKP, double pKD) {}

  public default void setMotionMagicConstants(double pCruiseVelRPS, double pMaxAccelRPS2, double pMaxJerkRPS3) {}

  public default void updateInputs(HoodInputs pInputs) {}

  public default void resetPPID() {}

  public default void setMotorPosition(Rotation2d pRotationSP, double pArbFF) {}

  public default void setMotorVolts(double pVolts) {}

  public default void setMotorAmps(double pAmps) {}

  public default void stopMotor() {}
}
