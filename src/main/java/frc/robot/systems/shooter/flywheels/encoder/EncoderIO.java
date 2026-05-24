package frc.robot.systems.shooter.flywheels.encoder;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface EncoderIO {
    @AutoLog
    public static class EncoderInputs {
      public boolean iIsEncoderConnected = false;
      public Rotation2d iEncoderPositionRot = Rotation2d.kZero;
      public Rotation2d iEncoderVelocityRPS = Rotation2d.kZero;
      public String iEncoderMagnetHealth = "";
    }

    public default void updateInputs(EncoderInputs pInputs) {}
    public default void setPosition(Rotation2d pNewRot) {}
}
