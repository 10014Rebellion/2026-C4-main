// REBELLION 10014

package frc.robot.systems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeRollerIO {
    @AutoLog
    public static class IntakeRollerInputs {
      public boolean iIsIntakeRollerConnected = false;
      public Rotation2d iIntakeRollerRPS = Rotation2d.kZero;
      public double iIntakeRollerAccelerationMPSS = 0.0;
      public double iIntakeRollerMotorVolts = 0.0;
      public double iIntakeRollerSupplyCurrentAmps = 0.0;
      public double iIntakeRollerStatorCurrentAmps = 0.0;
      public double iIntakeRollerTempCelsius = 0.0;
    }

    public default void updateInputs(IntakeRollerInputs pInputs) {}

    public default void setMotorVolts(double pVolts) {}

    public default void stopMotor() {}

}
