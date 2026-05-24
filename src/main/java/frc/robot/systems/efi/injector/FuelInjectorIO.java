// REBELLION 10014

package frc.robot.systems.efi.injector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface FuelInjectorIO {
    @AutoLog
    public static class FuelInjectorInputs {
      public boolean iIsIntakeRollerConnected = false;
      public Rotation2d iIntakeRollerRPS = Rotation2d.kZero;
      public double iIntakeRollerAccelerationMPSS = 0.0;
      public double iIntakeRollerMotorVolts = 0.0;
      public double iIntakeRollerSupplyCurrentAmps = 0.0;
      public double iIntakeRollerStatorCurrentAmps = 0.0;
      public double iIntakeRollerTempCelsius = 0.0;
    }

    public default void updateInputs(FuelInjectorInputs pInputs) {}

    public default void setMotorVolts(double pVolts) {}

    public default void stopMotor() {}

}
