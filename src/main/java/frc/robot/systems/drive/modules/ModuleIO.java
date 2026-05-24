// REBELLION 10014

package frc.robot.systems.drive.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleInputs {
        public boolean iIsDriveConnected = false;
        public double iDrivePositionM = 0.0;
        public double iDriveVelocityMPS = 0.0;
        public double iDriveStatorCurrentAmps = 0.0;
        public double iDriveSupplyCurrentAmps = 0.0;
        // public double iDriveTorqueCurrentAmps = 0.0;
        public double iDriveTemperatureCelsius = 0.0;
        public double iDriveMotorVolts = 0.0;

        public boolean iIsAzimuthConnected = false;
        public Rotation2d iAzimuthPosition = new Rotation2d();
        public Rotation2d iAzimuthVelocity = new Rotation2d();
        public double iAzimuthStatorCurrentAmps = 0.0;
        // public double iAzimuthTorqueCurrentAmps = 0.0;
        public double iAzimuthSupplyCurrentAmps = 0.0;
        public double iAzimuthTemperatureCelsius = 0.0;
        public double iAzimuthMotorVolts = 0.0;

        public boolean iIsCancoderConnected = false;
        public Rotation2d iAzimuthAbsolutePosition = new Rotation2d();

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsM = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleInputs inputs) {}

    public default void setDriveVelocity(double velocityMPS, double feedforward, int slot) {}

    public default void setDriveVolts(double volts) {}

    public default void setDriveAmperage(double amps) {}

    public default void setDrivePID(double kP, double kI, double kD, int slot) {}

    public default void setAzimuthVolts(double volts) {}

    public default void setAzimuthAmps(double amps) {}

    public default void setAzimuthPosition(Rotation2d rotation, double feedforward) {}

    public default void resetAzimuthEncoder() {}

    public default void setAzimuthPID(double kP, double kI, double kD) {}
}
