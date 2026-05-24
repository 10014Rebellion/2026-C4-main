// REBELLION 10014

package frc.robot.systems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroInputs {
        public boolean iConnected = false;
        public Rotation2d iYawPosition = new Rotation2d();
        public Rotation2d iYawVelocityPS = Rotation2d.kZero;
        public Rotation2d iPitchPosition = Rotation2d.kZero;
        public Rotation2d iRollPosition = Rotation2d.kZero;
        public double iAccXG = 0.0;
        public double iAccYG = 0.0;
        public double iAccZG = 0.0;

        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroInputs inputs) {}

    public default void resetGyro(Rotation2d rotation) {}

    public default double getAccMagG() {
        return 0.0;
    }
}
