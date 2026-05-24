// REBELLION 10014

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class RobotConstants {
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    // Mode.REPLAY;
    public static final boolean kTuningMode = !DriverStation.isFMSAttached();
    public static final CANBus kSubsystemsCANBus = new CANBus("overworld"); // Tuned for C3RBERUS!
    public static final double kPeriodicSec = 0.02;

    public static final boolean isSim() {
        return !(kCurrentMode.equals(Mode.REAL) || kCurrentMode.equals(Mode.REPLAY)); 
    }

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class ShooterPositionConstants {
        public static final Transform3d kLeftShooterOffset = new Transform3d(-0.0875919, 0.15875, 0.5551932, Rotation3d.kZero); // Z is height from ground
        public static final Transform3d kMiddleShooterOffset = new Transform3d(-0.0875919, 0, 0.5551932, Rotation3d.kZero); // Z is height from ground
        public static final Transform3d kRightShooterOffset = new Transform3d(-0.0875919, -0.15875, 0.5551932, Rotation3d.kZero); // Z is height from ground
    }

    public static class DashboardConstants {
        public static final boolean kDashboardEnabled = true;
        public static final String kDashboardPath = "dashboard";
        public static final int kDashboardPort = 5800;

        public static final boolean kDeployServerEnabled = false;
        public static final String kDeployServerPath = "";
        public static final int kDeployServerPort = 5801;
    }
}
