package frc.robot.systems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.units.Rot2d;

public class SWMConstants {
    public record EntireShotMapSample(double distanceMeters, Rot2d hoodAngle, Rot2d flywheelSpeed, double timeSeconds) {}


    // distances we trust the shooter to make it in.
    public static final double kMinValidShotDistanceMeters = 1.34; // TODO: TUNE ME
    public static final double kMaxValidShotDistanceMeters = 5.60; // TODO: TUNE ME

    public static final double kMinTofDistanceMeters = 1.38; // TODO: TUNE ME
    public static final double kMaxTofDistanceMeters = 5.68; // TODO: TUNE ME

   // ShooterYawOffset is the fixed yaw of shooter relative to robot forward.
   // Example: shooter points forward -> Rotation2d.kZero
   // Example: shooter points left -> Rotation2d.fromDegrees(90)
   public static final Rotation2d kShooterYawOffset = Rotation2d.kZero;


    // Hood angle tuning table (distance -> hood pitch)
    public static final EntireShotMapSample[] kShotMapSamplesWithToF = {
        new EntireShotMapSample(0.00, Rot2d.fromRev(0.5), Rot2d.fromRev(0.5), 0.00),
    };
}