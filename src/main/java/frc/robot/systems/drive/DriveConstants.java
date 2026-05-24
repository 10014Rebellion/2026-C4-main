// REBELLION 10014

package frc.robot.systems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotConstants;

public class DriveConstants {

    ///////////////////// DRIVE BASE \\\\\\\\\\\\\\\\\\\\\\\
    public static final double kRobotWidthXMeters = Units.inchesToMeters(27.75);
    public static final double kRobotWidthYMeters = Units.inchesToMeters(32.75);
    public static final double kTrackWidthXMeters = Units.inchesToMeters(25.0); // Track Width (front to front / back to back)
    public static final double kTrackWidthYMeters = Units.inchesToMeters(30.0); // Wheelbase (Left to left / right to right)
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0)
    };
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kModuleTranslations);

    public static final double kDrivebaseRadiusMeters = Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);

    /* DRIVEBASE CONSTRAINTS */
    public static final double kMaxLinearSpeedMPS = 4.0; // TODO: TUNE ME
    public static final double kMaxLinearAccelerationMPSS = 15.5; // TODO: TUNE ME

    public static final double kMaxRotationSpeedRadiansPS = kMaxLinearSpeedMPS / kDrivebaseRadiusMeters; // TODO: TUNE ME
    public static final double kMaxRotationAccelRadiansPS = Math.toRadians(5800); // TODO: TUNE ME
 
    public static final double kMaxAzimuthAngularRadiansPS = 4.5 * 2 * Math.PI; // TODO: TUNE ME

    /* Plugged into setpoint generator */
    public static final PathConstraints kAutoConstraints = new PathConstraints(
            kMaxLinearSpeedMPS, kMaxLinearAccelerationMPSS, kMaxRotationSpeedRadiansPS, kMaxRotationAccelRadiansPS);

    public static final PIDConstants kPPTranslationPID = new PIDConstants(10.0, 0.0, 0.5); // TODO: TUNE ME
    public static final PIDConstants kPPRotationPID = new PIDConstants(5.0, 0.0, 0.0); // TODO: TUNE ME

    /* DRIVEBASE TUNING / ODOMETRY / MISC*/
    public static final CANBus kCANBus = new CANBus("underworld"); // Tuned for C3RBERUS!
    public static final boolean isCANFD = false; 
    public static final double kOdometryFrequency = isCANFD ? 250.0 : 100.0;
    static final Lock kOdometryLock = new ReentrantLock(); 

    public static final double kDriftRate = RobotBase.isReal() ? 2.5 : 5.57; // TODO: TUNE ME
    public static final double kInputVoltage = 12.5;
    public static final double kDriveFFAggressiveness = RobotBase.isReal() ? 0.0001 : 0.5;
    public static final double kAzimuthDriveScalar = RobotBase.isReal() ? 0.0 : 0.0;
    public static final double kSkidRatioCap = 1.5; // TODO: TUNE ME
    public static final double kSkidScalar = 1.0; // TODO: TUNE ME
    public static final double kCollisionCapG = 1.95; // TODO: TUNE ME
    public static final double kCollisionScalar = 1.0; // TODO: TUNE ME
    public static final boolean kAccountForTilt = true; 
    public static final double kMinimumTiltFactor = 0.25;
    public static final Rotation2d kTiltCutOffPitch = Rotation2d.fromDegrees(5.0);
    public static final Rotation2d kTiltCutOffRoll = Rotation2d.fromDegrees(5.0);
    public static final double kTrustTime = 1.5; 

    public static final boolean kDoExtraLogging = false;

    ///////////////////// MODULES \\\\\\\\\\\\\\\\\\\\\\\
    /* GENERAL SWERVE MODULE CONSTANTS */
    public static final boolean kTurnMotorInvert = false;
    public static final double kCANCoderToMechanismRatio = 1;
    public static final double kAzimuthMotorGearing = 25.464 / 1.0;
    public static final double kDriveMotorGearing = 5.50 / 1.0;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(1.4175);
    public static final double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;
    public static final double kWheelInertia = 125.0 / 4.0;

    public static final boolean kUseVoltageFeedforward = RobotConstants.isSim();

    public static final double kPeakVoltage = 12.0;

    public static final double kDriveStatorAmpLimit = 85.14; 
    public static final double kDriveFOCAmpLimit = 85.14;
    public static final double kDriveSupplyAmpLimit = 60.0;
    public static final double kDriveSupplyAmpLowerLimit = 45.0;
    public static final double kDriveSupplyAmpLowerLimitTime = 0.5;

    public static final double kAzimuthStatorAmpLimit = 40.0;
    public static final double kAzimuthSupplyAmpLimit = 20.0;
    public static final double kAzimuthFOCAmpLimit = 40.0;

    public static final ModuleControlConfig kModuleControllerConfigs = !RobotConstants.isSim()
        // kV is generally 0 for FOC control, so double check in ModuleIOKraken to see whether kV should be applied
        ? new ModuleControlConfig(
            new PIDController(230.0, 0.0, 0.0), new SimpleMotorFeedforward(1.5, 0.0, 1.0), // DRIVE // TODO: TUNE ME
            /* TORQUE FOC NUMBERS FROM 6328 */
            new PIDController(4000.0, 0.0, 50.0), new SimpleMotorFeedforward(0.0, 0.0, 0.0)) // AZIMUTH // TODO: TUNE ME
        : new ModuleControlConfig(
            new PIDController(0.1, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 3.0, 0.005),
            new PIDController(4.5, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.5));

    public static final double kDriveAggressiveP = !RobotConstants.isSim() ? 400.0 : 0.1;

    /* MODULE SPECIFIC CONSTANTS */
    public static final int kPigeonCANID = 5; // TODO: TUNE ME

    /* If 180 was added, the person who got the offset had the bevel gears on the wrong side when they did it */
    // BEVEL FACING LEFT (it shoulda been facing right tho)
    public static final ModuleHardwareConfig kFrontLeftHardware = new ModuleHardwareConfig(31, 21, 11, 0.155273);

    public static final ModuleHardwareConfig kFrontRightHardware = new ModuleHardwareConfig(32, 22, 12, -0.077393);

    public static final ModuleHardwareConfig kBackLeftHardware = new ModuleHardwareConfig(33, 23, 13, 0.244873);

    public static final ModuleHardwareConfig kBackRightHardware = new ModuleHardwareConfig(34, 24, 14, -0.184814);

    ////////////////////////// RECORDS \\\\\\\\\\\\\\\\\\\\\\\\
    public static record ModuleHardwareConfig(int driveID, int azimuthID, int encoderID, double offset) {}

    public static record ModuleControlConfig(
        PIDController driveController,
        SimpleMotorFeedforward driveFF,
        PIDController azimuthController,
        SimpleMotorFeedforward azimuthFF) {}
}
