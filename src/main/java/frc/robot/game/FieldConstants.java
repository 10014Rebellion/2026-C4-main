// REBELLION 10014

package frc.robot.game;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

// TODO: UPDATE ALL POSES FOR GAME
public class FieldConstants {
    // ALL OF THESE VALUES ARE ASSUMED TO BE FROM 🔵 BLUE SIDE 🔵
    // ALL VALUES ARE FROM: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    // IF NOT INCLUDED THERE, THEY ARE FROM THE CAD: https://cad.onshape.com/documents/8a691e28680da30504859fce/w/c6aa636fb23edb3f1e272fb1/e/5e2b2310531e01f25fd97afd

    public enum FieldType {
        ANDYMARK,
        WELDED
    }

    public static final FieldType kFieldType = FieldType.ANDYMARK;

    private static final double kHubOuterHeightM = inToM(72.0);
    private static final double kHubInnerHeightM = inToM(59.459043);
    private static final double kHubHexOuterOffsetM = inToM(24.2487115); // Offset from center of hub to outer point of a hexagon
    private static final double kHubHexInnerOffsetM = inToM(13.7578975); // Offset from center of hub to inner point of a hexagon
    
    public static final Pose2d kClimbRightPose = new Pose2d(1.353, 3.15, Rotation2d.fromDegrees(0));
    public static final Pose2d kClimbLeftPose = new Pose2d(1.542, 4.147, Rotation2d.fromDegrees(0));

    public static final double kFieldXM = eval(650.12, 651.22);
    public static final double kFieldYM = eval(316.64, 317.69);

    public static final double kStartingLineXM = eval(156.06 + 1, 156.61 + 1); // Added 1 to get it on the very center of the starting line
    public static final double kOutpostCenterYM = eval(25.62, 26.22);

    private static final double kAllianceBorder = eval(181.56, 182.11);
    private static final double kHubXM = kAllianceBorder;
    private static final double kHubYM = kFieldYM / 2.0;

    public static final double kTrenchWallLengthM = inToM(12);
    public static final double kBumpLengthM = inToM(73.0);
    public static final double kTrenchGapLengthM = eval(49.86, 50.67);

    public static final Pose2d kTrenchPoseRight = new Pose2d(
        kAllianceBorder, 
        kTrenchGapLengthM / 2.0, 
        Rotation2d.kZero
    );

    public static final Pose2d kTrenchPoseLeft = new Pose2d(
        kAllianceBorder, 
        kFieldYM - (kTrenchGapLengthM / 2.0), 
        Rotation2d.kZero
    );

    public static final Pose2d kBumpPoseRight = new Pose2d(
        kAllianceBorder, 
        kTrenchGapLengthM + kTrenchWallLengthM + (kBumpLengthM / 2.0), 
        Rotation2d.kZero
    );

    public static final Pose2d kBumpPoseLeft = new Pose2d(
        kAllianceBorder, 
        kFieldYM - (kTrenchGapLengthM + kTrenchWallLengthM + (kBumpLengthM / 2.0)), 
        Rotation2d.kZero
    );

    public static final Pose2d kHubPose = new Pose2d(new Translation2d(kHubXM, kHubYM), Rotation2d.kZero);
    public static final Pose3d kHubInnerPose = new Pose3d(new Translation3d(kHubXM, kHubYM, kHubInnerHeightM), Rotation3d.kZero);
    public static final Pose3d kHubOuterPose = new Pose3d(new Translation3d(kHubXM, kHubYM, kHubOuterHeightM), Rotation3d.kZero);

    public static final Pose2d kSafeScoringPose = new Pose2d();

    public static final Pose3d[] kHubOuterHexPoints = buildHubHex(kHubOuterPose, kHubHexOuterOffsetM);
    public static final Pose3d[] kHubInnerHexPoints = buildHubHex(kHubInnerPose, kHubHexInnerOffsetM);

    public static final AprilTagFieldLayout kApriltagLayout = kFieldType.equals(FieldType.ANDYMARK) 
        ? AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark) 
        : AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private static Pose3d[] buildHubHex(Pose3d pHubPose, double pHubHexOffsetM) {
        Pose3d[] hex = new Pose3d[6];

        hex[0] = new Pose3d(pHubPose.getX() - pHubHexOffsetM, pHubPose.getY(), pHubPose.getZ(), Rotation3d.kZero);

        for(int i = 1; i < hex.length; i++) {
            hex[i] = hex[i - 1].rotateAround(pHubPose.getTranslation(), new Rotation3d(Rotation2d.fromDegrees(60)));
        }

        return hex;
    }

    private static double eval(double pAndymarkIn, double pWeldedIn) {
        return inToM(kFieldType.equals(FieldType.ANDYMARK) ? pAndymarkIn : pWeldedIn);
    }

    private static double inToM(double pValueIn) {
        return Units.inchesToMeters(pValueIn);
    }
}
