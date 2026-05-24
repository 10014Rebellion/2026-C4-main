package frc.lib.math;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.game.FieldConstants;

public class AllianceFlipUtil {
  public static double applyX(double pXM) {
    return shouldFlip() ? FieldConstants.kFieldXM - pXM : pXM;
  }

  public static double applyY(double pYM) {
    return shouldFlip() ? FieldConstants.kFieldYM - pYM : pYM;
  }

  public static Translation2d apply(Translation2d pTranslation) {
    return new Translation2d(applyX(pTranslation.getX()), applyY(pTranslation.getY()));
  }

  public static Rotation2d apply(Rotation2d pRotation) {
    return shouldFlip() ? pRotation.rotateBy(Rotation2d.kPi) : pRotation;
  }

  public static Pose2d apply(Pose2d pPose) {
    return shouldFlip()
        ? new Pose2d(apply(pPose.getTranslation()), apply(pPose.getRotation()))
        : pPose;
  }

  public static Translation3d apply(Translation3d pTranslation) {
    return new Translation3d(
        applyX(pTranslation.getX()), applyY(pTranslation.getY()), pTranslation.getZ());
  }

  public static Rotation3d apply(Rotation3d pRotation) {
    return shouldFlip() ? pRotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : pRotation;
  }

  public static Pose3d apply(Pose3d pPose) {
    return new Pose3d(apply(pPose.getTranslation()), apply(pPose.getRotation()));
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}