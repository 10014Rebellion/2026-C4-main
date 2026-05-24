// REBELLION 10014

package frc.lib.math;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
public class GeomUtil {
    /**
     * Creates a pure translating transform
     *
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Translation2d translation) {
        return new Transform2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure rotating transform
     *
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Rotation2d rotation) {
        return new Transform2d(new Translation2d(), rotation);
    }

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    public static Pose2d inverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
    }

    /**
     * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Creates a pure translated pose
     *
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure rotated pose
     *
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Multiplies a twist by a scaling factor
     *
     * @param twist The twist to multiply
     * @param factor The scaling factor for the twist components
     * @return The new twist
     */
    public static Twist2d multiply(Twist2d twist, double factor) {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }

    /**
     * Converts a Pose3d to a Transform3d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform3d toTransform3d(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Converts a Transform3d to a Transform2d
     *
     * @param transform The original transform
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Transform3d transform) {
        return new Transform2d(
            transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
    }
    
    /**
     * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose3d toPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and Z). chain
     *
     * @param speeds The original translation
     * @return The resulting translation
     */
    public static Twist2d toTwist2d(ChassisSpeeds speeds) {
        return new Twist2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Converts a Twist2d to a ChassisSpeeds by extracting two dimensions (Y and Z). chain
     *
     * @param twist The original translation
     * @return The resulting translation
     */
    public static ChassisSpeeds toChassisSpeeds(Twist2d twist, double dt) {
        return new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dtheta / dt);
    }

    public static Rotation2d getSmallestChangeInRotation(Rotation2d rotation1, Rotation2d rotation2) {
        Rotation2d smallestRotMagnitude = Rotation2d.fromRadians(Math.min(
                Math.min(
                        Math.abs(rotation2.minus(rotation1).getRadians()),
                        Math.abs(rotation2
                                .minus(Rotation2d.k180deg)
                                .minus(rotation1)
                                .getRadians())),
                Math.abs(rotation2
                        .minus(rotation1.minus(Rotation2d.k180deg))
                        .getRadians())));
        
        if(smallestRotMagnitude.equals( Rotation2d.fromRadians( Math.abs( rotation2.minus(rotation1).getRadians() ) ) ) ) {
            return rotation2.minus(rotation1);
        } else if(smallestRotMagnitude.equals( ( Rotation2d.fromRadians( Math.abs( rotation2.minus( Rotation2d.k180deg ).minus( rotation1 ).getRadians() ) ) ) ) ) {
            return rotation2.minus(Rotation2d.k180deg).minus(rotation1);
        } else {
            return rotation2.minus(rotation1.minus(Rotation2d.k180deg));
        }
    }

    public static boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, Rotation2d tolerance) {
        if (tolerance.getRadians() > Math.PI) {
            return true;
        }
        double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
        // cos(θ) >= cos(tolerance) means |θ| <= tolerance, for tolerance in [-pi, pi], as pre-checked
        // above.
        return dot > tolerance.getCos();
    }

    public static double hypot(double x, double y) {
        return Math.hypot(x, y);
    }

    public static double hypot(double x, double y, double z) {
        return Math.hypot(Math.hypot(x, y), z);
    }

    public static Pose3d toPose3d(Pose2d pose){
        return new Pose3d(new Translation3d(pose.getTranslation()), new Rotation3d(pose.getRotation()));
    }

}
