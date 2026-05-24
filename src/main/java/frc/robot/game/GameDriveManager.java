// REBELLION 10014

package frc.robot.game;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;

public class GameDriveManager {
    public static enum GameDriveStates {
        HUB_HEADING_ALIGN,
        AUTON_HUB_HEADING_ALIGN,
        LINE_TO_TRENCH,
        LINE_TO_BUMP,
        LINE_TO_O,
        LINE_TO_D,
        DRIVE_TO_SAFE_SCORE,
        LINE_TO_CLIMB;
    }

    public Drive mDrive;

    public GameDriveManager(Drive pDrive) {
        this.mDrive = pDrive;
    }

    public Command getSetGameDriveStateCmd(GameDriveStates pGameDriveState) {
        switch (pGameDriveState) {
            case HUB_HEADING_ALIGN:
                return mDrive.getDriveManager().setToGenericHeadingAlign(
                    () -> GameGoalPoseChooser.turnFromHub(mDrive.getPoseEstimate()),
                    () -> GameGoalPoseChooser.getHub());
            case AUTON_HUB_HEADING_ALIGN:
                return mDrive.getDriveManager().setToGenericHeadingAlignAuton(
                    () -> GameGoalPoseChooser.turnFromHub(mDrive.getPoseEstimate()),
                    () -> GameGoalPoseChooser.getHub());
            case LINE_TO_TRENCH:
                return mDrive.getDriveManager().setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getClosestTrench(mDrive.getPoseEstimate()),
                    () -> Rotation2d.kZero,
                    () -> 1.0,
                    () -> false);
            case LINE_TO_BUMP:
                return mDrive.getDriveManager().setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getClosestBump(mDrive.getPoseEstimate()),
                    () -> Rotation2d.kZero,
                    () -> 1.0,
                    () -> false);
            case LINE_TO_O:
                return mDrive.getDriveManager().setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getO(),
                    () -> Rotation2d.kZero,
                    () -> 1.0,
                    () -> false);
            case LINE_TO_D:
                return mDrive.getDriveManager().setToGenericLineAlign(
                    () -> GameGoalPoseChooser.getD(),
                    () -> Rotation2d.kZero,
                    () -> 1.0,
                    () -> false);
            case DRIVE_TO_SAFE_SCORE:
                return mDrive.getDriveManager().setToGenericAutoAlign(
                    () -> GameGoalPoseChooser.getSafeScoringPosition(), 
                    ConstraintType.LINEAR);

            case LINE_TO_CLIMB:
                return mDrive.getDriveManager().setToGenericLineAlign(
                    () -> AllianceFlipUtil.apply(FieldConstants.kClimbLeftPose),
                    () -> AllianceFlipUtil.apply(FieldConstants.kClimbLeftPose).getRotation(), 
                    () -> 0.5, 
                    () -> false);
            default:
                return new InstantCommand(() -> DriverStation.reportError(
                        "<<< UNACCOUNTED DRIVE STATE \"" + pGameDriveState.toString() + "\" >>>", true));
        }
    }
}
