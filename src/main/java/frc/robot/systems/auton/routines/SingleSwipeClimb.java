package frc.robot.systems.auton.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.AutoEvent;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.auton.Auton;
import frc.robot.systems.auton.AutonCommands;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.commands.FollowPathCommand;

public class SingleSwipeClimb extends Auton {
    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final double mFirstSwipeSwitchToAlignTime;
    private final double mFirstBeginningTimeout;
    private final boolean mIsMirrored;

    private final double kShotTimeSeconds = 6.5;

    private Pose2d mClimbPose;

    public SingleSwipeClimb(
        AutonCommands pAutos, 
        String pAutoName, 
        String pFirstSwipePathName, 
        double pFirstSwipeAlignTime,
        double pFirstBeginningTimeout,
        Pose2d pClimbPose,
        boolean pIsMirrored) {
        super(pAutos);
        mAutoName = pAutoName;
        mFirstSwipePathName = pFirstSwipePathName;
        mFirstSwipeSwitchToAlignTime = pFirstSwipeAlignTime;
        mFirstBeginningTimeout = pFirstBeginningTimeout;
        mClimbPose = pClimbPose;
        mIsMirrored = pIsMirrored;
    }

    @Override
    public AutoEvent getAuton() {
        AutoEvent auto = new AutoEvent(mAutoName, mAutos);
        Trigger autoActivted = auto.getIsRunningTrigger();

        mAutos.runIntake(
            auto.loggedCondition(
                auto.getName() + "/IntakeWhileInCenter", 
                () -> GameGoalPoseChooser.inCenter(mDriveSS.getPoseEstimate()), 
                false), 
            auto.getName(), 
            auto);

        FollowPathCommand firstSwipePath = 
            followChoreoPath(mFirstSwipePathName, true, auto, mIsMirrored);

        Pose2d lastPoseOfFirstSwipe = mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().size() - 1);

        Trigger firstPathEnded = mAutos.traversePathWithIntakeOutOnly(
            0.1 + mFirstBeginningTimeout,
            firstSwipePath, 
            autoActivted, 
            mFirstSwipePathName, 
            auto);

        Trigger autoAlignShotReadySwipe1 = mAutos.followPathToAutoAlignShoot(
            mDriveSS.getDriveManager().setToGenericAutoAlign(() -> getSwipeEndPose(lastPoseOfFirstSwipe), ConstraintType.LINEAR), 
            firstSwipePath.atTime(mFirstSwipeSwitchToAlignTime), 
            mFirstSwipePathName, 
            auto);

        Trigger fuelToHubHasEndedSwipe1 = mAutos.shootFuelToHub(
            kShotTimeSeconds, 
            autoAlignShotReadySwipe1, 
            mFirstSwipePathName, 
            auto);

        mClimbPose = GameGoalPoseChooser.getClosestClimbPose(mDriveSS.getPoseEstimate());

        Trigger hasClimbEnded = mAutos.goToClimb(
            fuelToHubHasEndedSwipe1, 
            () -> mClimbPose, 
            "/Climb", 
            auto);

        mAutos.resetAndEndAutos(hasClimbEnded, auto);

        return auto;
    }

    private Pose2d getSwipeEndPose(Pose2d pose) {
        return AllianceFlipUtil.apply(
            new Pose2d(
                pose.getX(), pose.getY(),
                GameGoalPoseChooser.turnFromHub(AllianceFlipUtil.apply(pose))
                    .plus((AllianceFlipUtil.shouldFlip()) 
                        ? Rotation2d.k180deg 
                        : Rotation2d.kZero)
        ));
    }
}