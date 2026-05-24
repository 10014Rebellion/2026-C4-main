package frc.robot.systems.auton.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.AutoEvent;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.game.FieldConstants;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.auton.Auton;
import frc.robot.systems.auton.AutonCommands;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.commands.FollowPathCommand;

public class SingleSwipe extends Auton {
    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final double mFirstSwipeSwitchToAlignTime;
    private final double mFirstBeginningTimeout;

    private final double kShotTimeSeconds = 6.5;

    private final boolean mIsMirrored;

    public SingleSwipe(
        AutonCommands pAutos, 
        String pAutoName, 
        String pFirstSwipePathName, 
        double pFirstSwipeAlignTime,
        double pFirstBeginningTimeout,
        boolean pIsMirrored) {
        super(pAutos);
        mAutoName = pAutoName;
        mFirstSwipePathName = pFirstSwipePathName;
        mFirstSwipeSwitchToAlignTime = pFirstSwipeAlignTime;
        mFirstBeginningTimeout = pFirstBeginningTimeout;
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
            mDriveSS.getDriveManager().setToGenericAutoAlignWithGeneratorReset(() -> getSwipeEndPose(lastPoseOfFirstSwipe), ConstraintType.LINEAR), 
            firstSwipePath.atTime(mFirstSwipeSwitchToAlignTime), 
            mFirstSwipePathName, 
            auto);

        Trigger fuelToHubHasEndedSwipe1 = mAutos.shootFuelToHub(
            kShotTimeSeconds, 
            autoAlignShotReadySwipe1, 
            mFirstSwipePathName, 
            auto);

        mAutos.resetAndEndAutos(fuelToHubHasEndedSwipe1, auto);

        return auto;
    }

    private Pose2d getSwipeEndPose(Pose2d pose) {
        return AllianceFlipUtil.apply(
            new Pose2d(
                pose.getX(), (!mIsMirrored) ? pose.getY() : FieldConstants.kFieldYM - pose.getY(),
                GameGoalPoseChooser.turnFromHub(AllianceFlipUtil.apply(mDriveSS.getPoseEstimate()))
        ));
    }
}