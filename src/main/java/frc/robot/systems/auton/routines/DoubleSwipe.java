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

public class DoubleSwipe extends Auton {
    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final double mFirstSwipeSwitchToAlignTime;
    private final String mSecondSwipePathName;
    private final double mSecondSwipeSwitchToAlignTime;
    private final double mFirstBeginningTimeout;
    private final boolean mIsMirrored;

    private final double kShotTime1Seconds = 3.0;
    private final double kShotTime2Seconds = 6.5;

    public DoubleSwipe(
        AutonCommands pAutos, 
        String pAutoName, 
        String pFirstSwipePathName,
        double pFirstSwipeSwitchToAlignTime,
        String pSecondSwipePathName, 
        double pSecondSwipeSwitchToAlignTime,
        double pFirstBeginningTimeout,
        boolean pIsMirrored) {
        super(pAutos);
        mAutoName = pAutoName;
        mFirstSwipePathName = pFirstSwipePathName;
        mFirstSwipeSwitchToAlignTime = pFirstSwipeSwitchToAlignTime;
        mSecondSwipePathName = pSecondSwipePathName;
        mSecondSwipeSwitchToAlignTime = pSecondSwipeSwitchToAlignTime;
        mFirstBeginningTimeout = pFirstBeginningTimeout;
        mIsMirrored = pIsMirrored;
    }

    @Override
    public AutoEvent getAuton() {
        AutoEvent auto = new AutoEvent(mAutoName, mAutos);
        Trigger autoActivated = auto.getIsRunningTrigger();

        mAutos.runIntake(
            auto.loggedCondition(
                auto.getName()+"/IntakeWhileInCenter", 
                () -> GameGoalPoseChooser.inCenter(mDriveSS.getPoseEstimate()),
                true), 
            auto.getName(), 
            auto);

        FollowPathCommand firstSwipePath = 
            followChoreoPath(mFirstSwipePathName, true, auto, mIsMirrored);
            
        Pose2d lastPoseOfFirstSwipe = mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().size() - 1);

        Trigger firstPathEnded = mAutos.traversePathWithIntakeOutOnly(
            0.1 + mFirstBeginningTimeout, 
            firstSwipePath, 
            autoActivated, 
            mFirstSwipePathName, 
            auto);

        /* Takes over mid path */
        Trigger autoAlignShotReadySwipe1 = mAutos.followPathToAutoAlignShoot(
            mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> getSwipeEndPose(lastPoseOfFirstSwipe),
                ConstraintType.LINEAR), 
            firstSwipePath.atTime(mFirstSwipeSwitchToAlignTime), 
            mFirstSwipePathName, 
            auto);

        Trigger fuelToHubHasEndedSwipe1 = mAutos.shootFuelToHub(
            kShotTime1Seconds, 
            autoAlignShotReadySwipe1, 
            mFirstSwipePathName, 
            auto);

        FollowPathCommand secondSwipePath = 
            followChoreoPath(mSecondSwipePathName, false, auto, mIsMirrored);
        Pose2d lastPoseOfSecondSwipe = mAutos.getTraj(mSecondSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mSecondSwipePathName).get().getPathPoses().size() - 1);

        Trigger secondPathHasEnded = mAutos.traversePathWithIntakeOutOnly(
            0.0,
            secondSwipePath, 
            fuelToHubHasEndedSwipe1, 
            mSecondSwipePathName, 
            auto);

        /* Takes over mid path */
        Trigger autoAlignShotReadySwipe2 = mAutos.followPathToAutoAlignShoot(
            mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> getSwipeEndPose(lastPoseOfSecondSwipe), 
                ConstraintType.LINEAR), 
            secondSwipePath.atTime(mSecondSwipeSwitchToAlignTime), 
            mSecondSwipePathName, 
            auto);

        Trigger fuelToHubHasEndedSwipe2 = mAutos.shootFuelToHub(
            kShotTime2Seconds, 
            autoAlignShotReadySwipe2, 
            mSecondSwipePathName, 
            auto);

        mAutos.resetAndEndAutos(fuelToHubHasEndedSwipe2, auto);

        return auto;
    }

    private Pose2d getSwipeEndPose(Pose2d pose) {
        return AllianceFlipUtil.apply(
            new Pose2d(
                pose.getX(), pose.getY(),
                GameGoalPoseChooser.turnFromHub(AllianceFlipUtil.apply(pose))
                    .plus((AllianceFlipUtil.shouldFlip()) 
                        ? Rotation2d.k180deg 
                        : Rotation2d.kZero)));
    }
    
}