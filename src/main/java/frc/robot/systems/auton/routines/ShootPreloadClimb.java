package frc.robot.systems.auton.routines;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.AutoEvent;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.auton.Auton;
import frc.robot.systems.auton.AutonCommands;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;

public class ShootPreloadClimb extends Auton{

    private final String mAutoName;
    private final Supplier<Pose2d> mShootPose;

    private final double kShotTimeSeconds = 6.5;
    
    public ShootPreloadClimb(
        AutonCommands pAutos, 
        String pAutoName,
        Supplier<Pose2d> pShootPose){
        super(pAutos);
        mAutoName = pAutoName;
        mShootPose = pShootPose;
    }

    @Override
    public AutoEvent getAuton() {
        AutoEvent auto = new AutoEvent(mAutoName, mAutos);
        Trigger autoActivated = auto.getIsRunningTrigger();

        autoActivated
            .onTrue(new InstantCommand(() -> mDriveSS.setPose(GameGoalPoseChooser.getPreloadStartingPosition())));

        Trigger autoAlignShotReadySwipe1 = mAutos.followPathToAutoAlignShoot(
            mDriveSS.getDriveManager().setToGenericAutoAlign(mShootPose, ConstraintType.LINEAR), 
            autoActivated.debounce(0.5),
            mAutoName, 
            auto);

        Trigger fuelToHubHasEndedSwipe1 = mAutos.shootFuelToHub(
            kShotTimeSeconds, 
            autoAlignShotReadySwipe1, 
            mAutoName, 
            auto);

        Pose2d climbPose = AllianceFlipUtil.apply(GameGoalPoseChooser.getClosestClimbPose(mDriveSS.getPoseEstimate()));

        Trigger hasClimbEnded = mAutos.goToClimb(
            fuelToHubHasEndedSwipe1, 
            () -> AllianceFlipUtil.apply(climbPose), 
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